#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB-CAN Patient Monitor Dashboard
Đọc CAN Extended ID từ STM32 qua USB-CAN CANable SLCAN, giải mã VITAL/LOADCELL,
xử lý cảnh báo tại Python, và hiển thị web realtime.

CAN Extended ID 29-bit:
  bit 28..26 : msg_type   1=VITAL, 2=LOADCELL
  bit 25..24 : wing       0=A, 1=B
  bit 23..16 : room       0..255
  bit 15..8  : bed        0..255
  bit 7..0   : patient    0..255

VITAL data:
  byte 0: HR low
  byte 1: HR high
  byte 2: SpO2
  byte 3: Temp x10 low
  byte 4: Temp x10 high
  byte 5: Battery
  byte 6: Seq
  byte 7: Status

LOADCELL data:
  byte 0: Volume ml low
  byte 1: Volume ml high
  byte 2: Drops/min low
  byte 3: Drops/min high
  byte 4: Seq
  byte 5: Status
  byte 6: Reserved
  byte 7: Reserved

Cài đặt:
  pip install python-can fastapi uvicorn

Chạy Windows CANable SLCAN:
  python patient_can_web_dashboard.py --channel COM27 --bitrate 125000 --web

Mở web:
  http://127.0.0.1:8000
"""

from __future__ import annotations

import argparse
import asyncio
import json
import threading
import time
from dataclasses import dataclass, asdict, field
from typing import Any, Dict, List, Optional

import can

MSG_TYPE_VITAL = 1
MSG_TYPE_LOADCELL = 2

WING_CODE_TO_TEXT = {
    0: "A",
    1: "B",
}

# =========================
# NGƯỠNG CẢNH BÁO
# Có thể chỉnh theo yêu cầu đồ án/bác sĩ
# =========================

# Sinh hiệu
HR_LOW_CRITICAL = 45
HR_LOW_WARNING = 55
HR_HIGH_WARNING = 110
HR_HIGH_CRITICAL = 130

SPO2_WARNING = 94
SPO2_CRITICAL = 90

TEMP_WARNING = 38.0
TEMP_CRITICAL = 39.0

BATTERY_WARNING = 20
BATTERY_CRITICAL = 10

# Loadcell / dịch truyền
INFUSION_EMPTY_WARNING_ML = 80
INFUSION_EMPTY_CRITICAL_ML = 30
DROPS_LOW_WARNING = 5
DROPS_HIGH_WARNING = 90
DROPS_HIGH_CRITICAL = 120

# Mất kết nối
OFFLINE_WARNING_SEC = 8
OFFLINE_CRITICAL_SEC = 15

# Xác nhận ngưng chảy: DPM = 0 liên tục bao lâu mới cảnh báo
INFUSION_STOP_CONFIRM_SEC = 20

# Giảm báo động giả: cần cùng cảnh báo lặp lại N lần mới hiện critical/warning
ALERT_CONFIRM_COUNT = 2


@dataclass
class Vital:
    hr: int
    spo2: int
    temp: float
    battery: int
    seq: int
    status: int
    updated_at: float


@dataclass
class Loadcell:
    volume_ml: int
    drops_per_min: int
    seq: int
    status: int
    updated_at: float


@dataclass
class Alert:
    level: str          # "warning" hoặc "critical"
    code: str
    message: str
    value: Any
    timestamp: float


@dataclass
class PatientState:
    wing: str
    room: int
    bed: int
    patient: int
    vital: Optional[Vital] = None
    loadcell: Optional[Loadcell] = None
    first_seen: float = field(default_factory=time.time)
    last_seen: float = field(default_factory=time.time)
    zero_dpm_since: Optional[float] = None
    alerts: List[Alert] = field(default_factory=list)
    alert_counter: Dict[str, int] = field(default_factory=dict)

    @property
    def key(self) -> str:
        return f"{self.wing}-R{self.room:03d}-B{self.bed:03d}-P{self.patient:03d}"


patients: Dict[str, PatientState] = {}
patients_lock = threading.Lock()

websocket_clients = set()
web_loop: Optional[asyncio.AbstractEventLoop] = None


def parse_can_id(can_id: int):
    msg_type = (can_id >> 26) & 0x07
    wing_code = (can_id >> 24) & 0x03
    room = (can_id >> 16) & 0xFF
    bed = (can_id >> 8) & 0xFF
    patient = can_id & 0xFF
    wing = WING_CODE_TO_TEXT.get(wing_code, f"X{wing_code}")
    return msg_type, wing, room, bed, patient


def make_patient_key(wing: str, room: int, bed: int, patient: int) -> str:
    return f"{wing}-R{room:03d}-B{bed:03d}-P{patient:03d}"


def get_or_create_patient(wing: str, room: int, bed: int, patient: int) -> PatientState:
    key = make_patient_key(wing, room, bed, patient)
    if key not in patients:
        patients[key] = PatientState(wing=wing, room=room, bed=bed, patient=patient)
    return patients[key]


def parse_vital_data(data: bytes) -> Vital:
    if len(data) < 8:
        raise ValueError("VITAL DLC must be 8")

    hr = data[0] | (data[1] << 8)
    spo2 = data[2]

    temp_x10 = data[3] | (data[4] << 8)
    if temp_x10 >= 32768:
        temp_x10 -= 65536

    return Vital(
        hr=hr,
        spo2=spo2,
        temp=temp_x10 / 10.0,
        battery=data[5],
        seq=data[6],
        status=data[7],
        updated_at=time.time(),
    )


def parse_loadcell_data(data: bytes) -> Loadcell:
    if len(data) < 8:
        raise ValueError("LOADCELL DLC must be 8")

    volume_ml = data[0] | (data[1] << 8)
    drops_per_min = data[2] | (data[3] << 8)

    return Loadcell(
        volume_ml=volume_ml,
        drops_per_min=drops_per_min,
        seq=data[4],
        status=data[5],
        updated_at=time.time(),
    )


def new_alert(level: str, code: str, message: str, value: Any) -> Alert:
    return Alert(
        level=level,
        code=code,
        message=message,
        value=value,
        timestamp=time.time(),
    )


def calculate_raw_alerts(p: PatientState) -> List[Alert]:
    now = time.time()
    alerts: List[Alert] = []

    # Mất kết nối
    offline_sec = now - p.last_seen
    if offline_sec >= OFFLINE_CRITICAL_SEC:
        return [new_alert("critical", "OFFLINE_CRITICAL", "Mất kết nối bệnh nhân", round(offline_sec, 1))]
    if offline_sec >= OFFLINE_WARNING_SEC:
        alerts.append(new_alert("warning", "OFFLINE_WARNING", "Node lâu chưa gửi dữ liệu", round(offline_sec, 1)))

    # Sinh hiệu
    if p.vital:
        v = p.vital

        # HR = 0 thường là cảm biến chưa đo được, không cảnh báo nhịp tim thấp ngay.
        if v.hr > 0:
            if v.hr < HR_LOW_CRITICAL:
                alerts.append(new_alert("critical", "HR_LOW_CRITICAL", "Nhịp tim rất thấp", v.hr))
            elif v.hr < HR_LOW_WARNING:
                alerts.append(new_alert("warning", "HR_LOW_WARNING", "Nhịp tim thấp", v.hr))
            elif v.hr > HR_HIGH_CRITICAL:
                alerts.append(new_alert("critical", "HR_HIGH_CRITICAL", "Nhịp tim rất cao", v.hr))
            elif v.hr > HR_HIGH_WARNING:
                alerts.append(new_alert("warning", "HR_HIGH_WARNING", "Nhịp tim cao", v.hr))

        # SpO2 = 0 thường là chưa có kết quả hợp lệ từ MAX30100, không cảnh báo SpO2.
        if v.spo2 > 0:
            if v.spo2 < SPO2_CRITICAL:
                alerts.append(new_alert("critical", "SPO2_CRITICAL", "SpO2 rất thấp", v.spo2))
            elif v.spo2 < SPO2_WARNING:
                alerts.append(new_alert("warning", "SPO2_WARNING", "SpO2 thấp", v.spo2))

        # Temp = 0 đang là giá trị mặc định trong node watch, không cảnh báo.
        if v.temp > 0:
            if v.temp >= TEMP_CRITICAL:
                alerts.append(new_alert("critical", "TEMP_CRITICAL", "Sốt cao", v.temp))
            elif v.temp >= TEMP_WARNING:
                alerts.append(new_alert("warning", "TEMP_WARNING", "Sốt", v.temp))

        if v.battery > 0:
            if v.battery <= BATTERY_CRITICAL:
                alerts.append(new_alert("critical", "BATTERY_CRITICAL", "Pin rất yếu", v.battery))
            elif v.battery <= BATTERY_WARNING:
                alerts.append(new_alert("warning", "BATTERY_WARNING", "Pin yếu", v.battery))

    # Loadcell
    if p.loadcell:
        l = p.loadcell

        if l.volume_ml <= INFUSION_EMPTY_CRITICAL_ML:
            alerts.append(new_alert("critical", "INFUSION_EMPTY_CRITICAL", "Dịch truyền gần cạn", l.volume_ml))
        elif l.volume_ml <= INFUSION_EMPTY_WARNING_ML:
            alerts.append(new_alert("warning", "INFUSION_EMPTY_WARNING", "Dịch truyền sắp hết", l.volume_ml))

        if l.drops_per_min >= DROPS_HIGH_CRITICAL:
            alerts.append(new_alert("critical", "DROPS_HIGH_CRITICAL", "Tốc độ nhỏ giọt quá nhanh", l.drops_per_min))
        elif l.drops_per_min >= DROPS_HIGH_WARNING:
            alerts.append(new_alert("warning", "DROPS_HIGH_WARNING", "Tốc độ nhỏ giọt nhanh", l.drops_per_min))
        elif 0 < l.drops_per_min <= DROPS_LOW_WARNING:
            alerts.append(new_alert("warning", "DROPS_LOW_WARNING", "Tốc độ nhỏ giọt thấp", l.drops_per_min))

        if l.volume_ml > INFUSION_EMPTY_WARNING_ML and l.drops_per_min == 0:
            if p.zero_dpm_since is not None:
                zero_sec = now - p.zero_dpm_since
                if zero_sec >= INFUSION_STOP_CONFIRM_SEC:
                    alerts.append(new_alert("critical", "INFUSION_STOP", "Dịch truyền có thể đã ngưng chảy", round(zero_sec, 1)))

    return alerts


def debounce_alerts(p: PatientState, raw_alerts: List[Alert]) -> List[Alert]:
    """
    Chống báo giả: cùng một code phải xuất hiện ALERT_CONFIRM_COUNT lần liên tiếp.
    Offline critical thì cho hiện ngay.
    """
    raw_codes = {a.code for a in raw_alerts}

    # reset counter cho cảnh báo đã hết
    for code in list(p.alert_counter.keys()):
        if code not in raw_codes:
            p.alert_counter[code] = 0

    confirmed: List[Alert] = []
    for a in raw_alerts:
        p.alert_counter[a.code] = p.alert_counter.get(a.code, 0) + 1
        if a.code.startswith("OFFLINE") or p.alert_counter[a.code] >= ALERT_CONFIRM_COUNT:
            confirmed.append(a)

    return confirmed


def update_alerts(p: PatientState):
    raw = calculate_raw_alerts(p)
    p.alerts = debounce_alerts(p, raw)


def update_patient_from_can(msg: can.Message) -> Optional[PatientState]:
    if not msg.is_extended_id:
        return None

    if len(msg.data) < 8:
        return None

    msg_type, wing, room, bed, patient_id = parse_can_id(msg.arbitration_id)

    with patients_lock:
        p = get_or_create_patient(wing, room, bed, patient_id)
        p.last_seen = time.time()

        if msg_type == MSG_TYPE_VITAL:
            p.vital = parse_vital_data(bytes(msg.data))
        elif msg_type == MSG_TYPE_LOADCELL:
            loadcell = parse_loadcell_data(bytes(msg.data))
            p.loadcell = loadcell

            if loadcell.drops_per_min == 0:
                if p.zero_dpm_since is None:
                    p.zero_dpm_since = time.time()
            else:
                p.zero_dpm_since = None
        else:
            return None

        update_alerts(p)
        return p


def patient_to_dict(p: PatientState) -> dict:
    return {
        "key": p.key,
        "wing": p.wing,
        "room": p.room,
        "bed": p.bed,
        "patient": p.patient,
        "vital": asdict(p.vital) if p.vital else None,
        "loadcell": asdict(p.loadcell) if p.loadcell else None,
        "first_seen": p.first_seen,
        "last_seen": p.last_seen,
        "alerts": [asdict(a) for a in p.alerts],
    }


def print_patient(p: PatientState):
    vital_text = "-"
    if p.vital:
        vital_text = (
            f"HR={p.vital.hr} SpO2={p.vital.spo2} "
            f"T={p.vital.temp:.1f}C Bat={p.vital.battery}% Seq={p.vital.seq}"
        )

    loadcell_text = "-"
    if p.loadcell:
        loadcell_text = f"ML={p.loadcell.volume_ml} DPM={p.loadcell.drops_per_min} Seq={p.loadcell.seq}"

    if p.alerts:
        alert_text = " | ".join([f"{a.level.upper()}:{a.message}({a.value})" for a in p.alerts])
    else:
        alert_text = "OK"

    print(f"{p.key} | VITAL: {vital_text} | LOADCELL: {loadcell_text} | ALERT: {alert_text}")


async def broadcast_patient_update(p: PatientState):
    if not websocket_clients:
        return

    payload = json.dumps({
        "type": "patient_update",
        "data": patient_to_dict(p),
    }, ensure_ascii=False)

    dead = []
    for ws in list(websocket_clients):
        try:
            await ws.send_text(payload)
        except Exception:
            dead.append(ws)

    for ws in dead:
        websocket_clients.discard(ws)


def schedule_broadcast(p: PatientState):
    if web_loop is not None:
        asyncio.run_coroutine_threadsafe(broadcast_patient_update(p), web_loop)


def can_reader_thread(interface: str, channel: str, bitrate: int, print_raw: bool):
    bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
    print(f"CAN reader started: interface={interface}, channel={channel}, bitrate={bitrate}")

    while True:
        try:
            msg = bus.recv(timeout=1.0)

            if msg is None:
                # Cập nhật cảnh báo offline mỗi giây
                with patients_lock:
                    for p in patients.values():
                        old_codes = [a.code for a in p.alerts]
                        update_alerts(p)
                        new_codes = [a.code for a in p.alerts]
                        if old_codes != new_codes:
                            print_patient(p)
                            # polling web sẽ tự cập nhật mỗi 1 giây
                continue

            if print_raw:
                print(
                    f"RAW ext={msg.is_extended_id} id=0x{msg.arbitration_id:X} "
                    f"dlc={msg.dlc} data={bytes(msg.data).hex(' ')}"
                )

            p = update_patient_from_can(msg)
            if p:
                print_patient(p)
                # polling web sẽ tự cập nhật mỗi 1 giây

        except KeyboardInterrupt:
            break
        except Exception as exc:
            print("CAN read error:", exc)
            time.sleep(0.5)


def run_web_server(host: str, port: int):
    import uvicorn
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, JSONResponse

    app = FastAPI(title="Patient CAN Dashboard")

    html = r"""
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<title>Patient CAN Dashboard</title>
<style>
    body {
        font-family: Arial, sans-serif;
        background: #f3f4f6;
        margin: 0;
        padding: 20px;
    }
    h1 { margin: 0 0 8px 0; }
    .topbar {
        display: flex;
        gap: 12px;
        align-items: center;
        margin-bottom: 16px;
        flex-wrap: wrap;
    }
    input, select {
        padding: 8px;
        border-radius: 8px;
        border: 1px solid #ccc;
    }
    .summary {
        display: flex;
        gap: 10px;
        flex-wrap: wrap;
        margin-bottom: 16px;
    }
    .summary-card {
        background: white;
        border-radius: 12px;
        padding: 12px 16px;
        box-shadow: 0 1px 6px #0001;
        min-width: 130px;
    }
    .grid {
        display: grid;
        grid-template-columns: repeat(auto-fill, minmax(350px, 1fr));
        gap: 12px;
    }
    .card {
        background: white;
        border-radius: 14px;
        padding: 14px;
        box-shadow: 0 2px 8px #0001;
        border-left: 8px solid #22c55e;
    }
    .warning { border-left-color: #f59e0b; }
    .critical { border-left-color: #ef4444; animation: blink 1s infinite alternate; }
    @keyframes blink { from { box-shadow: 0 0 0 #ef4444; } to { box-shadow: 0 0 14px #ef444477; } }
    .title {
        font-size: 18px;
        font-weight: bold;
        margin-bottom: 8px;
    }
    .row { margin: 5px 0; }
    .muted { color: #666; font-size: 13px; }
    .alert {
        padding: 6px 8px;
        border-radius: 8px;
        margin-top: 5px;
        background: #fee2e2;
        color: #991b1b;
        font-weight: bold;
    }
    .ok {
        color: #15803d;
        font-weight: bold;
    }

    .chart-panel {
        background: white;
        border-radius: 14px;
        padding: 14px;
        box-shadow: 0 2px 8px #0001;
        margin: 16px 0;
    }
    .chart-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(330px, 1fr));
        gap: 12px;
    }
    .chart-box {
        border: 1px solid #e5e7eb;
        border-radius: 12px;
        padding: 10px;
    }
    .chart-box h3 {
        margin: 0 0 8px 0;
        font-size: 15px;
    }
    canvas {
        width: 100%;
        height: 210px;
        display: block;
    }

    .chart-close-btn:hover {
        filter: brightness(0.96);
    }


    .close-chart-btn {
        padding: 9px 14px;
        border-radius: 9px;
        border: 1px solid #ef4444;
        background: #fee2e2;
        color: #991b1b;
        font-weight: bold;
        cursor: pointer;
        white-space: nowrap;
    }
    .close-chart-btn:hover {
        background: #fecaca;
    }

</style>
</head>
<body>
<h1>Patient CAN Dashboard</h1>

<div class="topbar">
    <input id="search" placeholder="Tìm bệnh nhân, phòng, dãy..." oninput="render()">
    <select id="filterLevel" onchange="render()">
        <option value="all">Tất cả</option>
        <option value="alert">Chỉ có cảnh báo</option>
        <option value="critical">Chỉ critical</option>
        <option value="warning">Chỉ warning</option>
        <option value="chart">Hiện biểu đồ</option>
    </select>
</div>

<div class="summary">
    <div class="summary-card"><b id="total">0</b><br><span class="muted">Bệnh nhân</span></div>
    <div class="summary-card"><b id="ok">0</b><br><span class="muted">Ổn định</span></div>
    <div class="summary-card"><b id="warning">0</b><br><span class="muted">Warning</span></div>
    <div class="summary-card"><b id="critical">0</b><br><span class="muted">Critical</span></div>
</div>


<div id="chartPanel" class="chart-panel" style="display:none;">
    <div class="topbar" style="justify-content:space-between; align-items:center;">
        <div style="display:flex; gap:12px; align-items:center; flex-wrap:wrap;">
            <b>Biểu đồ trạng thái</b>
            <select id="patientSelect" onchange="selectedKey=this.value; renderCharts();"></select>
            <span class="muted">Lưu lịch sử tạm thời trong trình duyệt, tối đa 120 điểm gần nhất.</span>
        </div>
        <button onclick="hideCharts()" class="close-chart-btn">✕ Tắt biểu đồ</button>
    </div>
    <div class="chart-grid">
        <div class="chart-box">
            <h3>Sinh hiệu: HR / SpO2 / Temp</h3>
            <canvas id="vitalChart"></canvas>
        </div>
        <div class="chart-box">
            <h3>Dịch truyền: Volume / DPM</h3>
            <canvas id="loadcellChart"></canvas>
        </div>
        <div class="chart-box">
            <h3>Trạng thái cảnh báo: OK / Warning / Critical</h3>
            <canvas id="statusChart"></canvas>
        </div>
    </div>
</div>

<div id="grid" class="grid"></div>

<script>
const patients = {};
const history = {}; // key -> [{t, hr, spo2, temp, volume, dpm, alertLevel}]
let lastUpdateText = "-";
let selectedKey = null;
const MAX_HISTORY_POINTS = 120; // 120 giây nếu polling 1s

function showCharts() {
    const chartPanel = document.getElementById("chartPanel");
    if (chartPanel) {
        chartPanel.style.display = "block";
        setTimeout(renderCharts, 50);
        chartPanel.scrollIntoView({behavior: "smooth", block: "start"});
    }
}

function hideCharts() {
    const chartPanel = document.getElementById("chartPanel");
    if (chartPanel) {
        chartPanel.style.display = "none";
    }
    const filterEl = document.getElementById("filterLevel");
    if (filterEl && filterEl.value === "chart") {
        filterEl.value = "all";
    }
}

function levelOf(p) {
    if (!p.alerts || p.alerts.length === 0) return "ok";
    if (p.alerts.some(a => a.level === "critical")) return "critical";
    return "warning";
}

function cardClass(p) {
    const level = levelOf(p);
    if (level === "critical") return "card critical";
    if (level === "warning") return "card warning";
    return "card";
}

function fmtTime(ts) {
    if (!ts) return "-";
    return new Date(ts * 1000).toLocaleTimeString();
}

function updateHistory(p) {
    if (!history[p.key]) history[p.key] = [];

    const v = p.vital;
    const l = p.loadcell;
    const point = {
        t: Date.now(),
        hr: v ? Number(v.hr) : null,
        spo2: v ? Number(v.spo2) : null,
        temp: v ? Number(v.temp) : null,
        volume: l ? Number(l.volume_ml) : null,
        dpm: l ? Number(l.drops_per_min) : null,
        alertLevel: levelOf(p),
    };

    const arr = history[p.key];
    const last = arr[arr.length - 1];

    // Chỉ thêm điểm mới khi seq thay đổi hoặc sau 1 giây
    const changed =
        !last ||
        last.hr !== point.hr ||
        last.spo2 !== point.spo2 ||
        last.temp !== point.temp ||
        last.volume !== point.volume ||
        last.dpm !== point.dpm ||
        last.alertLevel !== point.alertLevel ||
        point.t - last.t >= 1000;

    if (changed) {
        arr.push(point);
        while (arr.length > MAX_HISTORY_POINTS) arr.shift();
    }
}

function alertLevelNumber(level) {
    if (level === "critical") return 2;
    if (level === "warning") return 1;
    return 0;
}

function drawLineChart(canvas, points, fields) {
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width = canvas.clientWidth * window.devicePixelRatio;
    const h = canvas.height = canvas.clientHeight * window.devicePixelRatio;
    const dpr = window.devicePixelRatio;
    ctx.scale(dpr, dpr);

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    ctx.clearRect(0, 0, width, height);

    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, width, height);

    const padL = 42, padR = 12, padT = 18, padB = 26;
    const plotW = width - padL - padR;
    const plotH = height - padT - padB;

    ctx.strokeStyle = "#e5e7eb";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
        const y = padT + (plotH * i / 4);
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(width - padR, y);
        ctx.stroke();
    }

    if (!points || points.length < 2) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "14px Arial";
        ctx.fillText("Chưa đủ dữ liệu để vẽ biểu đồ", padL, height / 2);
        return;
    }

    let values = [];
    fields.forEach(f => {
        points.forEach(p => {
            if (p[f.key] !== null && p[f.key] !== undefined && !Number.isNaN(p[f.key])) {
                // Bỏ qua giá trị 0 cho HR/SpO2/temp nếu đó là chưa đo được, nhưng vẫn giữ volume/dpm
                if ((f.key === "hr" || f.key === "spo2" || f.key === "temp") && p[f.key] === 0) return;
                values.push(p[f.key]);
            }
        });
    });

    if (values.length === 0) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "14px Arial";
        ctx.fillText("Chưa có giá trị hợp lệ", padL, height / 2);
        return;
    }

    let minV = Math.min(...values);
    let maxV = Math.max(...values);
    if (minV === maxV) {
        minV -= 1;
        maxV += 1;
    }
    const margin = (maxV - minV) * 0.15;
    minV -= margin;
    maxV += margin;

    const t0 = points[0].t;
    const t1 = points[points.length - 1].t || (t0 + 1);

    function xOf(t) {
        return padL + ((t - t0) / Math.max(1, t1 - t0)) * plotW;
    }

    function yOf(v) {
        return padT + (1 - ((v - minV) / (maxV - minV))) * plotH;
    }

    ctx.fillStyle = "#6b7280";
    ctx.font = "11px Arial";
    ctx.fillText(maxV.toFixed(0), 4, padT + 4);
    ctx.fillText(minV.toFixed(0), 4, padT + plotH);

    const palette = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c"];
    fields.forEach((f, idx) => {
        ctx.strokeStyle = palette[idx % palette.length];
        ctx.lineWidth = 2;
        ctx.beginPath();

        let started = false;
        points.forEach(p => {
            let v = p[f.key];
            if (v === null || v === undefined || Number.isNaN(v)) return;
            if ((f.key === "hr" || f.key === "spo2" || f.key === "temp") && v === 0) return;

            const x = xOf(p.t);
            const y = yOf(v);

            if (!started) {
                ctx.moveTo(x, y);
                started = true;
            } else {
                ctx.lineTo(x, y);
            }
        });

        if (started) ctx.stroke();

        ctx.fillStyle = palette[idx % palette.length];
        ctx.font = "12px Arial";
        ctx.fillText(f.label, padL + idx * 92, 13);
    });

    ctx.fillStyle = "#6b7280";
    ctx.font = "11px Arial";
    ctx.fillText(new Date(t0).toLocaleTimeString(), padL, height - 6);
    ctx.fillText(new Date(t1).toLocaleTimeString(), width - 82, height - 6);
}

function drawStatusChart(canvas, points) {
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const w = canvas.width = canvas.clientWidth * window.devicePixelRatio;
    const h = canvas.height = canvas.clientHeight * window.devicePixelRatio;
    const dpr = window.devicePixelRatio;
    ctx.scale(dpr, dpr);

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, width, height);

    const padL = 42, padR = 12, padT = 18, padB = 26;
    const plotW = width - padL - padR;
    const plotH = height - padT - padB;

    ctx.strokeStyle = "#e5e7eb";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 2; i++) {
        const y = padT + plotH - (plotH * i / 2);
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(width - padR, y);
        ctx.stroke();
        ctx.fillStyle = "#6b7280";
        ctx.font = "11px Arial";
        const label = i === 0 ? "OK" : (i === 1 ? "WARN" : "CRIT");
        ctx.fillText(label, 4, y + 4);
    }

    if (!points || points.length < 2) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "14px Arial";
        ctx.fillText("Chưa đủ dữ liệu trạng thái", padL, height / 2);
        return;
    }

    const t0 = points[0].t;
    const t1 = points[points.length - 1].t || (t0 + 1);

    function xOf(t) {
        return padL + ((t - t0) / Math.max(1, t1 - t0)) * plotW;
    }

    function yOfLevel(level) {
        const n = alertLevelNumber(level);
        return padT + plotH - (n / 2) * plotH;
    }

    ctx.strokeStyle = "#111827";
    ctx.lineWidth = 2;
    ctx.beginPath();
    points.forEach((p, idx) => {
        const x = xOf(p.t);
        const y = yOfLevel(p.alertLevel);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    });
    ctx.stroke();

    points.forEach(p => {
        const x = xOf(p.t);
        const y = yOfLevel(p.alertLevel);
        ctx.fillStyle = p.alertLevel === "critical" ? "#ef4444" : (p.alertLevel === "warning" ? "#f59e0b" : "#22c55e");
        ctx.beginPath();
        ctx.arc(x, y, 3, 0, Math.PI * 2);
        ctx.fill();
    });

    ctx.fillStyle = "#6b7280";
    ctx.font = "11px Arial";
    ctx.fillText(new Date(t0).toLocaleTimeString(), padL, height - 6);
    ctx.fillText(new Date(t1).toLocaleTimeString(), width - 82, height - 6);
}

function renderCharts() {
    const select = document.getElementById("patientSelect");
    if (!select) return;

    const keys = Object.keys(patients).sort();
    const old = selectedKey;
    select.innerHTML = keys.map(k => `<option value="${k}">${k}</option>`).join("");

    if (old && keys.includes(old)) selectedKey = old;
    if (!selectedKey && keys.length > 0) selectedKey = keys[0];
    if (selectedKey) select.value = selectedKey;

    const points = selectedKey ? (history[selectedKey] || []) : [];
    drawLineChart(document.getElementById("vitalChart"), points, [
        {key: "hr", label: "HR"},
        {key: "spo2", label: "SpO2"},
        {key: "temp", label: "Temp"},
    ]);
    drawLineChart(document.getElementById("loadcellChart"), points, [
        {key: "volume", label: "Volume"},
        {key: "dpm", label: "DPM"},
    ]);
    drawStatusChart(document.getElementById("statusChart"), points);
}

function render() {
    const grid = document.getElementById("grid");
    const q = document.getElementById("search").value.toLowerCase();
    const filterEl = document.getElementById("filterLevel");
    let filter = filterEl.value;

    if (filter === "chart") {
        showCharts();
        filter = "all";
        filterEl.value = "all";
    }

    let arr = Object.values(patients);

    let total = arr.length;
    let ok = arr.filter(p => levelOf(p) === "ok").length;
    let warning = arr.filter(p => levelOf(p) === "warning").length;
    let critical = arr.filter(p => levelOf(p) === "critical").length;

    document.getElementById("total").textContent = total;
    document.getElementById("ok").textContent = ok;
    document.getElementById("warning").textContent = warning;
    document.getElementById("critical").textContent = critical;

    let statusEl = document.getElementById("connStatus");
    if (!statusEl) {
        statusEl = document.createElement("div");
        statusEl.id = "connStatus";
        statusEl.className = "muted";
        document.body.insertBefore(statusEl, document.getElementById("grid"));
    }
    statusEl.textContent = `Chế độ cập nhật: Polling mỗi 1 giây | Cập nhật cuối: ${lastUpdateText}`;

    arr = arr.filter(p => {
        const text = `${p.key} ${p.wing} ${p.room} ${p.bed} ${p.patient}`.toLowerCase();
        if (q && !text.includes(q)) return false;

        const level = levelOf(p);
        if (filter === "alert" && level === "ok") return false;
        if (filter === "critical" && level !== "critical") return false;
        if (filter === "warning" && level !== "warning") return false;

        return true;
    });

    arr.sort((a, b) => a.key.localeCompare(b.key));

    grid.innerHTML = "";
    arr.forEach(p => {
        const v = p.vital;
        const l = p.loadcell;
        const alerts = p.alerts || [];

        const div = document.createElement("div");
        div.className = cardClass(p);
        div.onclick = () => {
            selectedKey = p.key;
            renderCharts();
            window.scrollTo({top: 0, behavior: "smooth"});
        };

        div.innerHTML = `
            <div class="title">${p.key}</div>
            <div class="row"><b>VITAL:</b> ${
                v ? `HR ${v.hr} | SpO2 ${v.spo2}% | T ${Number(v.temp).toFixed(1)}°C | Pin ${v.battery}% | Seq ${v.seq}` : "-"
            }</div>
            <div class="row"><b>LOADCELL:</b> ${
                l ? `${l.volume_ml} ml | ${l.drops_per_min} giọt/phút | Seq ${l.seq}` : "-"
            }</div>
            <div class="row"><b>Cảnh báo:</b> ${
                alerts.length === 0
                    ? `<span class="ok">OK</span>`
                    : alerts.map(a => `<div class="alert">${a.level.toUpperCase()} - ${a.message}: ${a.value}</div>`).join("")
            }</div>
            <div class="muted">Last seen: ${fmtTime(p.last_seen)} | Bấm card để xem biểu đồ</div>
        `;

        grid.appendChild(div);
    });

    renderCharts();
}

async function refreshPatients() {
    try {
        const r = await fetch("/api/patients?ts=" + Date.now(), {cache: "no-store"});
        const data = await r.json();

        Object.keys(patients).forEach(k => delete patients[k]);

        data.forEach(p => {
            patients[p.key] = p;
            updateHistory(p);
        });

        lastUpdateText = new Date().toLocaleTimeString();
        render();
    } catch (e) {
        lastUpdateText = "Lỗi tải /api/patients";
        render();
    }
}

window.addEventListener("resize", renderCharts);

setInterval(refreshPatients, 1000);
refreshPatients();
</script>
</body>
</html>
"""

    @app.get("/")
    async def index():
        return HTMLResponse(html)

    @app.get("/api/patients")
    async def api_patients():
        with patients_lock:
            return JSONResponse([patient_to_dict(p) for p in patients.values()])

    @app.get("/api/thresholds")
    async def api_thresholds():
        return JSONResponse({
            "HR_LOW_CRITICAL": HR_LOW_CRITICAL,
            "HR_LOW_WARNING": HR_LOW_WARNING,
            "HR_HIGH_WARNING": HR_HIGH_WARNING,
            "HR_HIGH_CRITICAL": HR_HIGH_CRITICAL,
            "SPO2_WARNING": SPO2_WARNING,
            "SPO2_CRITICAL": SPO2_CRITICAL,
            "TEMP_WARNING": TEMP_WARNING,
            "TEMP_CRITICAL": TEMP_CRITICAL,
            "BATTERY_WARNING": BATTERY_WARNING,
            "BATTERY_CRITICAL": BATTERY_CRITICAL,
            "INFUSION_EMPTY_WARNING_ML": INFUSION_EMPTY_WARNING_ML,
            "INFUSION_EMPTY_CRITICAL_ML": INFUSION_EMPTY_CRITICAL_ML,
            "DROPS_LOW_WARNING": DROPS_LOW_WARNING,
            "DROPS_HIGH_WARNING": DROPS_HIGH_WARNING,
            "DROPS_HIGH_CRITICAL": DROPS_HIGH_CRITICAL,
            "OFFLINE_WARNING_SEC": OFFLINE_WARNING_SEC,
            "OFFLINE_CRITICAL_SEC": OFFLINE_CRITICAL_SEC,
            "INFUSION_STOP_CONFIRM_SEC": INFUSION_STOP_CONFIRM_SEC,
        })

    global web_loop
    web_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(web_loop)

    config = uvicorn.Config(app, host=host, port=port, log_level="info", loop="asyncio")
    server = uvicorn.Server(config)
    web_loop.run_until_complete(server.serve())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="slcan", help="Windows CANable thường dùng slcan")
    parser.add_argument("--channel", default="COM27", help="Ví dụ COM27")
    parser.add_argument("--bitrate", type=int, default=125000)
    parser.add_argument("--print-raw", action="store_true")
    parser.add_argument("--web", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    t = threading.Thread(
        target=can_reader_thread,
        args=(args.interface, args.channel, args.bitrate, args.print_raw),
        daemon=True,
    )
    t.start()

    if args.web:
        print(f"Web dashboard: http://{args.host}:{args.port}")
        run_web_server(args.host, args.port)
    else:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Exit")


if __name__ == "__main__":
    main()
