#!/usr/bin/env python3
"""
USB-CAN reader for STM32 patient CAN Extended ID.

For your current CANable SLCAN on Windows:
  python usb_can_patient_reader_125k.py --channel COM27

CAN bitrate:
  125000

CAN Extended ID:
  bit 28..26 : msg_type   1=VITAL, 2=LOADCELL
  bit 25..24 : wing       0=A, 1=B
  bit 23..16 : room
  bit 15..8  : bed
  bit 7..0   : patient
"""

import argparse
import time
import can

MSG_TYPE_VITAL = 1
MSG_TYPE_LOADCELL = 2

WING_MAP = {
    0: "A",
    1: "B",
}

patients = {}

def parse_can_id(can_id: int):
    msg_type = (can_id >> 26) & 0x07
    wing_code = (can_id >> 24) & 0x03
    room = (can_id >> 16) & 0xFF
    bed = (can_id >> 8) & 0xFF
    patient = can_id & 0xFF
    wing = WING_MAP.get(wing_code, f"X{wing_code}")
    return msg_type, wing, room, bed, patient

def parse_vital(data: bytes):
    hr = data[0] | (data[1] << 8)
    spo2 = data[2]
    temp_x10 = data[3] | (data[4] << 8)
    if temp_x10 >= 32768:
        temp_x10 -= 65536
    battery = data[5]
    seq = data[6]
    status = data[7]
    return {
        "hr": hr,
        "spo2": spo2,
        "temp": temp_x10 / 10.0,
        "battery": battery,
        "seq": seq,
        "status": status,
    }

def parse_loadcell(data: bytes):
    volume_ml = data[0] | (data[1] << 8)
    drops_per_min = data[2] | (data[3] << 8)
    seq = data[4]
    status = data[5]
    return {
        "volume_ml": volume_ml,
        "drops_per_min": drops_per_min,
        "seq": seq,
        "status": status,
    }

def key_of(wing, room, bed, patient):
    return f"{wing}-R{room:03d}-B{bed:03d}-P{patient:03d}"

def get_patient(wing, room, bed, patient):
    key = key_of(wing, room, bed, patient)
    if key not in patients:
        patients[key] = {
            "wing": wing,
            "room": room,
            "bed": bed,
            "patient": patient,
            "vital": None,
            "loadcell": None,
            "last_seen": time.time(),
            "zero_dpm_since": None,
            "alerts": [],
        }
    return key, patients[key]

def check_alerts(p):
    now = time.time()
    alerts = []

    if now - p["last_seen"] > 8:
        alerts.append("Mất kết nối node")
        return alerts

    v = p.get("vital")
    if v:
        if v["hr"] > 130:
            alerts.append("Nhịp tim cao")
        if 0 < v["hr"] < 45:
            alerts.append("Nhịp tim thấp")
        if 0 < v["spo2"] < 90:
            alerts.append("SpO2 rất thấp")
        elif 0 < v["spo2"] < 94:
            alerts.append("SpO2 thấp")
        if v["temp"] >= 39.0:
            alerts.append("Sốt cao")
        elif v["temp"] >= 38.0:
            alerts.append("Sốt")
        if 0 < v["battery"] < 20:
            alerts.append("Pin yếu")

    l = p.get("loadcell")
    if l:
        if l["volume_ml"] <= 50:
            alerts.append("Dịch gần hết")
        if l["drops_per_min"] > 120:
            alerts.append("Dịch chảy quá nhanh")
        if l["volume_ml"] > 50 and l["drops_per_min"] == 0:
            if p["zero_dpm_since"] is not None and now - p["zero_dpm_since"] >= 20:
                alerts.append("Dịch có thể ngưng chảy")

    return alerts

def print_patient(key, p):
    v = p["vital"]
    l = p["loadcell"]

    vital_text = "-"
    if v:
        vital_text = f"HR={v['hr']} SpO2={v['spo2']} Temp={v['temp']:.1f}C Bat={v['battery']}% Seq={v['seq']}"

    loadcell_text = "-"
    if l:
        loadcell_text = f"ML={l['volume_ml']} DPM={l['drops_per_min']} Seq={l['seq']}"

    alert_text = "OK" if not p["alerts"] else " | ".join(p["alerts"])
    print(f"{key} | VITAL: {vital_text} | LOADCELL: {loadcell_text} | ALERT: {alert_text}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="slcan")
    parser.add_argument("--channel", default="COM27")
    parser.add_argument("--bitrate", type=int, default=125000)
    parser.add_argument("--print-raw", action="store_true")
    args = parser.parse_args()

    bus = can.interface.Bus(
        interface=args.interface,
        channel=args.channel,
        bitrate=args.bitrate,
    )

    print(f"Listening CAN: interface={args.interface}, channel={args.channel}, bitrate={args.bitrate}")

    while True:
        msg = bus.recv(timeout=1.0)

        if msg is None:
            now = time.time()
            for key, p in patients.items():
                old = list(p["alerts"])
                p["alerts"] = check_alerts(p)
                if old != p["alerts"]:
                    print_patient(key, p)
            continue

        if args.print_raw:
            print(
                f"RAW ext={msg.is_extended_id} id=0x{msg.arbitration_id:X} "
                f"dlc={msg.dlc} data={bytes(msg.data).hex(' ')}"
            )

        if not msg.is_extended_id:
            continue

        if len(msg.data) < 8:
            continue

        msg_type, wing, room, bed, patient_id = parse_can_id(msg.arbitration_id)
        key, p = get_patient(wing, room, bed, patient_id)
        p["last_seen"] = time.time()

        if msg_type == MSG_TYPE_VITAL:
            p["vital"] = parse_vital(bytes(msg.data))

        elif msg_type == MSG_TYPE_LOADCELL:
            l = parse_loadcell(bytes(msg.data))
            p["loadcell"] = l

            if l["drops_per_min"] == 0:
                if p["zero_dpm_since"] is None:
                    p["zero_dpm_since"] = time.time()
            else:
                p["zero_dpm_since"] = None

        else:
            continue

        p["alerts"] = check_alerts(p)
        print_patient(key, p)

if __name__ == "__main__":
    main()
