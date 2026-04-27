/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CAN Receiver + UART2 Logger
  *
  * Chức năng:
  * - Nhận frame từ CAN1
  * - Giải mã ID theo room/bed
  * - Giải mã payload 4 byte:
  *     byte0 = HR
  *     byte1 = SpO2
  *     byte2 = temp_x10 low
  *     byte3 = temp_x10 high
  * - In kết quả ra UART2 ở baudrate 115200
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Struct lưu dữ liệu sau khi giải mã CAN */
typedef struct
{
  uint16_t can_id;
  uint8_t room;
  uint8_t bed;
  uint8_t hr;
  uint8_t spo2;
  int16_t temp_x10;
} PatientRxData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Độ dài buffer dùng để log UART2 */
#define UART_TX_BUF_SIZE 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Buffer log UART2 */
static char uart_tx_buf[UART_TX_BUF_SIZE];

/* Biến lưu dữ liệu nhận gần nhất */
static PatientRxData_t g_rx_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void UART2_Print(const char *msg);
static uint8_t CAN1_Filter_AllPass_Config(void);
static void Decode_CAN_To_Patient(uint16_t can_id, uint8_t data[8], PatientRxData_t *out);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Gửi chuỗi log ra UART2
  */
static void UART2_Print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/**
  * @brief  Cấu hình CAN filter nhận tất cả frame
  * @note   Dùng để test cho dễ. Sau này có thể lọc theo ID nếu muốn.
  */
static uint8_t CAN1_Filter_AllPass_Config(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  /* Nhận tất cả */
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;

  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    return 0;
  }

  return 1;
}

/**
  * @brief  Giải mã frame CAN sang dữ liệu bệnh nhân
  * @param  can_id: Standard ID 11-bit
  * @param  data  : mảng data nhận từ CAN
  * @param  out   : struct output
  *
  * Quy ước:
  * ID = 0x100 | (room << 3) | bed
  * room = 5 bit
  * bed  = 3 bit
  *
  * payload 4 byte:
  * data[0] = hr
  * data[1] = spo2
  * data[2] = temp_x10 low
  * data[3] = temp_x10 high
  */
static void Decode_CAN_To_Patient(uint16_t can_id, uint8_t data[8], PatientRxData_t *out)
{
  out->can_id = can_id;
  out->room = (uint8_t)((can_id >> 3) & 0x1F);
  out->bed  = (uint8_t)(can_id & 0x07);

  out->hr   = data[0];
  out->spo2 = data[1];
  out->temp_x10 = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
}

/**
  * @brief  Callback khi CAN FIFO0 có message mới
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8] = {0};

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
  {
    UART2_Print("CAN RX ERROR\r\n");
    return;
  }

  /* Chỉ xử lý Standard Data Frame có DLC = 4 */
  if ((rxHeader.IDE == CAN_ID_STD) &&
      (rxHeader.RTR == CAN_RTR_DATA) &&
      (rxHeader.DLC >= 4))
  {
    Decode_CAN_To_Patient(rxHeader.StdId, rxData, &g_rx_data);

    snprintf(uart_tx_buf, sizeof(uart_tx_buf),
             "RX CAN | ID=0x%03X Room=%u Bed=%u HR=%u SpO2=%u Temp=%.1f\r\n",
             g_rx_data.can_id,
             g_rx_data.room,
             g_rx_data.bed,
             g_rx_data.hr,
             g_rx_data.spo2,
             g_rx_data.temp_x10 / 10.0f);

    UART2_Print(uart_tx_buf);
  }
  else
  {
    snprintf(uart_tx_buf, sizeof(uart_tx_buf),
             "RX CAN | Unsupported frame IDE=%lu RTR=%lu DLC=%lu\r\n",
             (unsigned long)rxHeader.IDE,
             (unsigned long)rxHeader.RTR,
             (unsigned long)rxHeader.DLC);

    UART2_Print(uart_tx_buf);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  UART2_Print("F407 CAN Receiver Start\r\n");
  UART2_Print("UART2 = 115200\r\n");
  UART2_Print("CAN1 = 125 kbps\r\n");

  /* Cấu hình filter nhận tất cả frame */
  if (!CAN1_Filter_AllPass_Config())
  {
    UART2_Print("CAN Filter Config Failed\r\n");
    Error_Handler();
  }

  /* Start CAN */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    UART2_Print("CAN Start Failed\r\n");
    Error_Handler();
  }

  /* Bật ngắt khi có message mới ở FIFO0 */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    UART2_Print("CAN Notification Failed\r\n");
    Error_Handler();
  }

  UART2_Print("CAN Receiver Ready\r\n");

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    /* Không cần làm gì ở vòng lặp.
       Dữ liệu CAN được xử lý trong callback interrupt. */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   /* 42 MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   /* 84 MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;

  /* Bên nhận vẫn nên bật để bus ổn định hơn */
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;

  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
/* Không cần thêm gì ở đây */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
