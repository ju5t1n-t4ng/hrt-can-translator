/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN PV */
// CAN26 Variables
CANHandle_t CANHandle = &hfdcan2;
CANDevice_t CANDevice = {
		.motorDomain = 1,
		.deviceUUID = CAN_UUID_BLDC_ELBOW
};
CANPacket_t RxPacket;

// CubeMars Variables
CubeMarsConfig cubeMarsConfig = {
		.controller_id = 1,
		.pole_pairs = AK60_39_POLE_PAIRS,
		.gear_ratio = AK60_39_GEAR_RATIO,
		.input_is_joint_speed = false,
		.max_abs_erpm = 50000
};
CubeMarsFrame motorRxFrame;

volatile bool estopActive = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
  CANInit(CANHandle, &CANDevice);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (CANPollAndReceive(CANHandle, &RxPacket)) {
		  RxPacket.device.deviceUUID &= 0x3F;
		  int ack = RxPacket.command & 0x80;

		  switch (RxPacket.command & 0x7F) {
		  case CAN_COMMAND_ID__VERSION_GET: {
			  if (ack) {
				  CANUniversalPacket_GetFirmwareVersion_Decoded_t decoded =
						  CANUniversalPacket_GetFirmwareVersion_Decode(&RxPacket);
				  CANPacket_t response = CANUniversalPacket_FirmwareVersion(
						  decoded.receiver, decoded.sender, "elbw", 1);
				  CANSend(CANHandle, &response);
			  }
			  break;
		  }
		  case CAN_COMMAND_ID__E_STOP: {
			  CANUniversalPacket_EStop_Decoded_t decoded =
					  CANUniversalPacket_EStop_Decode(&RxPacket);
			  (void)decoded;
			  estopActive = true;
			  if (ack) {
				  CANPacket_t response = CANUniversalPacket_Acknowledge(
						  decoded.receiver, decoded.sender, false, (CANCommand_t)(RxPacket.command & 0x7F));
				  CANSend(CANHandle, &response);
			  }
			  break;
		  }
		  case CAN_COMMAND_ID__BLDC_INPUT_VELOCITY: {
			  if (estopActive) {
				  if (ack) {
					  CANPacket_t response = CANUniversalPacket_Acknowledge(
							  RxPacket.device,
							  (CANDevice_t){ .deviceUUID = RxPacket.senderUUID },
							  true,
							  (CANCommand_t)(RxPacket.command & 0x7F));
					  CANSend(CANHandle, &response);
				  }
				  break;
			  }

			  // motorFrame using translation
			  CubeMarsFrame motorFrame;
			  bool ok = CAN26ToCubeMarsServoVelocity(&RxPacket, &cubeMarsConfig, &motorFrame);

			  if (ok) {
				  FDCAN_TxHeaderTypeDef txHeader = {0};
				  txHeader.Identifier = motorFrame.id; // 29-bit CAN ID
				  txHeader.IdType = motorFrame.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;

				  txHeader.TxFrameType = FDCAN_DATA_FRAME;
				  txHeader.DataLength = FDCAN_DLC_BYTES_4; // Servo SET_RPM is 4 bytes of big-endian speed
				  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
				  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
				  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				  txHeader.MessageMarker = 0;

				  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, motorFrame.data) != HAL_OK) {
					  if (ack) {
						  CANPacket_t response = CANUniversalPacket_Acknowledge(
								  RxPacket.device,
								  (CANDevice_t){ .deviceUUID = RxPacket.senderUUID},
								  true,
								  (CANCommand_t)(RxPacket.command & 0x7f));
						  CANSend(CANHandle, &response);
					  }
					  break;
				  }


			  } if (ack) {
				  CANPacket_t response = CANUniversalPacket_Acknowledge(
						  RxPacket.device,
						  (CANDevice_t){ .deviceUUID = RxPacket.senderUUID },
						  false,
						  (CANCommand_t)(RxPacket.command & 0x7F));
				  CANSend(CANHandle, &response);
			  }
			  break;
		  }

		  default:
			  if (ack) {
				  CANPacket_t response = CANUniversalPacket_Acknowledge(
						  RxPacket.device,
						  (CANDevice_t){ .deviceUUID = RxPacket.senderUUID },
						  true,
						  (CANCommand_t)(RxPacket.command & 0x7F));
				  CANSend(CANHandle, &response);
			  }
			  break;
		  }
	  }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 5;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 2;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 27;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 8;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
