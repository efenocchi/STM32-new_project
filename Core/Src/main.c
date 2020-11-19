/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "psw.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "semphr.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_hsensor.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TERMINAL_USE

/* Update SSID and PASSWORD with own Access point settings */

//#define SSID "your SSID"
//#define PASSWORD "your password"


uint8_t RemoteIP[] = {192,168,43,10};
#define RemotePORT	8002

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

#if defined (TERMINAL_USE)
#define TERMOUT(...)  printf(__VA_ARGS__)
#else
#define TERMOUT(...)
#endif


extern I2C_HandleTypeDef hI2cHandler;
VL53L0X_Dev_t Dev =
{
  .I2cHandle = &hI2cHandler,
  .I2cDevAddr = PROXIMITY_I2C_ADDRESS
};


/* Private variables ---------------------------------------------------------*/
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */
static uint8_t RxData [500];


/* Private function prototypes -----------------------------------------------*/
#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small TERMOUT (option LD Linker->Libraries->Small TERMOUT
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);



extern  SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

extern  SPI_HandleTypeDef hspi;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
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
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;


/* Definitions for serialPrintTask*/

osThreadId_t serialTaskHandle;
const osThreadAttr_t serialTask_attributes = {
  .name = "serialTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};


/* Definitions for statisticsTask*/
osThreadId_t statisticsTaskHandle;
const osThreadAttr_t statisticsTask_attributes = {
  .name = "statisticsTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 232 * 4
};

/* Definitions for dewpointTask */
osThreadId_t dewpointTaskHandle;
const osThreadAttr_t dewpointTask_attributes = {
  .name = "dewpointTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 232 * 4
};

/* Definitions for wifiTask */
osThreadId_t wifiTaskHandle;
const osThreadAttr_t wifiTask_attributes = {
  .name = "wifiTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 232 * 4
};
/* Definitions for sensorsTask */
osThreadId_t sensorsTaskHandle;
const osThreadAttr_t sensorsTask_attributes = {
  .name = "sensorsTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 232 * 4
};

/* Definitions for proximityThread */
osThreadId_t proximityThreadHandle;
const osThreadAttr_t proximityThread_attributes = {
  .name = "proximityThread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 440 * 4
};
/* USER CODE BEGIN PV */

struct sharedValues_t{
int temperature_val1;
int temperature_val2;
float array_temp[5];
float average_temerature;
int n_elements_temp;
int check_mean_temp;
int updated_first;
int updated_second;
int humidity_val1;
int humidity_val2;
float array_humidity[5];
float average_humidity;
int n_elements_humidity;
int check_mean_humidity;

int pressure_val1;
int pressure_val2;
float array_pressure[5];
float average_pressure;
int n_elements_pressure;
int check_mean_pressure;
int WiFi_blocked;

int proximity;
int dewpoint;
int enableDew;




osSemaphoreId_t mutex;
osSemaphoreId_t primo;
osSemaphoreId_t secondo;


}sharedValues;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

static void VL53L0X_PROXIMITY_MspInit(void);
static uint16_t VL53L0X_PROXIMITY_GetDistance(void);
static void VL53L0X_PROXIMITY_Init(void);

void WiFiConnection(void *arguments);
void PeriodicPrint(void *arguments);
void Proximity_Test(void *arguments);
void StartDewpointTask(void *arguments);
void ComputeStatistics(void *arguments);
void SerialPrint(void *arguments);


/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void inizialize(struct sharedValues_t *sv){

	sv->updated_first=0;
	sv->updated_second=0;
	sv->temperature_val1=0;
	sv->temperature_val2=0;
	sv->average_temerature=0;
	sv->n_elements_temp=0;


	sv->humidity_val1=0;
	sv->humidity_val2=0;
	sv->average_humidity=0;
	sv->n_elements_humidity=0;

	sv->pressure_val1=0;
	sv->pressure_val2=0;
	sv->average_pressure=0;
	sv->n_elements_pressure=0;

	sv->proximity=0;
	sv->dewpoint=0;
	sv->enableDew=0;

	sv->check_mean_temp=0;
	sv->check_mean_humidity=0;
	sv->check_mean_pressure=0;

	sv->WiFi_blocked=0;

	sv->mutex = osSemaphoreNew(1, 1, NULL);
	sv->primo = osSemaphoreNew(1, 1, NULL);
	sv->secondo = osSemaphoreNew(1, 1, NULL);



}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


int main(void)
{
  /* USER CODE BEGIN 1 */

	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	   HAL_Init();

	   /* Configure the system clock */
	   SystemClock_Config();
	   /* Configure LED2 */
	   BSP_LED_Init(LED2);

      /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

#if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  BSP_COM_Init(COM1, &hDiscoUart);
#endif /* TERMINAL_USE */

		MX_GPIO_Init();
		MX_DFSDM1_Init();
		MX_I2C2_Init();
		MX_QUADSPI_Init();
		MX_SPI3_Init();
		MX_USART1_UART_Init();
		MX_USART3_UART_Init();
		MX_USB_OTG_FS_PCD_Init();
		BSP_TSENSOR_Init();
		BSP_PSENSOR_Init();
		BSP_HSENSOR_Init();
		VL53L0X_PROXIMITY_Init();


	  TERMOUT("****** WIFI Module in TCP Client mode demonstration ****** \n\n");
	  TERMOUT("TCP Client Instructions :\n");
	  TERMOUT("1- Make sure your Phone is connected to the same network that\n");
	  TERMOUT("   you configured using the Configuration Access Point.\n");
	  TERMOUT("2- Create a server by using the android application TCP Server\n");
	  TERMOUT("   with port(8002).\n");
	  TERMOUT("3- Get the Network Name or IP Address of your Android from the step 2.\n\n");


		    /* Init scheduler ----------------------------------------------------------------------------*/
	  osKernelInitialize();


	  /* USER CODE BEGIN RTOS_THREADS */
	  //inizialize the structure
	  inizialize(&sharedValues);

	  wifiTaskHandle = osThreadNew(WiFiConnection, NULL, &wifiTask_attributes);
	  sensorsTaskHandle = osThreadNew(PeriodicPrint, NULL, &sensorsTask_attributes);
	  proximityThreadHandle = osThreadNew(Proximity_Test, NULL, &proximityThread_attributes);
	  dewpointTaskHandle = osThreadNew(StartDewpointTask, NULL, &dewpointTask_attributes);
	  statisticsTaskHandle = osThreadNew(ComputeStatistics, NULL, &statisticsTask_attributes);
	  serialTaskHandle = osThreadNew(SerialPrint, NULL, &serialTask_attributes);

	  /* USER CODE END RTOS_THREADS */

	  /* Start scheduler */
	HAL_GPIO_WritePin(ARD_D8_GPIO_Port, ARD_D8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_SET);//0
	  osKernelStart();

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}


#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library TERMOUT function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */


/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */



static void VL53L0X_PROXIMITY_Init(void)
{
  uint16_t vl53l0x_id = 0;
  VL53L0X_DeviceInfo_t VL53L0X_DeviceInfo;

  /* Initialize IO interface */
  SENSOR_IO_Init();
  VL53L0X_PROXIMITY_MspInit();

  memset(&VL53L0X_DeviceInfo, 0, sizeof(VL53L0X_DeviceInfo_t));

  if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&Dev, &VL53L0X_DeviceInfo))
  {
    if (VL53L0X_ERROR_NONE == VL53L0X_RdWord(&Dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, (uint16_t *) &vl53l0x_id))
    {
      if (vl53l0x_id == VL53L0X_ID)
      {
        if (VL53L0X_ERROR_NONE == VL53L0X_DataInit(&Dev))
        {
          Dev.Present = 1;
          SetupSingleShot(Dev);
        }
        else
        {
          printf("VL53L0X Time of Flight Failed to send its ID!\n");
        }
      }
    }
    else
    {
      printf("VL53L0X Time of Flight Failed to Initialize!\n");
    }
  }
  else
  {
    printf("VL53L0X Time of Flight Failed to get infos!\n");
  }
}

/**
  * @brief  Get distance from VL53L0X proximity sensor.
  * @retval Distance in mm
  */
static uint16_t VL53L0X_PROXIMITY_GetDistance(void)
{
  VL53L0X_RangingMeasurementData_t RangingMeasurementData;

  VL53L0X_PerformSingleRangingMeasurement(&Dev, &RangingMeasurementData);

  return RangingMeasurementData.RangeMilliMeter;
}

static void VL53L0X_PROXIMITY_MspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin : VL53L0X_XSHUT_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VL53L0X_XSHUT_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);
}


/**
  * @brief  Function implementing the wifiTask thread.
  * @param  argument: Not used
  * @retval None
  */

int controlla_valori_telefono(struct sharedValues_t *sv, int32_t Socket){
	uint8_t TxData[] = "STM32 : Hello device!\n";
	int32_t ret;
	uint16_t Datalen;
	char text[30];


	while(1){

		//bloccante
		  ret = WIFI_ReceiveData(Socket, RxData, sizeof(RxData)-1, &Datalen, WIFI_READ_TIMEOUT);


		  if(ret == WIFI_STATUS_OK){

			if(Datalen > 0){
				osSemaphoreAcquire(sv->mutex, portMAX_DELAY);
				  RxData[Datalen]=0;

				  int ritorno = atoi(RxData);

				  if(ritorno==45){
					  TERMOUT("Il telefono è connesso al Wifi (tasto premuto)\n");
					  ret = WIFI_SendData(Socket, TxData, sizeof(TxData), &Datalen, WIFI_WRITE_TIMEOUT);
				  }

				  //PROXIMITY
				  if(ritorno == 0){
					  TERMOUT("Richiesta valore prossimità\n");
					  snprintf(text, 30, "Proximity value: %d mm\n", sv->proximity); // puts string into buffer
					  //TERMOUT("---text :%s\n",text);
					  ret = WIFI_SendData(Socket, text, sizeof(text), &Datalen, WIFI_WRITE_TIMEOUT);

				  }

				  //TEMPERATURE
				  if(ritorno==1){
					  TERMOUT("Richiesta valore temperature\n");
					  snprintf(text,30," Temperature = %d.%02d °C\n\r", sv->temperature_val1, sv->temperature_val2);
					  ret = WIFI_SendData(Socket, text, sizeof(text), &Datalen, WIFI_WRITE_TIMEOUT);
				  }

				  //HUMIDITY
				  if(ritorno==2){
					  TERMOUT("Richiesta valore umidità\n");
					  snprintf(text,30," Humidity = %d.%02d %%\n\r", sv->humidity_val1, sv->humidity_val2);
					  ret = WIFI_SendData(Socket, text, sizeof(text), &Datalen, WIFI_WRITE_TIMEOUT);
				  }

				  //PRESSSURE 1mBar = 1hPa (100Pa)
				  if(ritorno==3){
					  TERMOUT("Richiesta valore pressione\n");
					  snprintf(text,30," Pressure = %d.%02d hPa\n\r", sv->pressure_val1, sv->pressure_val2);
					  ret = WIFI_SendData(Socket, text, sizeof(text), &Datalen, WIFI_WRITE_TIMEOUT);
				  }

				  if(ritorno==4){
					  snprintf(text,30," Dewpoint = %d\n\r", sv->dewpoint);
					  ret = WIFI_SendData(Socket, text, sizeof(text), &Datalen, WIFI_WRITE_TIMEOUT);


				  }
				  sv->WiFi_blocked=0;
				  osSemaphoreRelease(sv->mutex);
				  //if we had any problems the mutex won't be used
				  if (ret != WIFI_STATUS_OK){
					TERMOUT("> ERROR : Failed to Send Data, connection closed\n");
					 HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_SET);//0

					break;
				  }
				}
			else{
				//Wifi not used for a long time
				osSemaphoreAcquire(sv->mutex, portMAX_DELAY);
				sv->WiFi_blocked++;
				TERMOUT("Timeout n°%d\n",sv->WiFi_blocked);

				if(sv->WiFi_blocked==3){
					HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_SET);
					TERMOUT("> ERROR : no data received for a long time\n");
					sv->WiFi_blocked=0;
					osSemaphoreRelease(sv->mutex);
					break;
				}
				osSemaphoreRelease(sv->mutex);
			}

		  }
		  else
		  {
			TERMOUT("> ERROR : Failed to Receive Data, connection closed\n");
			 HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_SET);//0
			 break;
		  }

		  HAL_Delay(500);
	}

		  return 1;
}





/* USER CODE END Header_WiFiConnection */

//Cerca di instaurare una connessione WiFi
void WiFiConnection(void *arguments)
{

	while(1){
			uint8_t  MAC_Addr[6];
			uint8_t  IP_Addr[4];

			int32_t Socket = -1;

			int16_t Trials = CONNECTION_TRIAL_MAX;


			/*Initialize  WIFI module */
			  if(WIFI_Init() ==  WIFI_STATUS_OK)
			  {
					TERMOUT("> WIFI Module Initialized.\n");
					if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
					{
					  TERMOUT("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n",
							   MAC_Addr[0],
							   MAC_Addr[1],
							   MAC_Addr[2],
							   MAC_Addr[3],
							   MAC_Addr[4],
							   MAC_Addr[5]);
					}
					else
					{
					  TERMOUT("> ERROR : CANNOT get MAC address\n");
					  BSP_LED_On(LED2);
					}

					if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
					{
					  TERMOUT("> es-wifi module connected \n");
					  if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
					  {
						TERMOUT("> es-wifi module got IP Address : %d.%d.%d.%d\n",
							   IP_Addr[0],
							   IP_Addr[1],
							   IP_Addr[2],
							   IP_Addr[3]);

						TERMOUT("> Trying to connect to Server: %d.%d.%d.%d:%d ...\n",
							   RemoteIP[0],
							   RemoteIP[1],
							   RemoteIP[2],
							   RemoteIP[3],
											 RemotePORT);

						while (Trials--)
						{
						  if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
						  {
							TERMOUT("> TCP Connection opened successfully.\n");
							Socket = 0;
							 HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_RESET);
							break;
						  }

						}
						if(Socket == -1)
						{
						  TERMOUT("> ERROR : Cannot open Connection\n");
						  BSP_LED_On(LED2);
						}
					  }
					  else
					  {
						TERMOUT("> ERROR : es-wifi module CANNOT get IP address\n");
						BSP_LED_On(LED2);
					  }
					}
					else
					{
					  TERMOUT("> ERROR : es-wifi module NOT connected\n");
					  BSP_LED_On(LED2);
					}
			  }
			  else
			  {
				TERMOUT("> ERROR : WIFI Module cannot be initialized.\n");
				BSP_LED_On(LED2);
			  }

		//connessione avvenuta
		if(Socket!=-1){
			int error_connession=0;
			for(;;){

				error_connession = controlla_valori_telefono(&sharedValues,Socket);

				//if we have an error after reaching the connection, we try to connect the board again
				//it will always be 1
				if(error_connession==1){
					//wait 3 seconds and try to connect to the device again
					HAL_Delay(3000);
					break;
				}

			}
		}

	//wait 4 seconds and try to connect to the device again
	HAL_Delay(4000);
	}
}

/* USER CODE BEGIN Header_PeriodicPrint */
/**
* @brief Function implementing the Print thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PeriodicPrint */

//Stampa su seriale i valori acquisiti dai sensori e le statistiche effettuate
void print_Values_UART(struct sharedValues_t *sv){
	 osSemaphoreAcquire(sv->mutex, portMAX_DELAY);

		float temperature,humidity,pressure;
		float separa = 0;
		int val1,val2;


		temperature = BSP_TSENSOR_ReadTemp();
		humidity = BSP_HSENSOR_ReadHumidity();
		pressure = BSP_PSENSOR_ReadPressure();

		if(temperature>27){
			HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(ARD_D5_GPIO_Port, ARD_D5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_RESET);
		}
//temperature
		val1 = temperature;
		separa = temperature - val1;
		val2 = trunc(separa * 100);
		sv->temperature_val1 = val1;
		sv->temperature_val2 = val2;
		if(sv->n_elements_temp<5){
			sv->array_temp[sv->n_elements_temp]=temperature;
			sv->n_elements_temp += 1;

		}

//humidity
		val1 = humidity;
		separa = humidity - val1;
		val2 = trunc(separa * 100);
		sv->humidity_val1 = val1;
		sv->humidity_val2 = val2;
		if(sv->n_elements_humidity<5){
			sv->array_humidity[sv->n_elements_humidity]=humidity;
			sv->n_elements_humidity +=1;
		}

//pressure
		val1 = pressure;
		separa = pressure - val1;
		val2 = trunc(separa * 100);
		sv->pressure_val1 = val1;
		sv->pressure_val2 = val2;
		if(sv->n_elements_pressure<5){
			sv->array_pressure[sv->n_elements_pressure]=pressure;
			sv->n_elements_pressure +=1;
		}

		if(sv->updated_first==0)
			sv->updated_first=1;
	  osSemaphoreRelease(sv->mutex);

}

void PeriodicPrint(void *arguments)
{
  /* USER CODE BEGIN PeriodicPrint */
  /* Infinite loop */
  for(;;)
  {
	  print_Values_UART(&sharedValues);
	  osDelay(500);
  }
  /* USER CODE END PeriodicPrint */
}

void update_Proximity(struct sharedValues_t *sv){
	uint16_t prox_value;

	//prendo il mutex
	osSemaphoreAcquire(sv->mutex, portMAX_DELAY);

	prox_value = VL53L0X_PROXIMITY_GetDistance();
	sv->proximity = prox_value;

	if(prox_value<100)
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);

	if(sv->updated_second==0)
		sv->updated_second=1;

	osSemaphoreRelease(sv->mutex);

}

void Proximity_Test(void *arguments)
{

  printf("\n*************************************************************\n");
  printf("\n********************** Proximity Test ************************\n");
  printf("\n*************************************************************\n\n");


  while(1)
  {
	  	  update_Proximity(&sharedValues);
	  	  HAL_Delay(1000);
  }

}

//controlla se il tasto è stato acquisito e nel caso positivo stampa su seriale il valore di rugiada calcolato
void printDewpoint(struct sharedValues_t *sv){
	int dewpoint=-1;
	char msg_d[30] = "";
	float hum_f,temp_f;

	osSemaphoreAcquire(sv->mutex, portMAX_DELAY);

	hum_f = sv->humidity_val1;
	temp_f = sv->temperature_val1;
	dewpoint = (pow(hum_f/100, 0.125)*(112+temp_f*0.9)+(0.1*temp_f)-112);
	sv->dewpoint=dewpoint;

	if(sv->enableDew==1){

		snprintf(msg_d,30," DEWPOINT = %d °C\n\r", dewpoint);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg_d, sizeof(msg_d), 1000);
		sv->enableDew=0;
	}
	osSemaphoreRelease(sv->mutex);


}

void StartDewpointTask(void *arguments){
	/*USER CODE BEGIN StartDewpointTask*/
	/* Infinite loop */
	for(;;)
	{
		printDewpoint(&sharedValues);
		 osDelay(1000);
	}
	/* USER CODE END StartDewpointTask */
}

//Faccio statistiche sui dati campionati dagli altri thread
void computeAndUpdate(struct sharedValues_t *sv){
	int i;

	osSemaphoreAcquire(sv->mutex, portMAX_DELAY);
	if(sv->n_elements_temp==5){
		sv->average_temerature=0;
		for(i=0;i<5;i++){
			sv->average_temerature = sv->average_temerature + sv->array_temp[i];
		}

		sv->average_temerature = sv->average_temerature / 5;
		sv->check_mean_temp=1;
	}


	if(sv->n_elements_pressure==5){
		sv->average_pressure=0;
		for(i=0;i<5;i++){
			sv->average_pressure = sv->average_pressure + sv->array_pressure[i];
		}
		sv->average_pressure = sv->average_pressure / 5;
		sv->check_mean_pressure=1;
	}


	if(sv->n_elements_humidity==5){
		sv->average_humidity=0;
		for(i=0;i<5;i++){
			sv->average_humidity = sv->average_humidity + sv->array_humidity[i];
		}
		sv->average_humidity = sv->average_humidity / 5;
		sv->check_mean_humidity=1;
	}

	osSemaphoreRelease(sv->mutex);

}

void ComputeStatistics(void *arguments){

	for(;;){
		computeAndUpdate(&sharedValues);

		osDelay(1000);
	}
}

//stampa valori sulla seriale (quando disponibili stampa anche i valori statistici)
void checkAndPrint(struct sharedValues_t *sv){


	osSemaphoreAcquire(sv->mutex, portMAX_DELAY);

	char msgm1[100] = "";
	char msgm2[100] = "";
	char msgm3[100] = "";
	char msg1[100] = "";
	char msg2[100] = "";
	char msg3[100] = "";
	char msg4[100] = "";


	float separa;
	int val1,val2;

	if(sv->updated_first==1 && sv->updated_second==1){
		snprintf(msg1,100,"Temperature = %d.%02d °C\n\r", sv->temperature_val1, sv->temperature_val2);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg1, sizeof(msg1), 1000);


		snprintf(msg2,100,"Relative Humidity = %d.%02d %%\n\r", sv->humidity_val1, sv->humidity_val2);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg2, sizeof(msg2), 1000);


		snprintf(msg3,100,"Pressure = %d.%02d hPa\n\r", sv->pressure_val1, sv->pressure_val2);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg3, sizeof(msg3), 1000);


		snprintf(msg4,100,"Distance = %d mm\n\r", sv->proximity);
		HAL_UART_Transmit(&huart1, (uint8_t*) msg4, sizeof(msg4), 1000);


		//statistical values
		if(sv->n_elements_temp==5 && sv->check_mean_temp==1 ){
			val1 = sv->average_temerature;
			separa = sv->average_temerature - val1;
			val2 = trunc(separa * 100);
			snprintf(msgm1,100,"Average Temperature = %d.%02d °C\n\r", val1, val2);
			HAL_UART_Transmit(&huart1, (uint8_t*) msgm1, sizeof(msgm1), 1000);

			sv->n_elements_temp=0;
			sv->check_mean_temp=0;
		}

		if(sv->n_elements_humidity==5 && sv->check_mean_humidity==1){
			val1 = sv->average_humidity;
			separa = sv->average_humidity - val1;
			val2 = trunc(separa * 100);
			snprintf(msgm2,100,"Average Humidity = %d.%02d %%\n\r", val1, val2);
			HAL_UART_Transmit(&huart1, (uint8_t*) msgm2, sizeof(msgm2), 1000);

			sv->n_elements_humidity=0;
			sv->check_mean_humidity=0;
		}

		if(sv->n_elements_pressure==5 && sv->check_mean_pressure==1){
			val1 = sv->average_pressure;
			separa = sv->average_pressure - val1;
			val2 = trunc(separa * 100);
			snprintf(msgm3,100,"Average Pressure = %d.%02d hPa\n\r", val1, val2);
			HAL_UART_Transmit(&huart1, (uint8_t*) msgm3, sizeof(msgm3), 1000);

			sv->n_elements_pressure=0;
			sv->check_mean_pressure=0;
		}
	}
	osSemaphoreRelease(sv->mutex);
}


void SerialPrint(void *arguments){


	for(;;){

		checkAndPrint(&sharedValues);
		osDelay(2000);
	}
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    case (BUTTON_EXTI13_Pin):
    {
    	if (HAL_GPIO_ReadPin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin) != GPIO_PIN_SET){
    		sharedValues.enableDew=1;
    	}
    }

    default:
    {
      break;
    }
  }
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
