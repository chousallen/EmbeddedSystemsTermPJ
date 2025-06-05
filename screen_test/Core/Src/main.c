/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stlogo.h"
#include <stdio.h>
#include "fir_design.h"
#include "math.h"
#include "string.h"
#include "draw_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* in draw_utils.h
typedef enum
{
    OSC,
    SIG,
    FIL,
	PUR,
} mode_t;

typedef enum
{
	SIN,
	SQR,
	TRI,
} waveform_t;
*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* in draw_utils.h
#define SCREEN_X 480
#define SCREEN_Y 272
#define FILTER_SELECT_BOX_X 95
#define FILTER_SELECT_BOX_Y 15
#define WAVEFORM_SELECT_BOX_X 160
#define WAVEFORM_SELECT_BOX_Y 50
#define FULL_SCREEN_BUT_X 160
#define FULL_SCREEN_BUT_Y 40
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TSResponse */
osThreadId_t TSResponseHandle;
const osThreadAttr_t TSResponse_attributes = {
  .name = "TSResponse",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for OscDrawing */
osThreadId_t OscDrawingHandle;
const osThreadAttr_t OscDrawing_attributes = {
  .name = "OscDrawing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/*in draw_utils.c
mode_t current_mode = OSC;
fir_type_t current_filter;
waveform_t current_waveform;
float waveform_mag = 1;
float waveform_freq = 10000000;
float waveform_duty = 0.3;
float osc_Vpp, osc_T, osc_V, osc_trigger, osc_freq;
int anomaly_flag = 0;
int test_flag = 0;
uint8_t databuffer[400];
uint8_t drawbuffer[400];
uint8_t prevdrawbuffer[400];
uint16_t trigger_index = 199;
uint8_t apply_filter = 0;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void *argument);
void StartTSResponse(void *argument);
void StartOscDrawing(void *argument);

/* USER CODE BEGIN PFP */

/* in draw_utils.h
void Display_DemoDescription(void);
void Display_Osc(void);
void Display_Sig_gen(void);
void Display_Filter(void);
void Display_pure(void);
void Select_Filter(uint16_t);
void Select_Waveform(uint16_t);
void touch_screen_response(void);
void draw_waveform(void);
void buffer_maker(float val, const char *prefix, char *out_buffer);
void float_to_string(float val, char *str, int decimal_places);
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* in draw_utils.c
uint8_t status = 0, gesture = 0;
uint8_t  lcd_status = LCD_OK, touch_screen_it = 0;
TS_StateTypeDef touch_screen_state;
*/
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
  MX_USART1_UART_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_DMA2D_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED1);

  /* Initialize the LCD */
  lcd_status = BSP_LCD_Init();
  if (lcd_status != LCD_OK)
  {
    Error_Handler();
  }
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER, LCD_FRAME_BUFFER);

  /* Get the LCD X and Y sizes */
  uint32_t lcd_x_size = BSP_LCD_GetXSize();
  uint32_t lcd_y_size = BSP_LCD_GetYSize();

  Display_DemoDescription();

  /* Set the LCD background and text colors */
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);

  BSP_LCD_DrawRect(50, 50, 100, 50);

  /* Display a string at the center of the screen */
  BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"LCD Test", CENTER_MODE);

  /* Draw a rectangle */
  BSP_LCD_SetTextColor(LCD_COLOR_RED);

  /* Draw a filled circle */
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_FillCircle(150, 150, 30);

  /* Display the sizes on the LCD */
  char size_message[50];
  snprintf(size_message, sizeof(size_message), "X: %lu, Y: %lu", lcd_x_size, lcd_y_size);
  BSP_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)size_message, CENTER_MODE);

  status = BSP_TS_Init(SCREEN_WIDTH, SCREEN_HEIGHT);
  BSP_TS_ITConfig();
  status = BSP_TS_ITGetStatus();

  /* ping test start */
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);

  Display_Osc();
  for(int i = 0; i < 400; ++i){
	  databuffer[i] = i % 255;
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TSResponse */
  TSResponseHandle = osThreadNew(StartTSResponse, NULL, &TSResponse_attributes);

  /* creation of OscDrawing */
  OscDrawingHandle = osThreadNew(StartOscDrawing, NULL, &OscDrawing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(touch_screen_it)
	  {
//		  touch_screen_response();
//		BSP_TS_ITClear();
//		BSP_TS_GetState(&touch_screen_state);
//		BSP_TS_Get_GestureId(&touch_screen_state);
//		if(touch_screen_state.gestureId != 0)
//			gesture = touch_screen_state.gestureId;
//		touch_screen_it = 0;
//		//	BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Touched", CENTER_MODE);
//		if(current_mode == PUR){
//			if(touch_screen_state.touchY[0] < SCREEN_Y - FULL_SCREEN_BUT_Y){
//				Display_Osc();
//				current_mode = OSC;
//				continue;
//			}
//		}
//		if(touch_screen_state.touchX[0] < 320 && touch_screen_state.touchX[0] > 160 && touch_screen_state.touchY[0] < 50){
//			Display_Sig_gen();
//			current_mode = SIG;
//		}
//		if(touch_screen_state.touchX[0] < 160 && touch_screen_state.touchY[0] < 50){
//			current_mode = OSC;
//			Display_Osc();
//		}
//		if(touch_screen_state.touchX[0] > 320 && touch_screen_state.touchY[0] < 50){
//			Display_Filter();
//			current_mode = FIL;
//		}
//		switch(current_mode){
//		case OSC:
//			if(touch_screen_state.touchY[0] > SCREEN_Y - FULL_SCREEN_BUT_Y && touch_screen_state.touchX[0] > SCREEN_X - FULL_SCREEN_BUT_X){
//				current_mode = PUR;
//				Display_pure();
//			}
//			break;
//		case SIG:
//			if(touch_screen_state.touchY[0] > 100 && touch_screen_state.touchY[0] < 200){
//				for(uint16_t j = 0; j < 3; j++){
//					if((touch_screen_state.touchX[0] > (WAVEFORM_SELECT_BOX_X*j)) && (touch_screen_state.touchX[0] < WAVEFORM_SELECT_BOX_X * (1 + j))){
//						Select_Waveform(j);
//						break;
//					}
//				}
//			}
//			break;
//		case FIL:
//			if(touch_screen_state.touchY[0] > 222){
//				for(uint16_t j = 0; j < 4; ++j){
//					if((touch_screen_state.touchX[0] > FILTER_SELECT_BOX_X * (j + 1)) && (touch_screen_state.touchX[0] < FILTER_SELECT_BOX_X * (j + 2))){
//						Select_Filter(j);
//						break;
//					}
//				}
//			}
//			break;
//		default:
//			break;
//		}
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20404768;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

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

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX0_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9
                           PC8 */
  GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
  GPIO_InitStruct.Pin = DCMI_D6_Pin|DCMI_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_INT_Pin */
  GPIO_InitStruct.Pin = Audio_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
  GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
  GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI2_SDB_Pin */
  GPIO_InitStruct.Pin = SAI2_SDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_D5_Pin */
  GPIO_InitStruct.Pin = DCMI_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : uSD_Detect_Pin */
  GPIO_InitStruct.Pin = uSD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_VSYNC_Pin */
  GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDMMC_D0_Pin */
  GPIO_InitStruct.Pin = SDMMC_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(SDMMC_D0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TP3_Pin NC2_Pin */
  GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_PWR_EN_Pin */
  GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_D4_Pin DCMI_D3_Pin DCMI_D0_Pin DCMI_D2_Pin
                           DCMI_D1_Pin */
  GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D0_Pin|DCMI_D2_Pin
                          |DCMI_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
  GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_A4_Pin ARDUINO_A5_Pin ARDUINO_A1_Pin ARDUINO_A2_Pin
                           ARDUINO_A3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A4_Pin|ARDUINO_A5_Pin|ARDUINO_A1_Pin|ARDUINO_A2_Pin
                          |ARDUINO_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_RXER_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_A0_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARDUINO_A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
  GPIO_InitStruct.Pin = DCMI_HSYNC_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LCD_INT_Pin)
	{
		touch_screen_it = 1;
//		BSP_TS_ITClear();
	}
}

void Display_DemoDescription(void)
{
  uint8_t desc[50];

  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);

  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);

  /* Clear the LCD */
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);

  /* Display LCD messages */
  BSP_LCD_DisplayStringAt(0, 10, (uint8_t *)"STM32F746G BSP", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 35, (uint8_t *)"Drivers examples", CENTER_MODE);

  /* Draw Bitmap */
  BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80) / 2, 65, (uint8_t *)stlogo);

  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() - 20, (uint8_t *)"Copyright (c) STMicroelectronics 2015", CENTER_MODE);

  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, BSP_LCD_GetYSize() / 2 + 15, BSP_LCD_GetXSize(), 60);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 30, (uint8_t *)"Press User Button to start :", CENTER_MODE);
  sprintf((char *)desc, "%s example", "BSP");
  BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2 + 45, (uint8_t *)desc, CENTER_MODE);
}
/* in draw_utils.c
void Display_Osc(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawRect(0, 0, 159, 50);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Oscilloscope", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Sig_gen", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Filter", RIGHT_MODE);

	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)"Full Screen", RIGHT_MODE);
	BSP_LCD_DrawRect(480 - FULL_SCREEN_BUT_X, 272 - FULL_SCREEN_BUT_Y, FULL_SCREEN_BUT_X - 1, FULL_SCREEN_BUT_Y);

	char buffer[50];
	sprintf(buffer, "trigger: %f", osc_trigger);
	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
}

void Display_pure(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);

//	char buffer[50];
//	sprintf(buffer, "Vpp: %f", osc_Vpp);
//	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, LEFT_MODE);
//	sprintf(buffer, "f: %f", osc_freq);
//	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
//	sprintf(buffer, "T: %f", osc_T);
//	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, RIGHT_MODE);
//	sprintf(buffer, "V: %f", osc_V);
//	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, RIGHT_MODE);
//
//	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"OSC", LEFT_MODE);
//	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"FIL", RIGHT_MODE);
}

void Display_Sig_gen(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Oscilloscope", LEFT_MODE);
	BSP_LCD_DrawRect(160, 0, 159, 50);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Sig_gen", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Filter", RIGHT_MODE);

	BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Select your waveform: ", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(8), (uint8_t *)"SINE", LEFT_MODE);
	BSP_LCD_DisplayStringAt(160, LINE(8), (uint8_t *)"SQUARE", LEFT_MODE);
	BSP_LCD_DisplayStringAt(320, LINE(8), (uint8_t *)"TRIANGLE", LEFT_MODE);
	BSP_LCD_DrawRect(160*current_waveform, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);

	char buffer[50];
	sprintf(buffer, "Vpp: %f", waveform_mag);
	BSP_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)buffer, LEFT_MODE);
	sprintf(buffer, "Freq: %f", waveform_freq);
	BSP_LCD_DisplayStringAt(0, LINE(12), (uint8_t *)buffer, LEFT_MODE);
	sprintf(buffer, "Duty ratio (for square): %f", waveform_duty);
	BSP_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)buffer, LEFT_MODE);
}

void Display_Filter(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Oscilloscope", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Sig_gen", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Filter", RIGHT_MODE);
	BSP_LCD_DrawRect(320, 0, 159, 50);

	BSP_LCD_DisplayStringAt(0, LINE(13), (uint8_t *)"Select your filter: ", LEFT_MODE);
	BSP_LCD_DisplayStringAt(96, LINE(14), (uint8_t *)"LPF", LEFT_MODE);
	BSP_LCD_DisplayStringAt(192, LINE(14), (uint8_t *)"HPF", LEFT_MODE);
	BSP_LCD_DisplayStringAt(288, LINE(14), (uint8_t *)"BPF", LEFT_MODE);
	BSP_LCD_DisplayStringAt(384, LINE(14), (uint8_t *)"BSF", LEFT_MODE);
	BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * (1 + current_filter), 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);

	BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)"GO!", CENTER_MODE);

	char buffer[50];
	sprintf(buffer, "fc1 (not used in LPF, HPF): %f", fir_fc1);
	BSP_LCD_DisplayStringAt(50, LINE(15), (uint8_t *)buffer, LEFT_MODE);
	sprintf(buffer, "fc2 (fc for LPF, HPF): %f", fir_fc2);
	BSP_LCD_DisplayStringAt(50, LINE(16), (uint8_t *)buffer, LEFT_MODE);
}

void Select_Waveform(uint16_t waveform){
	for(int i = 0; i < 3; ++i){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(WAVEFORM_SELECT_BOX_X*i, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
	}
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	switch(waveform){
	case 0:
		BSP_LCD_DrawRect(0, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
		current_waveform = SIN;
		break;
	case 1:
		BSP_LCD_DrawRect(160, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
		current_waveform = SQR;
		break;
	case 2:
		BSP_LCD_DrawRect(320, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
		current_waveform = TRI;
		break;
	}
}

void Select_Filter(uint16_t selected_fir){
	for(int i = 0; i < 4; ++i){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X*(i+1), 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
	}
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	switch(selected_fir){
	case 0:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_LOW;
		break;
	case 1:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * 2, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_HIGH;
		break;
	case 2:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * 3, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_BANDPASS;
		break;
	case 3:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * 4, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_BANDSTOP;
		break;
	}
}

void touch_screen_response(){
	BSP_TS_ITClear();
	BSP_TS_GetState(&touch_screen_state);
	BSP_TS_Get_GestureId(&touch_screen_state);
	if(touch_screen_state.gestureId != 0)
		gesture = touch_screen_state.gestureId;
	touch_screen_it = 0;
	if(touch_screen_state.touchX[0] > 320) anomaly_flag = 1;
	if(current_mode == PUR){
		if(touch_screen_state.touchY[0] < 50){
			if(touch_screen_state.touchX[0] < 40){
				current_mode = OSC;
				Display_Osc();
				apply_filter = 0;
			}else if(touch_screen_state.touchX[0] > 440){
				current_mode = FIL;
				Display_Filter();
				apply_filter = 1;
			}
		}
		return;
	}
	if(touch_screen_state.touchX[0] < 320 && touch_screen_state.touchX[0] > 160 && touch_screen_state.touchY[0] < 50){
		Display_Sig_gen();
		current_mode = SIG;
	}
	if(touch_screen_state.touchX[0] < 160 && touch_screen_state.touchY[0] < 50){
		current_mode = OSC;
		Display_Osc();
		apply_filter = 0;
	}
	if(touch_screen_state.touchX[0] > 320 && touch_screen_state.touchY[0] < 50){
		Display_Filter();
		current_mode = FIL;
		apply_filter = 1;
	}
	switch(current_mode){
	case OSC:
		if(touch_screen_state.touchY[0] > SCREEN_Y - FULL_SCREEN_BUT_Y && touch_screen_state.touchX[0] > SCREEN_X - FULL_SCREEN_BUT_X){
			current_mode = PUR;
			Display_pure();
		}
		return;
	case SIG:
		if(touch_screen_state.touchY[0] > 100 && touch_screen_state.touchY[0] < 200){
			for(uint16_t j = 0; j < 3; j++){
				if((touch_screen_state.touchX[0] > (WAVEFORM_SELECT_BOX_X*j)) && (touch_screen_state.touchX[0] < WAVEFORM_SELECT_BOX_X * (1 + j))){
					Select_Waveform(j);
					break;
				}
			}
		}
		return;
	case FIL:
		if(touch_screen_state.touchY[0] > 222){
			for(uint16_t j = 0; j < 4; ++j){
				if((touch_screen_state.touchX[0] > FILTER_SELECT_BOX_X * (j + 1)) && (touch_screen_state.touchX[0] < FILTER_SELECT_BOX_X * (j + 2))){
					Select_Filter(j);
					break;
				}
			}
		}else if(touch_screen_state.touchY[0] > 50){
			current_mode = PUR;
			Display_pure();
		}
		return;
	default:
		return;
	}
}

void float_to_string(float val, char *str, int decimal_places) {
    if (val < 0) {
        *str++ = '-';
        val = -val;
    }

    int int_part = (int)val;
    int frac_part = (int)((val - int_part) * pow(10, decimal_places));

    // Convert integer part
    itoa(int_part, str, 10);
    while (*str) str++;  // Move pointer to end

    *str++ = '.';

    // Convert fractional part with leading zero if needed
    if (decimal_places == 2 && frac_part < 10)
        *str++ = '0';

    itoa(frac_part, str, 10);
}

void buffer_maker(float val, const char *prefix, char *out_buffer) {
    char numbuff[20];
    float_to_string(val, numbuff, 2);

    strcpy(out_buffer, prefix);      // Copy prefix like "Vpp: "
    strcat(out_buffer, numbuff);     // Append converted float
}

void draw_waveform(){
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"OSC", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"FIL", RIGHT_MODE);

	char buffer[50];
	buffer_maker(osc_Vpp, "Vpp: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, LEFT_MODE);
	buffer_maker(osc_freq, "f: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
	buffer_maker(osc_T, "T: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, RIGHT_MODE);
	buffer_maker(osc_V, "V: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, RIGHT_MODE);

	for(int j = 0; j < 400; ++j){
		uint16_t db_index = (trigger_index - 199 + 400 + j) % 400;
		float point_y = 255 - databuffer[db_index];
		point_y = (point_y/255.0) * 215;
		drawbuffer[j] = (uint8_t) point_y;
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(j + 40, prevdrawbuffer[j], 1, 1);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DrawRect(j + 40, drawbuffer[j], 1, 1);
		prevdrawbuffer[j] = drawbuffer[j];
		databuffer[db_index] = (databuffer[db_index] + 5) % 255;
	}
}
*/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTSResponse */
/**
* @brief Function implementing the TSResponse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTSResponse */
void StartTSResponse(void *argument)
{
  /* USER CODE BEGIN StartTSResponse */
  /* Infinite loop */
  for(;;)
  {
	if(touch_screen_it) touch_screen_response();
//	BSP_TS_ITClear();
//    osDelay(1);
	HAL_Delay(1);
  }
  /* USER CODE END StartTSResponse */
}

/* USER CODE BEGIN Header_StartOscDrawing */
/**
* @brief Function implementing the OscDrawing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOscDrawing */
void StartOscDrawing(void *argument)
{
  /* USER CODE BEGIN StartOscDrawing */
  /* Infinite loop */
  for(;;)
  {
	if(current_mode == PUR){
		HAL_Delay(1);
		draw_waveform();

//		BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"OSC", LEFT_MODE);
//		BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"FIL", RIGHT_MODE);
//
//		char buffer[50];
//		buffer_maker(osc_Vpp, "Vpp: ", buffer);
//		BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, LEFT_MODE);
//		buffer_maker(osc_freq, "f: ", buffer);
//		BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
//		buffer_maker(osc_T, "T: ", buffer);
//		BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, RIGHT_MODE);
//		buffer_maker(osc_V, "V: ", buffer);
//		BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, RIGHT_MODE);
//
//		for(int j = 0; j < 400; ++j){
//			uint16_t db_index = (trigger_index - 199 + 400 + j) % 400;
//			float point_y = 255 - databuffer[db_index];
//			point_y = (point_y/255.0) * 215;
//			drawbuffer[j] = (uint8_t) point_y;
//			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//			BSP_LCD_DrawRect(j + 40, prevdrawbuffer[j], 1, 1);
//			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
//			BSP_LCD_DrawRect(j + 40, drawbuffer[j], 1, 1);
//			prevdrawbuffer[j] = drawbuffer[j];
//			databuffer[db_index] = (databuffer[db_index] + 5) % 255;
//		}
	}
	HAL_Delay(1);
//    osDelay(1);
  }
  /* USER CODE END StartOscDrawing */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
