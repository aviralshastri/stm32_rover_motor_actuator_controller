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
#include "usb_device.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

TIM_HandleTypeDef htim2;
uint8_t buffer[64];

// PCA9685 I2C address (7-bit address 0x40 shifted left for HAL)
#define PCA9685_ADDRESS (0x40 << 1)  // 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x00        // as in the datasheet page no 10/52
#define PCA9685_MODE2         0x01        // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x06        // as in the datasheet page no 10/52
#define PCA9685_ALL_LED_ON_L  0xFA        // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52

// Servo constants
#define SERVO_MIN_PULSE 102.4f    // 0.5ms pulse width
#define SERVO_MAX_PULSE 511.9f    // 2.5ms pulse width
#define SERVO_MAX_ANGLE 180.0f

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 100) != HAL_OK) {
    return; // Handle I2C error
  }

  if (Value == 0)
    readValue &= ~(1 << Bit);
  else
    readValue |= (1 << Bit);

  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 100);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;

  if(frequency >= 1526) {
    prescale = 0x03;
  }
  else if(frequency <= 24) {
    prescale = 0xFF;
  }
  else {
    // Calculate prescale value: prescale = round(osc_clock / (4096 * freq)) - 1
    // Internal 25 MHz oscillator as in the datasheet page no 1/52
    prescale = (uint8_t)(25000000.0f / (4096.0f * frequency) - 1.0f + 0.5f);
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;
  }

  // Put PCA9685 to sleep before changing prescaler
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_Delay(1); // Wait for sleep to take effect

  // Set prescaler
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 100);

  // Wake up PCA9685
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  HAL_Delay(1); // Wait for oscillator to stabilize

  // Restart PWM
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
  HAL_Delay(1);
}

void PCA9685_Init(uint16_t frequency)
{
  // Reset PCA9685
  uint8_t mode1 = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1, &mode1, 1, 100);
  HAL_Delay(1);

  // Set PWM frequency
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo

  // Enable auto-increment
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);

  // Configure MODE2 for totem pole output (not open-drain)
  uint8_t mode2 = 0x04; // OUTDRV = 1 for totem pole output
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE2, 1, &mode2, 1, 100);
  HAL_Delay(1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  if (Channel > 15) return; // PCA9685 has 16 channels (0-15)

  uint8_t registerAddress;
  uint8_t pwm[4];

  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);

  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;        // ON_L
  pwm[1] = (OnTime >> 8) & 0x0F; // ON_H (only lower 4 bits)
  pwm[2] = OffTime & 0xFF;       // OFF_L
  pwm[3] = (OffTime >> 8) & 0x0F;// OFF_H (only lower 4 bits)

  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 100);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  if (Channel > 15) return; // PCA9685 has 16 channels (0-15)
  if (Angle < 0) Angle = 0;
  if (Angle > SERVO_MAX_ANGLE) Angle = SERVO_MAX_ANGLE;

  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  Value = (Angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / SERVO_MAX_ANGLE) + SERVO_MIN_PULSE;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

void PCA9685_SetAllPWM(uint16_t OnTime, uint16_t OffTime)
{
  uint8_t pwm[4];

  // See datasheet for ALL_LED registers
  pwm[0] = OnTime & 0xFF;        // ALL_LED_ON_L
  pwm[1] = (OnTime >> 8) & 0x0F; // ALL_LED_ON_H (only lower 4 bits)
  pwm[2] = OffTime & 0xFF;       // ALL_LED_OFF_L
  pwm[3] = (OffTime >> 8) & 0x0F;// ALL_LED_OFF_H (only lower 4 bits)

  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_ALL_LED_ON_L, 1, pwm, 4, 100);
}

void PCA9685_SetAllServoAngle(float Angle)
{
  if (Angle < 0) Angle = 0;
  if (Angle > SERVO_MAX_ANGLE) Angle = SERVO_MAX_ANGLE;

  float Value = (Angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / SERVO_MAX_ANGLE) + SERVO_MIN_PULSE;
  PCA9685_SetAllPWM(0, (uint16_t)Value);
}

uint16_t MapPowerToPulse(uint8_t power) {
    if (power > 100) power = 100;
    return 1000 + (power * 10);
}

void SendFeedback(const char* message) {
    CDC_Transmit_FS((uint8_t*)message, strlen(message));
}

  // Single ESC Control
  void Single_Esc_Control(uint8_t* channel, uint16_t* value) {
      uint16_t pwmValue = MapPowerToPulse(*value);

      switch(*channel) {
          case 1: TIM2->CCR1 = pwmValue; break;
          case 2: TIM2->CCR2 = pwmValue; break;
          case 3: TIM2->CCR3 = pwmValue; break;
          case 4: TIM2->CCR4 = pwmValue; break;
          default: return;
      }

      char msg[64];
      sprintf(msg, "ESC %d set to %d\r\n", *channel, pwmValue);
      SendFeedback(msg);
  }

  // Batch ESC Control
  void Batch_Esc_Control(uint8_t* n, uint16_t* values) {
      for(uint8_t i = 0; i < *n; i++) {
          uint16_t pwmValue = MapPowerToPulse(values[i]);
          switch(i+1) {
              case 1: TIM2->CCR1 = pwmValue; break;
              case 2: TIM2->CCR2 = pwmValue; break;
              case 3: TIM2->CCR3 = pwmValue; break;
              case 4: TIM2->CCR4 = pwmValue; break;
              default: continue;
          }

          char msg[64];
          sprintf(msg, "ESC %d set to %d\r\n", i+1, pwmValue);
          SendFeedback(msg);
      }
  }

  // Burst ESC Control
  void Burst_Esc_Control(uint8_t* n, uint16_t* value) {
      uint16_t pwmValue = MapPowerToPulse(*value);
      for(uint8_t i = 0; i < *n; i++) {
          switch(i+1) {
              case 1: TIM2->CCR1 = pwmValue; break;
              case 2: TIM2->CCR2 = pwmValue; break;
              case 3: TIM2->CCR3 = pwmValue; break;
              case 4: TIM2->CCR4 = pwmValue; break;
              default: continue;
          }

          char msg[64];
          sprintf(msg, "ESC %d burst set to %d\r\n", i+1, pwmValue);
          SendFeedback(msg);
      }
  }

  // Command handler
  void Control_Command_Handler(uint8_t* Buf, uint32_t Len)
  {
      uint8_t escValue, escChannel, n;
      uint16_t valueArray[16];

      switch(Buf[1]){
          case 0x00: // Single ESC Control
              escValue = Buf[2];
              escChannel = Buf[3];
              {
                  uint16_t pwmValue = escValue;
                  Single_Esc_Control(&escChannel, &pwmValue);
              }
              break;

          case 0x02: // Batch ESC Control
              for(uint8_t i=0; i<4; i++) {
                  valueArray[i] = Buf[2+i];
              }
              n = 4;
              Batch_Esc_Control(&n, valueArray);
              break;

          case 0x04: // Burst ESC Control
              escValue = Buf[2];
              {
                  uint16_t pwmValue = escValue;
                  n = 4;
                  Burst_Esc_Control(&n, &pwmValue);
              }
              break;

          default:
              SendFeedback("Invalid Command\r\n");
              break;
      }
  }


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
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  TIM2->CCR1=1500;
  TIM2->CCR2=1500;
  TIM2->CCR3=1500;
  TIM2->CCR4=1500;
  PCA9685_Init(50);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  PCA9685_SetAllServoAngle(0);
	      HAL_Delay(1000);  // Wait 1 second

	      // Move all servos to 45°
	      PCA9685_SetAllServoAngle(45);
	      HAL_Delay(1000);

	      // Move all servos to 90°
	      PCA9685_SetAllServoAngle(90);
	      HAL_Delay(1000);

	      // Move all servos to 135°
	      PCA9685_SetAllServoAngle(135);
	      HAL_Delay(1000);

	      // Move all servos to 180°
	      PCA9685_SetAllServoAngle(180);
	      HAL_Delay(1000);

	      // Move back to center (90°)
	      PCA9685_SetAllServoAngle(90);
	      HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Extra_Setup_Button_Pin */
  GPIO_InitStruct.Pin = Extra_Setup_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Extra_Setup_Button_GPIO_Port, &GPIO_InitStruct);

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
