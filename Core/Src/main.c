/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdlib.h"
#include "string.h"

#include "usbd_cdc_if.h"
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/*
 * Hardware Configuration / Pinout
 *
 * Output side:
 * Mux Ch. | STM Ch. | Function
 * --------+---------+----------
 *    1    |   2.4   | Rudder
 *    2    |   2.3   | Sails
 *    3    |   2.2   | Unused
 *    4    |   2.1   | Unused
 *
 * Input side:
 * Mux Ch. | STM Ch.  | Connector Ch. | RC Ch. | RC Function
 * --------+----------+---------------+--------+------------------------
 *  (SW)   |    3.2   |   1           |   5    |  Switch TODO
 *   1     |    3.4   |   2           |   1    |  Aileron / right vert.
 *   2     |    3.1   |   3           |   3    |  Throttle / left hor.
 *   3     |    3.3   |   4           |   X    |  Unused
 *   4     |    1.2   |   5           |   X    |  Unused
 */

/* vvv configuration constants, change as needed vvv*/

/* PWM inputs, depending on clock and prescaler */
const uint32_t PWM_MIN_PULSE = 800;
const uint32_t PWM_MAX_PULSE = 1900;
/* PWM value > this is read as "switch on" */
const uint32_t SWITCH_ON_THRESHOLD = 1500;
/* rudder channel limits */ /* TODO */
const uint32_t CHAN1_OUT_MINVAL = 1300;
const uint32_t CHAN1_OUT_MAXVAL = 1700;
/* sail channel limits */
const uint32_t CHAN2_OUT_MINVAL = 1600;
const uint32_t CHAN2_OUT_MAXVAL = 1900;
/* main sail winch address used here */
const uint32_t CAN_ADDRESS_SAIL = 0x1E0C0000;
/* TODO: check */
const int32_t MAX_SAIL_ANGLE = 90;
const int32_t MIN_SAIL_ANGLE = -90;
/* rudder can address *//* TODO: check if this is correct */
const uint32_t CAN_ADDRESS_RUDDER = 0x1E0B0000;
const int32_t MAX_RUDDER_ANGLE = 90;
const int32_t MIN_RUDDER_ANGLE = -90;

/* uncomment this to log every input etc. */
#define DEBUG_PRINTS_ENABLE
#ifdef DEBUG_PRINTS_ENABLE
#warning "Lots of debug prints enabled!"
char debug_buffer[100];
#define D(...) do {snprintf(debug_buffer, sizeof(debug_buffer), __VA_ARGS__); CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));}while(0)
#else
#define D(...)
#endif

/* uncomment this to always use RC inputs, for testing */
/*#define FORCE_RC_OVERRIDE_ONLY*/
#ifdef FORCE_RC_OVERRIDE_ONLY
#warning "FORCE_RC_OVERRIDE_ONLY is enabled, RasPi control is disabled!"
/* RC control only, for testing */
void set_rc_override_enabled(uint32_t enable) {
/*	D("RC override force-enabled, no effect\r\n"); */
}
uint32_t is_rc_override_enabled() {
	return 1;
}
#else
/* RC switch determines if CAN commands are overridden with RC */
uint32_t rc_override_enabled = 0;
void set_rc_override_enabled(uint32_t enable) {
	rc_override_enabled = !!enable;
	/*D("RC override set to %lu\r\n", enable);*/
}
uint32_t is_rc_override_enabled() {
	return rc_override_enabled;
}
#endif

/* aliases to make talking to the mux'ed PWMs a bit nicer */
struct rc_pwm_channel {
	TIM_HandleTypeDef *timer;
	uint32_t active_channel;
	uint32_t tim_channel;
	volatile uint32_t *CCR;
};

struct rc_pwm_channel pwm_out_1;
struct rc_pwm_channel pwm_out_2;
struct rc_pwm_channel pwm_in_1;
struct rc_pwm_channel pwm_in_2;
struct rc_pwm_channel pwm_in_3;

void init_rc_pwms(void) {
	pwm_out_1.active_channel = HAL_TIM_ACTIVE_CHANNEL_4;
	pwm_out_2.active_channel = HAL_TIM_ACTIVE_CHANNEL_3;
	pwm_in_1.active_channel = HAL_TIM_ACTIVE_CHANNEL_2;
	pwm_in_2.active_channel = HAL_TIM_ACTIVE_CHANNEL_4;
	pwm_in_3.active_channel = HAL_TIM_ACTIVE_CHANNEL_1;

	pwm_out_1.tim_channel = TIM_CHANNEL_4;
	pwm_out_2.tim_channel = TIM_CHANNEL_3;
	pwm_in_1.tim_channel = TIM_CHANNEL_2;
	pwm_in_2.tim_channel = TIM_CHANNEL_4;
	pwm_in_3.tim_channel = TIM_CHANNEL_1;

	pwm_out_1.timer = &htim2;
	pwm_out_2.timer = &htim2;
	pwm_in_1.timer = &htim3;
	pwm_in_2.timer = &htim3;
	pwm_in_3.timer = &htim3;

	pwm_out_1.CCR = &pwm_out_1.timer->Instance->CCR4;
	pwm_out_2.CCR = &pwm_out_1.timer->Instance->CCR3;
	pwm_in_1.CCR = &pwm_in_1.timer->Instance->CCR2;
	pwm_in_2.CCR = &pwm_in_2.timer->Instance->CCR4;
	pwm_in_3.CCR = &pwm_in_3.timer->Instance->CCR1;
}

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t scale_pwm_value(int32_t value, int32_t oldmin, int32_t oldmax,
		int32_t newmin, int32_t newmax) {
	value = (value - oldmin) * (newmax - newmin);
	return (value / (oldmax - oldmin) + newmin);
}

void set_timer_polarity_and_reset(struct rc_pwm_channel *t, uint32_t polarity) {
	__HAL_TIM_SET_COUNTER(t->timer, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(t->timer, t->tim_channel, polarity);
}

uint32_t get_timer_polarity(struct rc_pwm_channel *t) {
	/* can't find a handy macro */
	/* D("reg: 0x%lx\r\n", (t->timer->Instance->CCER >> t->tim_channel) & 2);*/
	return (t->timer->Instance->CCER >> t->tim_channel) & 2;
}

uint32_t is_channel_active(TIM_HandleTypeDef *htim,
		struct rc_pwm_channel *pwm_chan) {
	return htim == pwm_chan->timer && htim->Channel == pwm_chan->active_channel;
}

uint32_t identify_timer(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) return 1;
	if (htim->Instance == TIM2) return 2;
	if (htim->Instance == TIM3) return 3;
	return -1;
}


/*
 * PWM input interrupt callback 1
 * Note: apparently, only one debug print statement per method call is allowed
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	/* we have triggered on the rising edge of a PWM pulse */
	D("rising egde from %lu\r\n", identify_timer(htim));

	/* first, reset all counters */
	__HAL_TIM_SET_COUNTER(pwm_in_1.timer, 0);
	__HAL_TIM_SET_COUNTER(pwm_in_2.timer, 0);
	__HAL_TIM_SET_COUNTER(pwm_in_3.timer, 0);
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	/* then set to trigger on falling edge */
	__HAL_TIM_SET_CAPTUREPOLARITY(pwm_in_1.timer, pwm_in_1.tim_channel,
			TIM_INPUTCHANNELPOLARITY_FALLING);
	__HAL_TIM_SET_CAPTUREPOLARITY(pwm_in_2.timer, pwm_in_2.tim_channel,
			TIM_INPUTCHANNELPOLARITY_FALLING);
	__HAL_TIM_SET_CAPTUREPOLARITY(pwm_in_3.timer, pwm_in_3.tim_channel,
			TIM_INPUTCHANNELPOLARITY_FALLING);

	/* and disable interrupts */
	__HAL_TIM_DISABLE_IT(pwm_in_1.timer, pwm_in_1.active_channel << 1);
	__HAL_TIM_DISABLE_IT(pwm_in_2.timer, pwm_in_2.active_channel << 1);
	__HAL_TIM_DISABLE_IT(pwm_in_3.timer, pwm_in_3.active_channel << 1);

	/* finally, start a timer to collect our values in 10 ms (half a PWM period) */
	HAL_TIM_Base_Start_IT(&htim1);
}

/*
 * PWM input interrupt callback 2
 * Note: apparently, only one debug print statement per method call is allowed
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* we have triggered after waiting 10 ms for all the PWM signals */

	/*D("Ch 2 in %lu, Ch3 in %lu, Ch 1 in %lu, 10ms timer %lu, triggered by %lu\r\n ",
			*pwm_in_2.CCR, *pwm_in_3.CCR, *pwm_in_1.CCR, htim1.Instance->CNT,
			identify_timer(htim));*/

	/* connector 1 => mode switch */
	/* long pulse means override, short or no pulse means no override */
	set_rc_override_enabled((*pwm_in_1.CCR) > SWITCH_ON_THRESHOLD);

	if (is_rc_override_enabled()) {
		/* connector 2 => rudder at channel 1 */
		*pwm_out_1.CCR = scale_pwm_value(*pwm_in_2.CCR, PWM_MIN_PULSE,
				PWM_MAX_PULSE, CHAN1_OUT_MINVAL, CHAN1_OUT_MAXVAL);
		/* connector 3 => sail at channel 2 */
		*pwm_out_2.CCR = scale_pwm_value(*pwm_in_3.CCR, PWM_MIN_PULSE,
				PWM_MAX_PULSE, CHAN2_OUT_MINVAL, CHAN2_OUT_MAXVAL);
		D("Rudder out: %lu, sail out: %lu\r\n", *pwm_out_1.CCR, *pwm_out_2.CCR);
	}

	/* disable 10 ms timer */
	HAL_TIM_Base_Stop_IT(&htim1);

	/* trigger on rising edge */
	__HAL_TIM_SET_CAPTUREPOLARITY(pwm_in_1.timer, pwm_in_1.tim_channel,
			TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_SET_CAPTUREPOLARITY(pwm_in_2.timer, pwm_in_2.tim_channel,
			TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_SET_CAPTUREPOLARITY(pwm_in_3.timer, pwm_in_3.tim_channel,
			TIM_INPUTCHANNELPOLARITY_RISING);

	/* enable interrupts on channel timers again */
	__HAL_TIM_ENABLE_IT(pwm_in_1.timer, pwm_in_1.active_channel << 1);
	__HAL_TIM_ENABLE_IT(pwm_in_2.timer, pwm_in_2.active_channel << 1);
	__HAL_TIM_ENABLE_IT(pwm_in_3.timer, pwm_in_3.active_channel << 1);

}

/* CAN interrupt callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	D("CAN received %02x %02x %02x %02x %02x %02x %02x %02x\n", RxData[0],
			RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6],
			RxData[7]);

	if (!is_rc_override_enabled()) {
		return;
	}

	if (RxHeader.ExtId == CAN_ADDRESS_RUDDER) {
		/* Out-Ch 1 is rudder */
		*pwm_out_1.CCR = scale_pwm_value(*(int32_t*) RxData,
				MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE, CHAN1_OUT_MINVAL,
				CHAN1_OUT_MAXVAL);
		D("CAN got rudder angle %lu, output %lu", *(int32_t* ) RxData,
				*pwm_out_1.CCR);
	} else if (RxHeader.ExtId == CAN_ADDRESS_SAIL) {
		/* Out-Ch 2 is sail */
		*pwm_out_2.CCR = scale_pwm_value(*(int32_t*) RxData,
				MIN_RUDDER_ANGLE, MAX_RUDDER_ANGLE, CHAN2_OUT_MINVAL,
				CHAN2_OUT_MAXVAL);
		D("CAN got sail angle %lu, output %lu", *(int32_t* ) RxData,
				*pwm_out_2.CCR);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

#ifdef DEBUG_PRINTS_ENABLE
	/* give the user some time to connect via usb/uart */
	HAL_Delay(5*1000);
#endif

	D("Init...\r\n");

	/* init pwm structs */
	init_rc_pwms();

	/* start capturing pwm inputs */
	HAL_TIM_IC_Start_IT(pwm_in_1.timer, pwm_in_1.tim_channel);
	HAL_TIM_IC_Start_IT(pwm_in_2.timer, pwm_in_2.tim_channel);
	HAL_TIM_IC_Start_IT(pwm_in_3.timer, pwm_in_3.tim_channel);


	/* Initialize CAN filter */
//	/* TODO: Copied verbatim from robooter motor controller, wtf does it do? Can we delete this? */
//	sFilterConfig.FilterBank = 0;
//	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//	sFilterConfig.FilterIdHigh = 0x0000;
//	sFilterConfig.FilterIdLow = 0x0000;
//	sFilterConfig.FilterMaskIdHigh = 0x0000;
//	sFilterConfig.FilterMaskIdLow = 0x0000;
//	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
//	sFilterConfig.FilterActivation = ENABLE;
//	sFilterConfig.SlaveStartFilterBank = 14;
//	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
//		Error_Handler();
//	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
					| CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Error_Handler();
	}

	D("Ready\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* heart beat */
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		/* D("."); */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

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
	while (1) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
