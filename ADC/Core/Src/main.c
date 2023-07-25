#include "main.h"
#include "usbpd.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
//#include <math.h>

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define ABS(x) ((x) < 0 ? -(x) : (x))

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CORDIC_HandleTypeDef hcordic;

FMAC_HandleTypeDef hfmac;

I2C_HandleTypeDef hi2c1;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

enum led_states {LED_OFF = 0, LED_READY, LED_RUNNING, LED_DONE, LED_ERROR};
enum fsm_states {IDLE = 0, INIT, RUNNING, DONE, ERR};

// FLAGS
uint8_t evaluate_FSM = 0;

struct fsm_t {
  uint8_t led; // led mode
  uint8_t mode; // FSM mode
} fsm;

uint16_t adc[9]; // ADC buf

struct settings_t {
  uint32_t samplingFreq; // sampling frequency
  uint16_t psc; // sampling frequency
  uint16_t arr; // sampling frequency
  uint16_t OverSampling; // oversampling ration; 0, x2, x4, x8, x16
  uint16_t PGAgain[3]; // PGA gain
  uint8_t configChanged; // flag for config has changed 2^0 frequency has changed; 2^1 psc, arr has changed; 2^2 PGA has changed
  uint8_t telegramSize;
} settings;

uint8_t usbDataReady; // flag for data ready
uint16_t usbDataLength; // length of USB data
uint8_t usbBuf[32]; // data buffer

uint8_t adcDMAready = 0;
struct telegram_t {
  uint16_t data[257][9];
  uint16_t idx; // current index
  uint16_t lastidx; // last index
  uint16_t sendcnt; // send counter
} t;

struct usb_t {
  uint8_t state; // state to determine wether the USB send flag should be set
  uint8_t send; // flag for USB send
  uint8_t res; // usb send result flag
} usb;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UCPD1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FMAC_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM6_Init(void);
void led_task(void);
void get_timer_value(struct settings_t *set);
void settings_task(struct settings_t *set);
void adc_task(void);
void fsm_task(void);
void send_usb(uint8_t len);
void usb_parser(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UCPD1_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();

  // LED
  HAL_TIM_Base_Start_IT(&htim6);
  // ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &adc[0], 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &adc[4], 5);
  HAL_TIM_Base_Start_IT(&htim4);

  settings.samplingFreq = 100;
  settings.configChanged = 1;
  settings.telegramSize = 127;

  //MX_USBPD_Init();
  while (1)
  {
    //USBPD_DPM_Run();
    settings_task(&settings);

    //if(adc_ready) adc_task();
    if(evaluate_FSM) fsm_task();
    //if(evaluate_LED) led_task();
    if(usbDataReady) usb_parser();

  }
}

uint32_t number;
void usb_parser(){
  if(usbDataLength >= 4){
    // extract channel number, char - 48 (or ascii '0') = sent integer
    //uint8_t channel = usb_buf[3] - '0';
    number = atoi((char *)usbBuf + 3);

    if(strncmp("SPS", (char *) usbBuf, 3) == 0){
      if(number > 0 && number < 100000){
        settings.samplingFreq = number;
      }
    } else if(strncmp("SET", (char *) usbBuf, 3) == 0) {
      settings.configChanged = 1;
    }
    // else if(strncmp("RST", (char *) usb_buf, 3) == 0){
    //  t[channel].rst = 1;
    //}

    /*else if(strncmp("STR", (char *) usb_buf, 3) == 0){
      telegram[channel].usb.state = 1;
    } else if(strncmp("STP", (char *) usb_buf, 3) == 0){
      telegram[channel].usb.state = 0;
    } else if(strncmp("STP", (char *) usb_buf, 3) == 0){
      telegram[channel].usb.state = 0;
    } else if(strncmp("STP", (char *) usb_buf, 3) == 0){
      telegram[channel].usb.state = 0;
    } else if(strncmp("HST", (char *) usb_buf, 3) == 0){
      for(uint8_t i = 0; i <= HISTORY-1; i++){
        send_telegram(telegram[channel].serialNumber, &telegram[channel].hist[i]);
      }
    } else if(strncmp("TSK", (char *) usb_buf, 3) == 0){
      memcpy((void *)&t[channel], (void *)&usb_buf[4], 28);
    } else {
      CDC_Transmit_FS((uint8_t *) "Could not parse command\r\n", 25);
    }
    */
  }
  usbDataReady = 0;
}

uint8_t idx = 0;
uint16_t curridx;
void send_usb(uint8_t len){
  if(t.idx == 0){
    idx = 255;
  } else {
    idx = t.idx;
  }

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);

  char package[len][56];
  for(uint16_t i = 0; i < len; i++){
    memset(package[i], '\0' , 56);
    curridx = (i + t.lastidx) % 256;
    sprintf(package[i], "%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n", HAL_GetTick(),
                                                              t.data[curridx][0],
                                                              t.data[curridx][1],
                                                              t.data[curridx][2],
                                                              t.data[curridx][3],
                                                              t.data[curridx][4],
                                                              t.data[curridx][5],
                                                              t.data[curridx][6],
                                                              t.data[curridx][7],
                                                              t.data[curridx][8]);
  }

  usb.res = CDC_Transmit_FS((uint8_t *) package, 56*len);

  t.lastidx = idx;

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
}

uint8_t idxdif = 0;
void adc_task(){
  if(adcDMAready == 3){
    adcDMAready = 0;
    memcpy(&t.data[t.idx][0], &adc[0], 18);

    t.idx = (t.idx <= 255) ? (t.idx + 1) : 0;

    t.sendcnt++;

    if(t.sendcnt > settings.telegramSize){
      send_usb(t.sendcnt);
      t.sendcnt = 0;
    }
  }
}

// finite state machine
void fsm_task(){
    switch (fsm.mode){
      case IDLE:
      default:
        // Idle means:
        //   - Adapt to settings
        //   - Start on USB command

        if(0){
          fsm.mode = ERROR;
        } else if(0){
          fsm.mode = RUNNING;
        }
        break;
      case INIT:
        // ADC
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &adc[0], 4);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &adc[4], 5);
        HAL_TIM_Base_Start_IT(&htim4);

        t.idx = 0;

        if(0){
          fsm.mode = ERROR;
        } else {
          fsm.mode = RUNNING;
        }
        break;
      case RUNNING:
        // Running means:
        //   - Collect ADC Samples
        //   - Send to USB
        //   - Adapt to change in settings
        
        usb.send = 1;
        

        if(0){
          fsm.mode = ERROR;
        } else if(0){
          fsm.mode = DONE;
        }
        break;
      case DONE:
        // Done means:
        //   - Finished with the task
        //   - Cleanup and return to IDLE

        if(0){
          fsm.mode = IDLE;
        }
        break;
      case ERR:
        // Error means:
        //   - Something went wrong, not sure what this means yet

        // if error reset
        if(0){
          fsm.mode = IDLE;
        }
        break;
    }
}

void led_task(){
  /*
  switch (fsm.led){
  case LED_OFF:
  default:
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
    break;
  case LED_READY:
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 1);
    break;
  case LED_RUNNING:
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
    break;
  case LED_DONE:
    if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1))HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
    break;
  case LED_ERROR:
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
    break;
  }
  */
}

// settings
void settings_task(struct settings_t *set){
  if(set->configChanged == 1){
    get_timer_value(set);
    set->configChanged &= ~(1);
  }
  if(set->configChanged == 2){
    set->configChanged &= ~(1 << 1);
    __HAL_TIM_SET_PRESCALER(&htim4, set->psc);
    TIM4->ARR = set->arr;

    settings.telegramSize = CLAMP(settings.samplingFreq / 10, 1, 196);
  }

}

uint32_t fake_power(uint32_t m, uint32_t n){
  if(n <= 0) return 0;
  uint32_t result = 10;
  for(uint8_t i = 0; i < n-1; i++){
    result *= 10;
  }
  return result;
}

// calculate timer values
void get_timer_value(struct settings_t *set){
    uint32_t num = 168000000/set->samplingFreq;
    uint8_t n = 0;
    while (num > 65534) {
      n += 1;
      num = num/10;
    };
    set->psc = fake_power(10, n);
    if(set->psc > 0) set->psc -= 1;
    set->arr = num - 1;
    set->configChanged += 2;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  HAL_ADC_Init(&hadc1);

  multimode.Mode = ADC_MODE_INDEPENDENT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_VOPAMP1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_ADC2_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 5;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  HAL_ADC_Init(&hadc2);

  multimode.Mode = ADC_MODE_INDEPENDENT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  HAL_ADCEx_MultiModeConfigChannel(&hadc2, &multimode);

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_VOPAMP2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}

static void MX_CORDIC_Init(void)
{
  hcordic.Instance = CORDIC;
  HAL_CORDIC_Init(&hcordic);
}

static void MX_FMAC_Init(void)
{
  hfmac.Instance = FMAC;
  HAL_FMAC_Init(&hfmac);
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20501E65;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_OPAMP1_Init(void)
{
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  HAL_OPAMP_Init(&hopamp1);
}

static void MX_OPAMP2_Init(void)
{
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = ENABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_2_OR_MINUS_1;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  HAL_OPAMP_Init(&hopamp2);
}

static void MX_OPAMP3_Init(void)
{
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  HAL_OPAMP_Init(&hopamp3);
}

static void MX_RNG_Init(void)
{
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  HAL_RNG_Init(&hrng);
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 16799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 99;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);
}

static void MX_UCPD1_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_UCPD1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_UCPD1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  NVIC_SetPriority(UCPD1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(UCPD1_IRQn);
}

static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);
  HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_DisableFifoMode(&huart1);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 1);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, 0);
    HAL_Delay(250);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
