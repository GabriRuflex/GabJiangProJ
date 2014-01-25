#include "adc71.h"

void InitializeTimer()
{
  /* Enable the Low Speed APB (APB1) peripheral clock. */
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  /* Configure the timer */
  uint16_t tmpcr1 = 0;
  tmpcr1 = TIM4->CR1;

  /* Select the Counter Mode */
  tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS)); //Set CMS to 00 (Edge-aligned mode) and DIR to 0 (Upcounter)
  //tmpcr1 |= (uint32_t)0x0000;

  /* Set the clock division */
  tmpcr1 &=  (uint16_t)(~TIM_CR1_CKD); //Set CKD to 00 (timer clock = sampling clock used by the digital filters)
  //tmpcr1 |= (uint32_t)0x0000;

  TIM4->CR1 = tmpcr1;

  /* Set the Autoreload value */
  TIM4->ARR = 10 - 1 ; // 1 msec

  /* Set the Prescaler value */
  TIM4->PSC = 8400 - 1; // 84000 kHz / 8400 = 10kHz

  /* Generate an update event to reload the Prescaler
     and the repetition counter(only for TIM1 and TIM8) value immediatly */
  TIM4->EGR = TIM_EGR_UG;

  /* Enable the Counter */
  TIM4->CR1 |= TIM_CR1_CEN;
  /* Enable the Interrupt sources */
  TIM4->DIER |= TIM_DIER_UIE;

  /* Enable TIM4 interrupt */
  NVIC_SetPriority(TIM4_IRQn, (1<<__NVIC_PRIO_BITS) - 1); //Set the lowest priority to TIM4 Interrupts
  NVIC_ClearPendingIRQ(TIM4_IRQn);
  NVIC_EnableIRQ(TIM4_IRQn);
}

void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  /* Initialize the ADC_Mode member */
  ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b;

  /* initialize the ADC_ScanConvMode member */
  ADC_InitStruct->ADC_ScanConvMode = DISABLE;

  /* Initialize the ADC_ContinuousConvMode member */
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;

  /* Initialize the ADC_ExternalTrigConvEdge member */
  ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;

  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

  /* Initialize the ADC_DataAlign member */
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;

  /* Initialize the ADC_NbrOfConversion member */
  ADC_InitStruct->ADC_NbrOfConversion = 1;
}

/**
  * @brief  Deinitializes all ADCs peripherals registers to their default reset
  *         values.
  * @param  None
  * @retval None
  */
void ADC_DeInit(void)
{
  /* Enable all ADCs reset state */
  RCC->APB2RSTR |= RCC_APB2ENR_ADC1EN;

  /* Release all ADCs from reset state */
  RCC->APB2RSTR &= ~RCC_APB2ENR_ADC1EN;
}

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_InitStruct: pointer to an ADC_InitTypeDef structure that contains
  *         the configuration information for the specified ADC peripheral.
  * @retval None
  */
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;

  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;

  /* Clear RES and SCAN bits */
  tmpreg1 &= CR1_CLEAR_MASK;

  /* Configure ADCx: scan conversion mode and resolution */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  /* Set RES bit according to ADC_Resolution value */
  tmpreg1 |= (uint32_t)(((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8) | \
                                   ADC_InitStruct->ADC_Resolution);
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;
  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;

  /* Clear CONT, ALIGN, EXTEN and EXTSEL bits */
  tmpreg1 &= CR2_CLEAR_MASK;

  /* Configure ADCx: external trigger event and edge, data alignment and
     continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTEN bits according to ADC_ExternalTrigConvEdge value */
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | \
                        ADC_InitStruct->ADC_ExternalTrigConv |
                        ADC_InitStruct->ADC_ExternalTrigConvEdge | \
                        ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));

  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;
  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;

  /* Clear L bits */
  tmpreg1 &= SQR1_L_RESET;

  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfConversion value */
  tmpreg2 |= (uint8_t)(ADC_InitStruct->ADC_NbrOfConversion - (uint8_t)1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);

  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;
}

/**
  * @brief  Configures for the selected ADC regular channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure.
  * @param  Rank: The rank in the regular group sequencer.
  *          This parameter must be between 1 to 16.
  * @param  ADC_SampleTime: The sample time value to be set for the selected channel.
  * @retval None
  */
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
  /* with some limitations: channel < 10, rank < 8 */
  uint32_t tmpreg1 = 0, tmpreg2 = 0;

  /* Get the old register value */
  tmpreg1 = ADCx->SMPR2;

  /* Calculate the mask to clear */
  tmpreg2 = ADC_SMPR2_SMP0 << (3 * ADC_Channel);

  /* Clear the old sample time */
  tmpreg1 &= ~tmpreg2;

  /* Calculate the mask to set */
  tmpreg2 = (uint32_t)ADC_SampleTime << (3 * ADC_Channel);

  /* Set the new sample time */
  tmpreg1 |= tmpreg2;

  /* Store the new register value */
  ADCx->SMPR2 = tmpreg1;

  /* Get the old register value */
  tmpreg1 = ADCx->SQR3;

  /* Calculate the mask to clear */
  tmpreg2 = ADC_SQR3_SQ1 << (5 * (Rank - 1));

  /* Clear the old SQx bits for the selected rank */
  tmpreg1 &= ~tmpreg2;

  /* Calculate the mask to set */
  tmpreg2 = (uint32_t)ADC_Channel << (5 * (Rank - 1));

  /* Set the SQx bits for the selected rank */
  tmpreg1 |= tmpreg2;

  /* Store the new register value */
  ADCx->SQR3 = tmpreg1;
}

uint16_t ADC_SingleAcquisition()
{
  uint16_t res;

  /* ADCx regular channel 8 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_480Cycles);

  /* Enable ADC1 conversion for regular group */
  ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;

  /* Wait until ADCx end of conversion */
  while((ADC1->SR & ADC_SR_EOC) == 0);

  /* Get ADCx conversion value */
  res = (uint16_t)ADC1->DR;

  return res;
}

void checkEnvrionment()
{
  printf("Max: %u Min: %u", max, min);
}

void handleADC()
{
  /* Run acquisition */
  adcval = ADC_SingleAcquisition();
  if (values == 1000) {
    checkEnvrionment();
    values = 0;
    min = max = adcval;
  }

  buffer[values] = adcval;
  max = (adcval > max) ? adcval : max;
  min = (adcval < min) ? adcval : min;
  printf("ok");
  values++;
  printf("%u",max);
#ifdef DEBUG
  if (adcval < 300)
  {
    ledGreen::high();
    ledOrange::low();
    ledRed::low();
    ledBlue::low();
  }
  else if (adcval < 400)
  {
    ledGreen::low();
    ledOrange::high();
    ledRed::low();
    ledBlue::low();
  }
  else if (adcval < 500)
  {
    ledGreen::low();
    ledOrange::low();
    ledRed::high();
    ledBlue::low();
  }
  else
  {
    ledGreen::low();
    ledOrange::low();
    ledRed::low();
    ledBlue::high();
  }
#endif
}

void InitializeBoard()
{
  //unsigned int adcval;
  ADC_InitTypeDef ADC_InitStructure;

  /* Inizialization of ADC's GPIO */
  adcGPIO::mode(Mode::INPUT_ANALOG); ///Floating Input       (MODE=11 TYPE=0 PUP=00)
  adcGPIO::speed(Speed::_50MHz);

  /* ADC1 Periph clock enable */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* Reset ADC to default values */
  ADC_DeInit();

  /* ADC1 Configuration */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;

#ifdef DEBUG
  /* Inizialization of LED's GPIOs*/
  ledGreen::mode(Mode::OUTPUT); ///Push Pull  Output    (MODE=01 TYPE=0 PUP=00)
  ledOrange::mode(Mode::OUTPUT);
  ledRed::mode(Mode::OUTPUT);
  ledBlue::mode(Mode::OUTPUT);
#endif
}

int main()
{
  InitializeBoard();
  if (POLLING)
  {
    while (1) { handleADC(); }
  }
  else
  {
    InitializeTimer();
    while(1);
  }

  return 0;
}

void TIM4_IRQHandler()
{
  uint16_t itstatus = 0x0, itenable = 0x0;

  /* Check interrupt status */
  itstatus = TIM4->SR & TIM_SR_UIF;
  /* Check interrupt enabled */
  itenable = TIM4->DIER & TIM_DIER_UIE;

  if ((itstatus != (uint16_t)RESET) && (itenable != (uint16_t)RESET))
  {
    /* Clear the IT pending Bit */
    TIM4->SR = (uint16_t)~TIM_SR_UIF;
    handleADC();
  }
}