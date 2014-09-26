#include "adc71.h"
/**************************************************************************************/
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  /* Initialize the ADC_StructInit members */
  ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStruct->ADC_ScanConvMode = DISABLE;
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;
  ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct->ADC_NbrOfConversion = 1;
  ADC_InitStruct->ADC_Channel = 8;
  ADC_InitStruct->ADC_Rank = 1;
  ADC_InitStruct->ADC_SampleTime = 7; //480Cycles
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
  uint32_t tmpreg1 = 0, tmpreg3 = 0;
  uint8_t tmpreg2 = 0;

  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;

  /* Configure ADCx: scan conversion mode and resolution */
  tmpreg1 &= ~ADC_CR1_SCAN; // Clear SCAN bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8; // Set SCAN bit according to ADC_ScanConvMode value

  tmpreg1 &= ~ADC_CR1_RES; // Clear RES bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_Resolution << 24; // Set RES bit according to ADC_Resolution value

  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;
  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;

  /* Configure ADCx: external trigger event and edge, data alignment and
     continuous conversion mode */
  tmpreg1 &= ~ADC_CR2_EXTEN; // Clear EXTEN bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ExternalTrigConvEdge << 28; // Set EXTEN bits according to ADC_ExternalTrigConvEdge value

  tmpreg1 &= ~ADC_CR2_EXTSEL; // Clear EXTSEL bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ExternalTrigConv << 24; // Set EXTSEL bits according to ADC_ExternalTrigConv value

  tmpreg1 &= ~ADC_CR2_ALIGN; // Clear ALIGN bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_DataAlign << 11; // Set ALIGN bit according to ADC_DataAlign value

  tmpreg1 &= ~ADC_CR2_CONT; // Clear CONT bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1; // Set CONT bit according to ADC_ContinuousConvMode value

  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;
  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;

  /* Clear L bits */
  tmpreg1 &= ADC_SQR1_L;

  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfConversion value */
  tmpreg2 |= (uint8_t)(ADC_InitStruct->ADC_NbrOfConversion - (uint8_t)1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);

  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;
  /*---------------------------- ADCx SMPR2 Configuration -----------------*/
  /* Channel < 10, rank < 8 */
  tmpreg1 = 0, tmpreg3 = 0;

  /* Get the old register value */
  tmpreg1 = ADCx->SMPR2;

  /* Calculate the mask to clear */
  tmpreg3 = ADC_SMPR2_SMP0 << (3 * ADC_InitStruct->ADC_Channel);

  /* Clear the old sample time */
  tmpreg1 &= ~tmpreg2;

  /* Calculate the mask to set */
  tmpreg3 = (uint32_t)ADC_InitStruct->ADC_SampleTime << (3 * ADC_InitStruct->ADC_Channel);

  /* Set the new sample time */
  tmpreg1 |= tmpreg3;

  /* Store the new register value */
  ADCx->SMPR2 = tmpreg1;
  /*---------------------------- ADCx SQ3 Configuration -----------------*/
  /* Get the old register value */
  tmpreg1 = ADCx->SQR3;

  /* Calculate the mask to clear */
  tmpreg3 = ADC_SQR3_SQ1 << (5 * (ADC_InitStruct->ADC_Rank - 1));

  /* Clear the old SQx bits for the selected rank */
  tmpreg1 &= ~tmpreg3;

  /* Calculate the mask to set */
  tmpreg2 = (uint32_t)ADC_InitStruct->ADC_Channel << (5 * (ADC_InitStruct->ADC_Rank - 1));

  /* Set the SQx bits for the selected rank */
  tmpreg1 |= tmpreg2;

  /* Store the new register value */
  ADCx->SQR3 = tmpreg1;
}
/**************************************************************************************/
void InitializeBoard()
{
  //unsigned int adcval;
  ADC_InitTypeDef ADC_InitStructure;

  /* Inizialization of ADC's GPIO */
  adcGPIO::mode(Mode::INPUT_ANALOG); ///Floating Input       (MODE=11 TYPE=0 PUP=00)
  adcGPIO::speed(Speed::_100MHz);

  /* ADC1 Periph clock enable */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* Reset ADC to default values */
  ADC_DeInit();

  /* ADC1 Struct initialization */
  ADC_StructInit(&ADC_InitStructure);

  /* ADC1 Configuration */
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Enable Interrupts */
  ADC1->CR1 |= (uint32_t)ADC_CR1_EOCIE;

  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  SCB->AIRCR = (uint32_t)0x05FA0000 | (uint32_t)0x300; //4 bits for preemp priority 0 bit for sub priority

  NVIC_SetPriority(ADC_IRQn,  15); //Set the lowest priority to ADC1 Interrupts
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_EnableIRQ(ADC_IRQn);

  /* ADC Power ON*/
  ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;

#ifdef DEBUG
  /* Inizialization of LED's GPIOs*/
  ledGreen::mode(Mode::OUTPUT); ///Push Pull  Output    (MODE=01 TYPE=0 PUP=00)
  ledOrange::mode(Mode::OUTPUT);
  ledRed::mode(Mode::OUTPUT);
  ledBlue::mode(Mode::OUTPUT);

  ledGreen::speed(Speed::_100MHz);
  ledOrange::speed(Speed::_100MHz);
  ledRed::speed(Speed::_100MHz);
  ledBlue::speed(Speed::_100MHz);
#endif
}
/**************************************************************************************/
void InitializeTimer()
{
  /* Enable the Low Speed APB (APB1) peripheral clock. */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  /* Configure the timer */
  uint16_t tmpcr1 = 0;
  tmpcr1 = TIM2->CR1;

  /* Select the Counter Mode */
  tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS)); //Set CMS to 00 (Edge-aligned mode) and DIR to 0 (Upcounter)

  /* Set the clock division */
  tmpcr1 &=  (uint16_t)(~TIM_CR1_CKD); //Set CKD to 00 (timer clock = sampling clock used by the digital filters)

  TIM2->CR1 = tmpcr1;

  /* Set the Autoreload value */
  TIM2->ARR = 2 - 1 ; // 10 kHz / 2 = 5 kHz
  /* Set the Prescaler value */
  TIM2->PSC = 8400 - 1; // 84000 kHz / 8400 = 10 kHz

  /* Generate an update event to reload the Prescaler
     and the repetition counter(only for TIM1 and TIM8) value immediatly */
  TIM2->EGR = TIM_EGR_UG;

  /* Reset the MMS Bits */
  TIM2->CR2 &= (uint16_t)~TIM_CR2_MMS;
  /* Select the TRGO source */
  TIM2->CR2 |=  TIM_CR2_MMS_1;

  /* Enable the Counter */
  TIM2->CR1 |= TIM_CR1_CEN;
}
/**************************************************************************************/
void handleADC()
{
  /* Run acquisition */
  adcval = (uint16_t)ADC1->DR;
  buffer[values] = adcval;
  values++;

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

void ADC_IRQHandler()
{
  /* Check the status of the EOC ADC interrupt */
  if ((((ADC1->SR & ADC_SR_EOC) != (uint32_t)RESET) && (ADC1->CR1 & ADC_CR1_EOCIE)) && (values < NVAL))
  {
    /* Clear the selected ADC interrupt pending bits */
    ADC1->SR &= ~(uint32_t)ADC_SR_EOC;

    handleADC();
    if (NVAL == values)
    {
      NVIC_DisableIRQ(ADC_IRQn);
    }
  }
}
/**************************************************************************************/

void *threadFunction(void *arg)
{
  printf("Valore letto: %u\n", adcval);
}

int main()
{
	adcval = values = 0;
  pthread_t t;

  InitializeBoard();
  InitializeTimer();
  
  while(1) {
    if (values == NVAL) {
      /*pthread_create(&t,NULL,&threadFunction,NULL);
      pthread_join(t,NULL);*/
      
      int i, sum = 0;
      for (i = 0; i < NVAL; i++) {
        sum += buffer[i];
      }
      
      float avg = (float) sum / NVAL; 
      printf("Valore letto: %.2f\n", avg);

      values = 0;

      NVIC_EnableIRQ(ADC_IRQn);
    }
  }

  return 0;
}