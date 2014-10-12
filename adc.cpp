void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  //! Initializes the ADC_StructInit members
  ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b; //! Sets the ADC resolution to 12bit
  ADC_InitStruct->ADC_ScanConvMode = DISABLE; //! Disables the Scan Conversion mode
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE; //! Disables the Continuous Conversion mode
  ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising; //! Sets the external trigger's conversion edge to rising
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //! Sets the Timer 2 trigger as the external trigger for conversion
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right; //! Alignes the ADC's data to right
  ADC_InitStruct->ADC_NbrOfConversion = 1; //! Sets the number of conversion to 1
  ADC_InitStruct->ADC_Channel = 8; //! Sets the ADC's channel to 8 (ADC1)
  ADC_InitStruct->ADC_Rank = 1; //! Sets the ADC's rank to 1
  ADC_InitStruct->ADC_SampleTime = 7; //! Sets the sample time to 480Cycles
}

/*!
* \brief  Deinitializes all ADCs peripherals registers to their default reset values.
* \param  void: None
* \return void: None
*/
void ADC_DeInit(void)
{  
  RCC->APB2RSTR |= RCC_APB2ENR_ADC1EN; //! Enables all ADCs reset state
  RCC->APB2RSTR &= ~RCC_APB2ENR_ADC1EN; //! Releases all ADCs from reset state
}
  
/*!
* \brief  Initializes the ADCx peripheral according to the specified parameters in the ADC_InitStruct.
* \param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* \param  ADC_InitStruct: pointer to an ADC_InitTypeDef structure that contains the configuration information for the specified ADC peripheral.
* \return void: None
*/
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg1 = 0, tmpreg3 = 0;
  uint8_t tmpreg2 = 0;

  /*!---------------------------- ADCx CR1 Configuration -----------------*/
  tmpreg1 = ADCx->CR1; //! Gets the ADCx CR1 value

  //! Configures ADCx: scan conversion mode and resolution
  tmpreg1 &= ~ADC_CR1_SCAN; //! Clears SCAN bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8; //! Sets SCAN bit according to ADC_ScanConvMode value

  tmpreg1 &= ~ADC_CR1_RES; //! Clears RES bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_Resolution << 24; //! Sets RES bit according to ADC_Resolution value

  //! Writes to ADCx CR1
  ADCx->CR1 = tmpreg1;
  /*!---------------------------- ADCx CR2 Configuration -----------------*/
  tmpreg1 = ADCx->CR2; //! Gets the ADCx CR2 value

  //! Configures ADCx: external trigger event and edge, data alignment and continuous conversion mode 
  tmpreg1 &= ~ADC_CR2_EXTEN; //! Clears EXTEN bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ExternalTrigConvEdge << 28; //! Sets EXTEN bits according to ADC_ExternalTrigConvEdge value

  tmpreg1 &= ~ADC_CR2_EXTSEL; //! Clears EXTSEL bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ExternalTrigConv << 24; //! Sets EXTSEL bits according to ADC_ExternalTrigConv value

  tmpreg1 &= ~ADC_CR2_ALIGN; //! Clears ALIGN bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_DataAlign << 11; //! Sets ALIGN bit according to ADC_DataAlign value

  tmpreg1 &= ~ADC_CR2_CONT; //! Clears CONT bits
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1; //! Sets CONT bit according to ADC_ContinuousConvMode value

  ADCx->CR2 = tmpreg1; //! Writes to ADCx CR2
  /*!---------------------------- ADCx SQR1 Configuration -----------------*/
  tmpreg1 = ADCx->SQR1; //! Gets the ADCx SQR1 value

  tmpreg1 &= ADC_SQR1_L; //! Clears L bits

  //! Configures ADCx: regular channel sequence length
  //! Sets L bits according to ADC_NbrOfConversion value
  tmpreg2 |= (uint8_t)(ADC_InitStruct->ADC_NbrOfConversion - (uint8_t)1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);

  ADCx->SQR1 = tmpreg1; //! Writes to ADCx SQR1
  /*!---------------------------- ADCx SMPR2 Configuration -----------------*/
  tmpreg1 = 0, tmpreg3 = 0;

  tmpreg1 = ADCx->SMPR2; //! Gets the old register value

  tmpreg3 = ADC_SMPR2_SMP0 << (3 * ADC_InitStruct->ADC_Channel); //! Calculates the mask to clear

  tmpreg1 &= ~tmpreg2; //! Clears the old sample time

  tmpreg3 = (uint32_t)ADC_InitStruct->ADC_SampleTime << (3 * ADC_InitStruct->ADC_Channel); //! Calculates the mask to set

  tmpreg1 |= tmpreg3; //! Sets the new sample time

  ADCx->SMPR2 = tmpreg1; //! Stores the new register value
  /*!---------------------------- ADCx SQ3 Configuration -----------------*/
  tmpreg1 = ADCx->SQR3; //! Gets the old register value

  tmpreg3 = ADC_SQR3_SQ1 << (5 * (ADC_InitStruct->ADC_Rank - 1)); //! Calculates the mask to clear

  tmpreg1 &= ~tmpreg3; //! Clears the old SQx bits for the selected rank

  tmpreg2 = (uint32_t)ADC_InitStruct->ADC_Channel << (5 * (ADC_InitStruct->ADC_Rank - 1)); //! Calculates the mask to set

  tmpreg1 |= tmpreg2; //! Sets the SQx bits for the selected rank

  ADCx->SQR3 = tmpreg1; //! Stores the new register value
}

void InitializeADC()
{
  ADC_InitTypeDef ADC_InitStructure;

  //! Inizialization of ADC's GPIO
  adcGPIO::mode(Mode::INPUT_ANALOG); //! Floating Input (MODE=11 TYPE=0 PUP=00)
  adcGPIO::speed(Speed::_100MHz);

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //! ADC1 Periph clock enable

  ADC_DeInit(); //! Reset ADC to default values

  ADC_StructInit(&ADC_InitStructure); //! ADC1 Struct initialization
  ADC_Init(ADC1, &ADC_InitStructure); //! ADC1 Configuration

  ADC1->CR1 |= (uint32_t)ADC_CR1_EOCIE; //! Enable Interrupts

  ADC1->CR2 |= (uint32_t)ADC_CR2_ADON; //! ADC Power ON

#ifdef DEBUG
  //! Inizialization of LED's GPIOs
  ledGreen::mode(Mode::OUTPUT); //! Push Pull Output (MODE=01 TYPE=0 PUP=00)
  ledOrange::mode(Mode::OUTPUT);
  ledRed::mode(Mode::OUTPUT);
  ledBlue::mode(Mode::OUTPUT);

  ledGreen::speed(Speed::_100MHz);
  ledOrange::speed(Speed::_100MHz);
  ledRed::speed(Speed::_100MHz);
  ledBlue::speed(Speed::_100MHz);
#endif
}

void DisableADC()
{
  NVIC_DisableIRQ(ADC_IRQn); //! Disable the ADC Interrupts
}

void EnableADC()
{
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn,  15); //! Set the lowest priority to ADC Interrupts
  NVIC_EnableIRQ(ADC_IRQn); //! Enable the ADC Interrupts
}
/**************************************************************************************/
void handleADC()
{
  adcval = (uint16_t)ADC1->DR; //! Run acquisition
  buffer[values] = adcval; //! Save the value to the buffer
  values++; //! Increases the number of values

/*!
  Enables board's leds blinking to show values
  led green: between 0 and 299
  led orange: between 300 and 399
  led red: between 400 and 499
  led blue: the remaining values
*/
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
/**************************************************************************************/
void __attribute__((naked)) ADC_IRQHandler()
{
  saveContext();
  asm volatile("bl _Z17ADCIRQHandlerImplv"); //! Before restore context, calls the interrupt's handler
  restoreContext();
}

void __attribute__((used)) ADCIRQHandlerImpl()
{
  //! Check the status of the EOC ADC interrupt
  if (((((ADC1->SR & ADC_SR_EOC) != (uint32_t)RESET) && (ADC1->CR1 & ADC_CR1_EOCIE)) && (values < NVAL)) && waiting != 0)
  {
    ADC1->SR &= ~(uint32_t)ADC_SR_EOC; //! Clears the selected ADC interrupt pending bits

    handleADC(); //! Handles the end of ADC's conversion

    if (NVAL == values) //! If the buffer is completely full, disables ADC interrupts and wakeup the main thread (which has higher priority)
    {
      DisableADC(); //! Disables ADC interrupts

      if(waiting == 0) {
        return; //! If there isn't waiting thread, terminates execution
      }

      waiting->IRQwakeup(); //! Wakes up waiting thread

      if(waiting->IRQgetPriority() > Thread::IRQgetCurrentThread()->IRQgetPriority()) {
		    Scheduler::IRQfindNextThread(); //! Finds next thread to execute
      }

      waiting = 0; //! There is no more waiting thread, main was woken up
    }
  }
}

void waitForADC()
{
    FastInterruptDisableLock dLock; //! Disables interrupts
    waiting = Thread::IRQgetCurrentThread(); //! Sets current thread (main) as waiting
    
    EnableADC(); //! Enables ADC's interrupts

    while(waiting)
    {
        Thread::IRQwait(); //! Puts the current thread in wait status (in a piece of code where interrupts are disabled)
        FastInterruptEnableLock eLock(dLock); //! Temporarily re enables interrupts (in a scope where they are disabled)
        Thread::yield(); //!  Suggests the kernel to pause the current thread, and run another one. 
    }
}
