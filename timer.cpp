void InitializeTimer()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //! Enables the Low Speed APB (APB1) peripheral clock.

  //! Configures the timer
  uint16_t tmpcr1 = 0;
  tmpcr1 = TIM2->CR1;

  tmpcr1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS)); //! Selects the Counter Mode: sets CMS to 00 (Edge-aligned mode) and DIR to 0 (Upcounter)
  tmpcr1 &=  (uint16_t)(~TIM_CR1_CKD); //! Sets the Clock Division: sets CKD to 00 (timer clock = sampling clock used by the digital filters)

  TIM2->CR1 = tmpcr1;

  TIM2->ARR = 2 - 1 ; //! Sets the Autoreload value (10 kHz / 2 = 5 kHz)
  TIM2->PSC = 8400 - 1; //! Sets the Prescaler value (84000 kHz / 8400 = 10 kHz)

  TIM2->EGR = TIM_EGR_UG; //! Generates an update event to reload the Prescaler and the repetition counter(only for TIM1 and TIM8) value immediatly

  TIM2->CR2 &= (uint16_t)~TIM_CR2_MMS; //! Resets the MMS Bits
  TIM2->CR2 |=  TIM_CR2_MMS_1; //! Selects the TRGO source

  TIM2->CR1 |= TIM_CR1_CEN; //! Enables the Counter
}

