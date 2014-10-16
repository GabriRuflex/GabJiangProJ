/*!
* \brief  Disables the ADC's interrupts.
* \param  void
* \return void: None
*/
void DisableADC()
{
  NVIC_DisableIRQ(ADC_IRQn); //! Disable the ADC Interrupts
}

/*!
* \brief  Enables the ADC's interrupts.
* \param  void
* \return void: None
*/
void EnableADC()
{
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn,  15); //! Set the lowest priority to ADC Interrupts
  NVIC_EnableIRQ(ADC_IRQn); //! Enable the ADC Interrupts
}

/*!
* \brief  Handles the calling of the ADC's interrupt handler.
* \param  void
* \return void: None
*/
void __attribute__((naked)) ADC_IRQHandler()
{
  saveContext();
  asm volatile("bl _Z17ADCIRQHandlerImplv"); //! Before restore context, calls the interrupt's handler
  restoreContext();
}

/*!
* \brief  Ridefines the ADC handler to handle the ADC's returned values.
* \param  void
* \return void: None
*/
void __attribute__((used)) ADCIRQHandlerImpl()
{
  //! Check the status of the EOC ADC interrupt
  if (((((ADC1->SR & ADC_SR_EOC) != (uint32_t)RESET) &&
    (ADC1->CR1 & ADC_CR1_EOCIE)) && (values < NVAL)) && waiting != 0)
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

      if(waiting->IRQgetPriority() >
        Thread::IRQgetCurrentThread()->IRQgetPriority())
      {
        Scheduler::IRQfindNextThread(); //! Finds next thread to execute
      }

      waiting = 0; //! There is no more waiting thread, main was woken up
    }
  }
}
