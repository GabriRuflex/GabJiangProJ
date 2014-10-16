#include "libs.h"

/*!
* \brief  Waits for the filling of buffer's values
* \param  void
* \return void: None
*/
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

int main()
{
  int i, sum;
  float avg;

  adcval = values = 0;

  InitializeADC(); //! Initializes the board
  InitializeTimer(); //! Initializes the timer

  while(1) {
    waitForADC(); //! The main thread goes to sleep mode until ADC fills all the buffer with its values

    sum = 0;
    for (i = 0; i < NVAL; i++) {
      sum += buffer[i];
    }
    avg = (float) sum / NVAL; //! Calculates the average of values stored in the buffer
    
    printf("Ultimo valore: %d Num.valori: %d Media: %.2f\n", adcval, values, avg); //! Shows to the user the last ADC's value, the number of values stored in the buffer and the average of them

    values = 0; //! Sets the values count to 0 in order to restart the conversion
  }

  return 0;
}
