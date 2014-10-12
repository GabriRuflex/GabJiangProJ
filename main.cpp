#include "libs.h"

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
