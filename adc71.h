#include <cstdio>
#include <unistd.h>
#include "miosix.h"

using namespace std;
using namespace miosix;

#define POLLING                            0
#define DEBUG

#ifdef DEBUG
typedef Gpio<GPIOB_BASE,0> adcGPIO;
typedef Gpio<GPIOD_BASE,12> ledGreen;
typedef Gpio<GPIOD_BASE,13> ledOrange;
typedef Gpio<GPIOD_BASE,14> ledRed;
typedef Gpio<GPIOD_BASE,15> ledBlue;
#endif

/* ADC definitions */
/* CR1 register Mask */
#define CR1_CLEAR_MASK                     ((uint32_t)0xFCFFFEFF)
/* CR2 register Mask */
#define CR2_CLEAR_MASK                     ((uint32_t)0xC0FFF7FD)
/* ADC L Mask */
#define SQR1_L_RESET                       ((uint32_t)0xFF0FFFFF)
 
/* ADC_resolution */ 
#define ADC_Resolution_12b	               ((uint32_t)0x00000000)
/* ADC_external_trigger_edge_for_regular_channels_conversion */
#define ADC_ExternalTrigConvEdge_None      ((uint32_t)0x00000000)
/* ADC_extrenal_trigger_sources_for_regular_channels_conversion */ 
#define ADC_ExternalTrigConv_T1_CC1        ((uint32_t)0x00000000)
/* ADC_data_align */ 
#define ADC_DataAlign_Right                ((uint32_t)0x00000000)
/* ADC_channels */ 
#define ADC_Channel_8                      ((uint8_t)0x08)
/* ADC_sampling_times */ 
#define ADC_SampleTime_480Cycles	           ((uint8_t)0x07)

/* TIM_Counter_Mode */
#define TIM_CounterMode_Up                 ((uint16_t)0x0000)

/** 
  * @brief  TIM Time Base Init structure definition  
  * @note   This structure is used with all TIMx except for TIM6 and TIM7.  
  */
typedef struct
{
  uint16_t TIM_Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                       This parameter can be a number between 0x0000 and 0xFFFF */
  uint16_t TIM_CounterMode;       /*!< Specifies the counter mode.
                                       This parameter can be a value of @ref TIM_Counter_Mode */
  uint32_t TIM_Period;            /*!< Specifies the period value to be loaded into the active
                                       Auto-Reload Register at the next update event.
                                       This parameter must be a number between 0x0000 and 0xFFFF.  */ 
  uint16_t TIM_ClockDivision;     /*!< Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */
  uint8_t TIM_RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                       reaches zero, an update event is generated and counting restarts
                                       from the RCR value (N).
                                       This means in PWM mode that (N+1) corresponds to:
                                          - the number of PWM periods in edge-aligned mode
                                          - the number of half PWM period in center-aligned mode
                                       This parameter must be a number between 0x00 and 0xFF. */
} TIM_TimeBaseInitTypeDef;

typedef struct
{
  uint32_t ADC_Resolution;                /*!< Configures the ADC resolution dual mode. 
                                               This parameter can be a value of @ref ADC_resolution */                                   
  FunctionalState ADC_ScanConvMode;       /*!< Specifies whether the conversion 
                                               is performed in Scan (multichannels) 
                                               or Single (one channel) mode.
                                               This parameter can be set to ENABLE or DISABLE */ 
  FunctionalState ADC_ContinuousConvMode; /*!< Specifies whether the conversion 
                                               is performed in Continuous or Single mode.
                                               This parameter can be set to ENABLE or DISABLE. */
  uint32_t ADC_ExternalTrigConvEdge;      /*!< Select the external trigger edge and
                                               enable the trigger of a regular group. 
                                               This parameter can be a value of 
                                               @ref ADC_external_trigger_edge_for_regular_channels_conversion */
  uint32_t ADC_ExternalTrigConv;          /*!< Select the external event used to trigger 
                                               the start of conversion of a regular group.
                                               This parameter can be a value of 
                                               @ref ADC_extrenal_trigger_sources_for_regular_channels_conversion */
  uint32_t ADC_DataAlign;                 /*!< Specifies whether the ADC data  alignment
                                               is left or right. This parameter can be 
                                               a value of @ref ADC_data_align */
  uint8_t  ADC_NbrOfConversion;           /*!< Specifies the number of ADC conversions
                                               that will be done using the sequencer for
                                               regular channel group.
                                               This parameter must range from 1 to 16. */
}ADC_InitTypeDef;

unsigned int adcval;