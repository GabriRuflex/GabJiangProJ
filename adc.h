typedef Gpio<GPIOB_BASE,0> adcGPIO;

#ifdef DEBUG
typedef Gpio<GPIOD_BASE,12> ledGreen;
typedef Gpio<GPIOD_BASE,13> ledOrange;
typedef Gpio<GPIOD_BASE,14> ledRed;
typedef Gpio<GPIOD_BASE,15> ledBlue;
#endif

/* ADC_resolution */
#define ADC_Resolution_12b                 ((uint32_t)0x00)
/* ADC_external_trigger_edge_for_regular_channels_conversion */
#define ADC_ExternalTrigConvEdge_Rising      ((uint32_t)0x01)
/* ADC_extrenal_trigger_sources_for_regular_channels_conversion */
#define ADC_ExternalTrigConv_T2_TRGO        ((uint32_t)0x06)
/* ADC_data_align */
#define ADC_DataAlign_Right                ((uint32_t)0x00)
/* ADC_channels */
#define ADC_Channel_8                      ((uint8_t)0x08)
/* ADC_sampling_times */
#define ADC_SampleTime_480Cycles             ((uint8_t)0x07)

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
  uint8_t  ADC_Channel;                   /*!< Specifies the number of ADC conversions
                                               that will be done using the sequencer for
                                               regular channel group.
                                               This parameter must range from 1 to 16. */
  uint8_t  ADC_Rank;                          /*!< Specifies the number of ADC conversions
                                               that will be done using the sequencer for
                                               regular channel group.
                                               This parameter must range from 1 to 16. */
  uint8_t  ADC_SampleTime;                /*!< Specifies the number of ADC conversions
                                               that will be done using the sequencer for
                                               regular channel group.
                                               This parameter must range from 1 to 16. */
}ADC_InitTypeDef;

static Thread *waiting = 0;
volatile static uint16_t adcval;
volatile static int values, buffer[NVAL];
