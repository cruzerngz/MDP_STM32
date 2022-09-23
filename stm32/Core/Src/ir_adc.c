// static uint8_t watchdog_val = 0;
// static uint8_t coll_det_flag = 1;

// static uint32_t ticks = 0;

// static uint16_t raw;

#include "ir_adc.h"
#include "math.h"

static ADC_HandleTypeDef *HADC;

static uint16_t adc_buf[ADC_BUF_LEN];
static uint8_t buffer_ADC[16];

static __IO uint16_t uhADCxConvertedValue;
static uint8_t ubAnalogWatchdogStatus = RESET;

static int i, sum, avg;

uint16_t IR_ADC_AVERAGE_READOUT = 0;

static uint32_t AGGREGATE_SUM = 0;
static uint32_t ADC_RAW_READOUT = 0;

void _accumulate_sum(void);

// Bring relevant adc control variables into scope.
void ir_adc_init(ADC_HandleTypeDef *hadc)
{
    HADC = hadc;
    HAL_ADC_Start_DMA(HADC, (uint32_t *)adc_buf, ADC_BUF_LEN);
    AGGREGATE_SUM = HAL_ADC_GetValue(HADC) << IR_ADC_SUM_SIZE_BITS; // set global once on init
}

// Accumulates + averages the ADC value if the value is not out of jitter range
// Jitter limits derived from observations of the IR sensor
void _accumulate_sum(void)
{
    uint32_t fraction = AGGREGATE_SUM >> IR_ADC_SUM_SIZE_BITS;

    // ignore if out of range - periodic jitter from signal
    if (abs(fraction - ADC_RAW_READOUT) > IR_ADC_REJECT_THRESHOLD)
    {
        return;
    }
    else // remove 1/8(or 1/2 ** sum_size_bits), add new read value
    {
        AGGREGATE_SUM -= fraction;
        AGGREGATE_SUM += ADC_RAW_READOUT;
        IR_ADC_AVERAGE_READOUT = AGGREGATE_SUM >> IR_ADC_SUM_SIZE_BITS;
    }
}

void ir_adc_poll(void)
{

    //	OLED_Display_On();

    //	  uint8_t buffer_ADC[16];
    // Get ADC value
    //	  i = 10;
    //	  sum = 0;
    //	  while (i > 0) {
    //		  raw = HAL_ADC_GetValue(&hadc1);
    //		  sum += raw;
    //		  i--;
    //		  osDelay(1);
    //	  }
    //
    //	  avg = sum / 10;

    ADC_RAW_READOUT = HAL_ADC_GetValue(HADC);
    _accumulate_sum();
    // taskENTER_CRITICAL();
    // sprintf(buffer_ADC, "%d\r\n", raw);
    // taskEXIT_CRITICAL();

    // HAL_UART_Transmit(&huart3, buffer_ADC, strlen(buffer_ADC), HAL_MAX_DELAY);

    //	  __disable_irq();
    //	  watchdog_val = ubAnalogWatchdogStatus;
    //	  __enable_irq();
    //
    //	  if (watchdog_val == SET)
    //      {
    //		  if (coll_det_flag) {
    //	    	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    //	    	  motor_stop();
    //
    ////	    	  taskENTER_CRITICAL();
    ////			  sprintf(buffer_ADC, "##stop at %hu\r\n", avg);
    //	    	  strcpy(buffer_ADC, "Found\r\n");
    ////			  taskEXIT_CRITICAL();
    //
    //			  HAL_UART_Transmit(&huart3, buffer_ADC, strlen(buffer_ADC), HAL_MAX_DELAY);
    //
    ////	    	  OLED_ShowString(0, 10, out_buf);
    //
    //	    	  // disable coll det if exceed threshold
    //	    	  coll_det_flag = 0;
    //		  }
    //
    //    	  __disable_irq();
    //    	  watchdog_val = RESET;
    //    	  ubAnalogWatchdogStatus = RESET;
    //    	  __enable_irq();
    //
    //    	  strcpy(buffer_ADC, "Idle\r\n");
    //		  HAL_UART_Transmit(&huart3, buffer_ADC, strlen(buffer_ADC), HAL_MAX_DELAY);
    //
    //    	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
    //      }
    //      else
    //      {
    ////    	  taskENTER_CRITICAL();
    ////    	  sprintf(buffer_ADC, "IR: %d\r\n", avg);
    //    	  // strcpy(buffer_ADC, "Sensing\r\n");
    ////    	  taskEXIT_CRITICAL();
    //
    //		  //HAL_UART_Transmit(&huart3, buffer_ADC, strlen(buffer_ADC), HAL_MAX_DELAY);
    //
    //    //	  OLED_ShowString(0, 0, buffer_ADC);
    //    	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    //
    //    	  // enable coll det if enters threshold
    //    	  coll_det_flag = 1;
    //      }

    //	  OLED_Refresh_Gram();
}
