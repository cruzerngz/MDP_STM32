#include "stm32f4xx_hal.h"

#ifndef INC_IR_ADC_H_
#define INC_IR_ADC_H_

#define ADC_BUF_LEN 4096

#define IR_ADC_REJECT_THRESHOLD 4000 // in adc value steps
#define IR_ADC_SUM_SIZE_BITS 3       // 2 ** this = number of values accumulated (8)

#define IR_ADC_POLLING_RATE_TICKS 10
#define IR_ADC_POLLING_RATE_HZ 100

// For other modules to access
// Read this variable atomically
extern uint16_t IR_ADC_AVERAGE_READOUT;

void ir_adc_init(ADC_HandleTypeDef *hadc);
void ir_adc_poll(void);
void ir_adc_reset(void);

#endif