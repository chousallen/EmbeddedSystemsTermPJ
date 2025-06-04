/*
 * adc_process.h
 *
 *  Created on: Jun 4, 2025
 *      Author: choua
 */

#ifndef INC_ADC_PROCESS_H_
#define INC_ADC_PROCESS_H_

#include "myglobals.h"
#include "stm32f7xx_hal.h"

#include <stdint.h>

#define NUM_SAMPLE_RATES 15
extern uint32_t sample_rate_map[NUM_SAMPLE_RATES];

extern uint8_t adc_buff[ADC_NUM_INS][ADC_SINGLE_BUFF_LEN];

extern TIM_HandleTypeDef *myhtim;

uint32_t get_sample_rate(void);
uint8_t get_sample_rate_index(void);
uint32_t set_sample_rate_index(uint8_t index);
void adc_process(void);

#endif /* INC_ADC_PROCESS_H_ */
