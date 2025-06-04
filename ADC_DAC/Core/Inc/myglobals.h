/*
 * myglobals.h
 *
 *  Created on: Jun 4, 2025
 *      Author: choua
 */

#ifndef INC_MYGLOBALS_H_
#define INC_MYGLOBALS_H_

#include <stdint.h>

#define GLOBAL_CLK_RATE (uint32_t) 216000000

#define ADC_SINGLE_BUFF_LEN 2400
#define ADC_NUM_INS 3

#define PLOT_DATA_LEN 400

/* signals */
#define TOP_HALF 1
#define BOTTOM_HALF 2
extern uint8_t start_adc_process;
extern uint8_t start_plot;

/* variables */
extern uint8_t trigger_level;
#define RISING_EDGE 0
#define FALLING_EDGE 1
extern uint8_t trigger_type;
extern uint8_t *plot_buff; // Pointer to the plot buffer to be used in the plot thread
extern uint16_t trigger_index; // Index of the center in the plot buffer
extern uint16_t zero_index;

#endif /* INC_MYGLOBALS_H_ */
