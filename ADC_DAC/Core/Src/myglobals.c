/*
 * myglobals.c
 *
 *  Created on: Jun 4, 2025
 *      Author: choua
 */
#include <stdint.h>

#include "myglobals.h"

/* signals */
uint8_t start_adc_process = 0;
uint8_t start_plot = 0;

/* variables */
uint8_t trigger_level = 128;
uint8_t trigger_type = RISING_EDGE;
uint8_t *plot_buff = 0; // Pointer to the plot buffer to be used in the plot thread
uint16_t trigger_index = 0; // Index of the center in the plot buffer