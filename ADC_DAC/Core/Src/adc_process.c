/*
 * adc_process.c
 *
 *  Created on: Jun 4, 2025
 *      Author: choua
 */

#include <adc_process.h>
#include "main.h"
#include "myglobals.h"

#include <stdint.h>

typedef enum {
    STATE_FIND_TRIGGER, 
    STATE_COLLECT_DATA,
} adc_state_t;

adc_state_t adc_process_state = STATE_FIND_TRIGGER;

uint8_t adc_buff[ADC_NUM_INS][ADC_SINGLE_BUFF_LEN];
uint32_t sample_rate_map[NUM_SAMPLE_RATES] =
{
    200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 450000, 900000, 1800000, 3600000, 7200000
};
uint8_t sample_rate_index = NUM_SAMPLE_RATES - 1; // Start with the highest sample rate
uint16_t curr_dest = 0;
uint8_t *adc_buff_ptr[ADC_NUM_INS] = {adc_buff[0], adc_buff[1], adc_buff[2]};

uint8_t my_plot_buff[2][PLOT_DATA_LEN];
// use which screen_plot_buff
uint8_t use_plot_buff = 0; // 0 for first, 1 for second

TIM_HandleTypeDef *myhtim = 0;

uint32_t get_sample_rate(void)
{
    return sample_rate_map[sample_rate_index];
}

uint8_t get_sample_rate_index(void)
{
    return sample_rate_index;
}

uint32_t set_sample_rate_index(uint8_t index)
{
    if (index < NUM_SAMPLE_RATES)
    {
        sample_rate_index = index;
        uint32_t sample_rate = sample_rate_map[sample_rate_index];
  	  __HAL_TIM_SET_PRESCALER(myhtim, GLOBAL_CLK_RATE*3/sample_rate/TIM1_Period);

        return sample_rate;
    }
    return 0; // Invalid rate
}

void adc_process(void)
{
    // set where to start accessing the ADC buffer
    if(start_adc_process == TOP_HALF)
    {
        adc_buff_ptr[0] = adc_buff[0];
        adc_buff_ptr[1] = adc_buff[1];
        adc_buff_ptr[2] = adc_buff[2];
    }
    else if(start_adc_process == BOTTOM_HALF)
    {
        adc_buff_ptr[0] = adc_buff[0] + ADC_SINGLE_BUFF_LEN / 2;
        adc_buff_ptr[1] = adc_buff[1] + ADC_SINGLE_BUFF_LEN / 2;
        adc_buff_ptr[2] = adc_buff[2] + ADC_SINGLE_BUFF_LEN / 2;
    }

    uint8_t last = adc_buff_ptr[0][0], curr;

    switch (adc_process_state)
    {
    case STATE_FIND_TRIGGER:
        // Process the ADC data to find the trigger
        if(trigger_type == RISING_EDGE)
        {
            // Look for a rising edge
            for (uint16_t i = 0; i < ADC_SINGLE_BUFF_LEN/2; i++)
            {
                curr = my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[0][i];
                if (curr_dest >= PLOT_DATA_LEN) // Check if we reached the end of the plot buffer
                {
                    curr_dest = 0; // Reset the destination index
                }
                if (last < trigger_level && curr >= trigger_level)
                {
                    trigger_index = curr_dest - 1; // Store the index of the trigger
                    adc_process_state = STATE_COLLECT_DATA; // Move to the next state
                    break;
                }
                last = curr;

                curr = my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[1][i];
                if (curr_dest >= PLOT_DATA_LEN) // Check if we reached the end of the plot buffer
                {
                    curr_dest = 0; // Reset the destination index
                }
                if (last < trigger_level && curr >= trigger_level)
                {
                    trigger_index = curr_dest - 1; // Store the index of the trigger
                    adc_process_state = STATE_COLLECT_DATA; // Move to the next state
                    break;
                }
                last = curr;

                curr = my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[2][i];
                if (curr_dest >= PLOT_DATA_LEN) // Check if we reached the end of the plot buffer
                {
                    curr_dest = 0; // Reset the destination index
                }
                if (last < trigger_level && curr >= trigger_level)
                {
                    trigger_index = curr_dest - 1; // Store the index of the trigger
                    adc_process_state = STATE_COLLECT_DATA; // Move to the next state
                    break;
                }
                last = curr;
            }
        }
        else if(trigger_type == FALLING_EDGE)
        {
            // Look for a falling edge
            for (uint16_t i = 0; i < ADC_SINGLE_BUFF_LEN/2; i++)
            {
                curr = my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[0][i];
                if (curr_dest >= PLOT_DATA_LEN)
                {
                    curr_dest = 0;
                }
                if (last > trigger_level && curr <= trigger_level)
                {
                    trigger_index = curr_dest - 1;
                    adc_process_state = STATE_COLLECT_DATA;
                    break;
                }
                last = curr;

                curr = my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[1][i];
                if (curr_dest >= PLOT_DATA_LEN)
                {
                    curr_dest = 0;
                }
                if (last > trigger_level && curr <= trigger_level)
                {
                    trigger_index = curr_dest - 1;
                    adc_process_state = STATE_COLLECT_DATA;
                    break;
                }
                last = curr;

                curr = my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[2][i];
                if (curr_dest >= PLOT_DATA_LEN)
                {
                    curr_dest = 0;
                }
                if (last > trigger_level && curr <= trigger_level)
                {
                    trigger_index = curr_dest - 1;
                    adc_process_state = STATE_COLLECT_DATA;
                    break;
                }
                last = curr;
            }
        }
        if(adc_process_state == STATE_FIND_TRIGGER)
            break;

    case STATE_COLLECT_DATA:
        // Process the ADC data to collect it
        for(uint16_t i = 0; i < PLOT_DATA_LEN/2; i++)
        {
            my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[0][i];
            if (curr_dest >= PLOT_DATA_LEN) // Check if we reached the end of the plot buffer
            {
                curr_dest = 0; // Reset the destination index
            }

            my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[1][i];
            if (curr_dest >= PLOT_DATA_LEN)
            {
                curr_dest = 0;
            }

            my_plot_buff[use_plot_buff][curr_dest++] = adc_buff_ptr[2][i];
            if (curr_dest >= PLOT_DATA_LEN)
            {
                curr_dest = 0;
            }
        }
        plot_buff = my_plot_buff[use_plot_buff]; // Use the current plot buffer
        start_plot = 1; // Signal to start plotting
        use_plot_buff = 1-use_plot_buff; // Toggle the buffer to use
        adc_process_state = STATE_FIND_TRIGGER; // Reset the state to find the next trigger
        curr_dest = 0; // Reset the destination index for the next plot
        break;
    
    default:
        break;
    }
}
