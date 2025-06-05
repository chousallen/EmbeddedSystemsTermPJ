/*
 * draw_utils.c
 *
 *  Created on: Jun 5, 2025
 *      Author: User
 */

#include "draw_utils.h"
#include "fir_design.h"
#include <stdio.h>
#include "myglobals.h"

mode_t current_mode = OSC;
fir_type_t current_filter = FIR_LOW;
waveform_t current_waveform = SIN;
float waveform_mag = 1;
float waveform_freq = 1000;
float waveform_duty = 0.3;
float osc_Vpp, osc_T, osc_V, osc_trigger, osc_freq;
int anomaly_flag = 0;
int test_flag = 0;
uint8_t databuffer[400];
uint8_t drawbuffer[400];
uint8_t prevdrawbuffer[400];
//uint16_t trigger_index = 199;
uint8_t apply_filter = 0;
uint8_t status = 0, gesture = 0;
uint8_t  lcd_status = LCD_OK, touch_screen_it = 0;
TS_StateTypeDef touch_screen_state;

void Display_Osc(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DrawRect(0, 0, 159, 50);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Oscilloscope", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Sig_gen", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Filter", RIGHT_MODE);

	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)"Full Screen", RIGHT_MODE);
	BSP_LCD_DrawRect(480 - FULL_SCREEN_BUT_X, 272 - FULL_SCREEN_BUT_Y, FULL_SCREEN_BUT_X - 1, FULL_SCREEN_BUT_Y);

	char buffer[50];
	sprintf(buffer, "trigger: %f", osc_trigger);
	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
}

void Display_pure(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);

//	char buffer[50];
//	sprintf(buffer, "Vpp: %f", osc_Vpp);
//	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, LEFT_MODE);
//	sprintf(buffer, "f: %f", osc_freq);
//	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
//	sprintf(buffer, "T: %f", osc_T);
//	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, RIGHT_MODE);
//	sprintf(buffer, "V: %f", osc_V);
//	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, RIGHT_MODE);
//
//	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"OSC", LEFT_MODE);
//	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"FIL", RIGHT_MODE);
}

void Display_Sig_gen(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Oscilloscope", LEFT_MODE);
	BSP_LCD_DrawRect(160, 0, 159, 50);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Sig_gen", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Filter", RIGHT_MODE);

	BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Select your waveform: ", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(8), (uint8_t *)"SINE", LEFT_MODE);
	BSP_LCD_DisplayStringAt(160, LINE(8), (uint8_t *)"SQUARE", LEFT_MODE);
	BSP_LCD_DisplayStringAt(320, LINE(8), (uint8_t *)"TRIANGLE", LEFT_MODE);
	BSP_LCD_DrawRect(160*current_waveform, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);

	char buffer[50];
	sprintf(buffer, "Vpp: %f", waveform_mag);
	BSP_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)buffer, LEFT_MODE);
	sprintf(buffer, "Freq: %f", waveform_freq);
	BSP_LCD_DisplayStringAt(0, LINE(12), (uint8_t *)buffer, LEFT_MODE);
	sprintf(buffer, "Duty ratio (for square): %f", waveform_duty);
	BSP_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)buffer, LEFT_MODE);
}

void Display_Filter(){
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Oscilloscope", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Sig_gen", CENTER_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"Filter", RIGHT_MODE);
	BSP_LCD_DrawRect(320, 0, 159, 50);

	BSP_LCD_DisplayStringAt(0, LINE(13), (uint8_t *)"Select your filter: ", LEFT_MODE);
	BSP_LCD_DisplayStringAt(96, LINE(14), (uint8_t *)"LPF", LEFT_MODE);
	BSP_LCD_DisplayStringAt(192, LINE(14), (uint8_t *)"HPF", LEFT_MODE);
	BSP_LCD_DisplayStringAt(288, LINE(14), (uint8_t *)"BPF", LEFT_MODE);
	BSP_LCD_DisplayStringAt(384, LINE(14), (uint8_t *)"BSF", LEFT_MODE);
	BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * (1 + current_filter), 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);

	BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)"GO!", CENTER_MODE);

	char buffer[50];
	sprintf(buffer, "fc1 (not used in LPF, HPF): %f", fir_fc1);
	BSP_LCD_DisplayStringAt(50, LINE(15), (uint8_t *)buffer, LEFT_MODE);
	sprintf(buffer, "fc2 (fc for LPF, HPF): %f", fir_fc2);
	BSP_LCD_DisplayStringAt(50, LINE(16), (uint8_t *)buffer, LEFT_MODE);
}

void Select_Waveform(uint16_t waveform){
	for(int i = 0; i < 3; ++i){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(WAVEFORM_SELECT_BOX_X*i, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
	}
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	switch(waveform){
	case 0:
		BSP_LCD_DrawRect(0, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
		current_waveform = SIN;
		break;
	case 1:
		BSP_LCD_DrawRect(160, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
		current_waveform = SQR;
		break;
	case 2:
		BSP_LCD_DrawRect(320, 100, WAVEFORM_SELECT_BOX_X - 1, WAVEFORM_SELECT_BOX_Y);
		current_waveform = TRI;
		break;
	}
}

void Select_Filter(uint16_t selected_fir){
	for(int i = 0; i < 4; ++i){
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X*(i+1), 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
	}
	BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
	switch(selected_fir){
	case 0:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_LOW;
		break;
	case 1:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * 2, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_HIGH;
		break;
	case 2:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * 3, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_BANDPASS;
		break;
	case 3:
		BSP_LCD_DrawRect(FILTER_SELECT_BOX_X * 4, 222, FILTER_SELECT_BOX_X, FILTER_SELECT_BOX_Y);
		current_filter = FIR_BANDSTOP;
		break;
	}
}

void touch_screen_response(){
	BSP_TS_ITClear();
	BSP_TS_GetState(&touch_screen_state);
//	BSP_TS_Get_GestureId(&touch_screen_state);
	touch_screen_it = 0;
	if(touch_screen_state.touchX[0] > 320) anomaly_flag = 1;
	if(current_mode == PUR){
		if(touch_screen_state.touchY[0] < 50){
			if(touch_screen_state.touchX[0] < 40){
				current_mode = OSC;
				Display_Osc();
				apply_filter = 0;
			}else if(touch_screen_state.touchX[0] > 440){
				current_mode = FIL;
				Display_Filter();
				apply_filter = 1;
			}
		}
		return;
	}
	if(touch_screen_state.touchX[0] < 320 && touch_screen_state.touchX[0] > 160 && touch_screen_state.touchY[0] < 50){
		Display_Sig_gen();
		current_mode = SIG;
	}
	if(touch_screen_state.touchX[0] < 160 && touch_screen_state.touchY[0] < 50){
		current_mode = OSC;
		Display_Osc();
		apply_filter = 0;
	}
	if(touch_screen_state.touchX[0] > 320 && touch_screen_state.touchY[0] < 50){
		Display_Filter();
		current_mode = FIL;
		apply_filter = 1;
	}
	switch(current_mode){
	case OSC:
		if(touch_screen_state.touchY[0] > SCREEN_Y - FULL_SCREEN_BUT_Y && touch_screen_state.touchX[0] > SCREEN_X - FULL_SCREEN_BUT_X){
			current_mode = PUR;
			Display_pure();
		}
		return;
	case SIG:
		if(touch_screen_state.touchY[0] > 100 && touch_screen_state.touchY[0] < 200){
			for(uint16_t j = 0; j < 3; j++){
				if((touch_screen_state.touchX[0] > (WAVEFORM_SELECT_BOX_X*j)) && (touch_screen_state.touchX[0] < WAVEFORM_SELECT_BOX_X * (1 + j))){
					Select_Waveform(j);
					break;
				}
			}
		}
		else
		{
			current_mode = SIG;
		}
		break;
	case FIL:
		if(touch_screen_state.touchY[0] > 222){
			for(uint16_t j = 0; j < 4; ++j){
				if((touch_screen_state.touchX[0] > FILTER_SELECT_BOX_X * (j + 1)) && (touch_screen_state.touchX[0] < FILTER_SELECT_BOX_X * (j + 2))){
					Select_Filter(j);
					break;
				}
			}
		}else if(touch_screen_state.touchY[0] > 50){
			current_mode = PUR;
			Display_pure();
		}
		return;
	default:
		return;
	}
}

void float_to_string(float val, char *str, int decimal_places) {
    if (val < 0) {
        *str++ = '-';
        val = -val;
    }

    int int_part = (int)val;
    int frac_part = (int)((val - int_part) * pow(10, decimal_places));

    // Convert integer part
    itoa(int_part, str, 10);
    while (*str) str++;  // Move pointer to end

    *str++ = '.';

    // Convert fractional part with leading zero if needed
    if (decimal_places == 2 && frac_part < 10)
        *str++ = '0';

    itoa(frac_part, str, 10);
}

void buffer_maker(float val, const char *prefix, char *out_buffer) {
    char numbuff[20];
    float_to_string(val, numbuff, 2);

    strcpy(out_buffer, prefix);      // Copy prefix like "Vpp: "
    strcat(out_buffer, numbuff);     // Append converted float
}

void draw_waveform(){
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"OSC", LEFT_MODE);
	BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)"FIL", RIGHT_MODE);

	char buffer[50];
	buffer_maker(osc_Vpp, "Vpp: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, LEFT_MODE);
	buffer_maker(osc_freq, "f: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, LEFT_MODE);
	buffer_maker(osc_T, "T: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(15), (uint8_t *)buffer, RIGHT_MODE);
	buffer_maker(osc_V, "V: ", buffer);
	BSP_LCD_DisplayStringAt(0, LINE(16), (uint8_t *)buffer, RIGHT_MODE);

	for(int j = 0; j < 400; ++j){
		uint16_t db_index = (trigger_index - 199 + 400 + j) % 400;
		float point_y = 255 - plot_buff[db_index];
		point_y = (point_y/255.0) * 215;
		drawbuffer[j] = (uint8_t) point_y;
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(j + 40, prevdrawbuffer[j], 1, 1);
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_DrawRect(j + 40, drawbuffer[j], 1, 1);
		prevdrawbuffer[j] = drawbuffer[j];
//		databuffer[db_index] = (databuffer[db_index] + 5) % 255;
	}
}
