/*
 * draw_utils.h
 *
 *  Created on: Jun 5, 2025
 *      Author: User
 */

#ifndef INC_DRAW_UTILS_H_
#define INC_DRAW_UTILS_H_

#include "math.h"
#include "string.h"
#include "stdint.h"
#include "fir_design.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_lcd.h"

typedef enum
{
    OSC,
    SIG,
    FIL,
	PUR,
} mode_t;

typedef enum
{
	SIN,
	SQR,
	TRI,
} waveform_t;

#define SCREEN_X 480
#define SCREEN_Y 272
#define FILTER_SELECT_BOX_X 95
#define FILTER_SELECT_BOX_Y 15
#define WAVEFORM_SELECT_BOX_X 160
#define WAVEFORM_SELECT_BOX_Y 50
#define FULL_SCREEN_BUT_X 160
#define FULL_SCREEN_BUT_Y 40

extern mode_t current_mode;
extern fir_type_t current_filter;
extern waveform_t current_waveform;
extern float waveform_mag;
extern float waveform_freq;
extern float waveform_duty;
extern float osc_Vpp, osc_T, osc_V, osc_trigger, osc_freq;
extern int anomaly_flag;
extern int test_flag;
extern uint8_t databuffer[400];
extern uint8_t drawbuffer[400];
extern uint8_t prevdrawbuffer[400];
extern uint16_t trigger_index;
extern uint8_t apply_filter;

extern uint8_t status, gesture;
extern uint8_t  lcd_status, touch_screen_it;
extern TS_StateTypeDef touch_screen_state;

void Display_Osc(void);
void Display_Sig_gen(void);
void Display_Filter(void);
void Display_pure(void);
void Select_Filter(uint16_t);
void Select_Waveform(uint16_t);
void touch_screen_response(void);
void draw_waveform(void);
void buffer_maker(float val, const char *prefix, char *out_buffer);
void float_to_string(float val, char *str, int decimal_places);

#endif /* INC_DRAW_UTILS_H_ */
