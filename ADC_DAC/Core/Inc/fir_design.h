/*
 * fir_design.h
 *
 *  Created on: Jun 3, 2025
 *      Author: User
 */

#ifndef INC_FIR_DESIGN_H_
#define INC_FIR_DESIGN_H_

#include <stdint.h>

#define fir_N 400 // Number of filter taps
#define fir_M (fir_N - 1)
#define PI 3.14159265358979323846f

// Enum for filter types
typedef enum
{
    FIR_LOW,
    FIR_HIGH,
    FIR_BANDPASS,
    FIR_BANDSTOP
} fir_type_t;

// Global variables (extern declarations)
extern float fir_coeffs[fir_N];
extern float fir_fc1; // Lower cutoff (used also for single cutoff filters)
extern float fir_fc2; // Upper cutoff
extern fir_type_t fir_current_type;

// Function prototype
void design_fir(void);

#endif /* INC_FIR_DESIGN_H_ */
