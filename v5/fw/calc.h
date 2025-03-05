#ifndef CALC_H
#define CALC_H

#include <stdint.h>
#include "main.h"
#define FRAM_VAR __attribute__((section(".fram_vars")))


// Values and Constants
#define LUT_SIZE 2048

#define CALC_OK             0x01
#define DIVISION_ZERO       0x02

#pragma SET_DATA_SECTION(".fram_vars")
extern int16_t X_BIAS;              // This value is multiplied by 8, to compensate for the scaling factor SCALE (13)
extern int16_t Y_BIAS;              // This value is multiplied by 8, to compensate for the scaling factor SCALE (20.6*8 = 165)
extern uint16_t VALUE_Z;            // This is the height of the pinhole scaled to the same size of the sensor
extern int16_t TEMPERATURE_BIAS;     // Temperature BIAS in deciDegC

extern uint16_t INT_TIME;
extern uint16_t SAMPLING_TIME;
extern uint16_t SAT_LEVEL;
extern uint8_t GAIN;
#pragma SET_DATA_SECTION()

// Variables
extern volatile int ind;
extern uint8_t x_data[256];
extern uint8_t y_data[256];
extern uint16_t SNR_X, SNR_Y;
extern int16_t VALUE_X;
extern int16_t VALUE_Y;
//extern uint8_t SCALE;

extern volatile uint8_t dataRequested;

// Functions
// sample sensor
int SAMPLE_SENSOR(void);
// sends start signals continuously to the sensor
void ST_SIGNAL_ENABLE(void);
// stops sending signals to sensor
// NO SAMPLING
void ST_SIGNAL_DISABLE(void);

// Function used to estimate the center bin location through interpolation
int16_t quadratic_middle(uint16_t *arr, char axis);
// This is the rolling filter, which filters the raw data
uint8_t rolling_filter(const uint8_t *arr, uint16_t *filtered_arr);
//Set sensor gains
void ss_gain(uint8_t gain);

// Requires a lot of memory!
#ifdef CALC_ANGLES
extern const uint16_t lt[LUT_SIZE];
// calculate saturation middle
uint16_t sat_calc_middle(uint16_t *arr, char axis);
// calculate sun angle
// Requires a lot of memory!
int16_t angle(uint16_t middle);
#endif

#endif /* CALC_H */
