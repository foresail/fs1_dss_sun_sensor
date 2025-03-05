#ifndef MAIN_H_
#define MAIN_H_
#define FRAM_VAR __attribute__((section(".fram_vars")))

#include <stdint.h>

#define USE_WDT

// Flip to enable or disable DEBUG mode
//#define DEBUG

// Values and Constants
#define LUT_SIZE 2048
#define TEMPERATURE_BIAS 0					// Calibration value for temperature

#define CALC_OK             0x01
#define DIVISION_ZERO       0x02

extern int16_t FRAM_VAR X_BIAS;            	// This value is multiplied by 8, to compensate for the scaling factor SCALE (13)
extern int16_t FRAM_VAR Y_BIAS;				// This value is multiplied by 8, to compensate for the scaling factor SCALE (20.6*8 = 165)
extern uint16_t FRAM_VAR VALUE_Z;			// This is the height of the pinhole scaled to the same size of the sensor

extern uint16_t FRAM_VAR INT_TIME;
extern uint16_t FRAM_VAR SAMPLING_TIME;
extern uint16_t FRAM_VAR SAT_LEVEL;
extern uint8_t FRAM_VAR GAIN;

// Variables
extern volatile int ind;
extern int sleep_mode;
extern uint8_t x_data[256];
extern uint8_t y_data[256];
extern uint16_t SNR_X, SNR_Y;
extern int16_t VALUE_X;
extern int16_t VALUE_Y;
extern uint8_t SCALE;
extern const uint16_t lt[LUT_SIZE];

extern volatile uint8_t dataRequested;

// Functions
int SAMPLE_SENSOR(void);
void CLOCK_INIT(void);
void DMA_INIT(void);
void IO_INIT(void);
void ST_SIGNAL_ENABLE(void);
#ifdef DEBUG
uint16_t sat_calc_middle(uint16_t *arr, char axis);
#endif
int16_t quadratic_middle(uint16_t *arr, char axis);
uint8_t rolling_filter(const uint8_t *arr, uint16_t *filtered_arr);
void ss_gain(uint8_t gain);
#ifdef DEBUG
int16_t angle(uint16_t middle);
#endif
void sleep(void);
void wakeup(void);
void INTEGRATION_TIMER_INIT(void);
void HB_TIMER_INIT(void);

// Macros
#define INTEGRATION_TIMER_ENABLE() TA0CTL |= MC_1
#define INTEGRATION_TIMER_DISABLE() TA0CTL &= ~MC_1

#define HB_TIMER_ENABLE() TB0CTL |= MC_1
#define HB_TIMER_DISABLE() TB0CTL &= ~MC_1

#define BOOST_ENABLE()  P2OUT |= BIT6
#define BOOST_DISABLE() P2OUT &= ~BIT6

#ifdef DEBUG
#define LED1_ON() P1OUT &= ~BIT4
#define LED2_ON() PJOUT &= ~BIT0

#define LED1_OFF() P1OUT |= BIT4
#define LED2_OFF() PJOUT |= BIT0
#endif

// Start pin control
#define START_PINS_UP() {\
        PJOUT |= BIT4;\
        P3OUT |= BIT7;\
	}

#define START_PINS_DOWN() {\
        PJOUT &= ~BIT4;\
        P3OUT &= ~BIT7;\
	}


#endif /* MAIN_H_ */
