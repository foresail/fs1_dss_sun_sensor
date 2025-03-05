#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#define USE_WDT

// Flip to enable or disable DEBUG mode
//#define DEBUG

extern uint8_t sleep_mode;

void CLOCK_INIT(void);
void DMA_INIT(void);
void IO_INIT(void);

void reset_idle_counter(void);
void sleep(void);
void wakeup(void);
void INTEGRATION_TIMER_INIT(void);
void HB_TIMER_INIT(void);

// Timers
#define INTEGRATION_TIMER_ENABLE() TA0CTL |= MC_1
#define INTEGRATION_TIMER_DISABLE() TA0CTL &= ~MC_1

#define HB_TIMER_ENABLE() TB0CTL |= MC_1
#define HB_TIMER_DISABLE() TB0CTL &= ~MC_1

// 5V BOOST
#define BOOST_ENABLE()  P3OUT |= BIT6
#define BOOST_DISABLE() P3OUT &= ~BIT6

#ifdef DEBUG
// LEDS
#define LED1_ON() P1OUT &= ~BIT3
#define LED1_OFF() P1OUT |= BIT3
#define LED1_TOGGLE()do { P1OUT ^=  BIT3; } while(0)

#define LED2_ON() P1OUT &= ~BIT4
#define LED2_OFF() P1OUT |= BIT4
#define LED2_TOGGLE()do { P1OUT ^=  BIT4; } while(0)

#else
#define LED1_ON()
#define LED1_OFF()
#define LED1_TOGGLE()
#define LED2_ON()
#define LED2_OFF()
#define LED2_TOGGLE()
#endif

// Start pin control
// Start X = PJ.4
// Start Y = P3.7
#define START_PINS_UP() {\
        PJOUT |= BIT4;\
        P3OUT |= BIT7;\
	}

#define START_PINS_DOWN() {\
        PJOUT &= ~BIT4;\
        P3OUT &= ~BIT7;\
	}


#endif /* MAIN_H_ */
