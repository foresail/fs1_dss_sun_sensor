#include <msp430.h>
#include <stdint.h>
#include "adc.h"
#include "calc.h"
#include "main.h"

volatile unsigned char adc_done;
volatile int adc_res;

#define CAL_ADC_15T30  *((uint16_t *)0x1A1A)   // Temperature Sensor Calibration-30 C for 1V5
                                               // See device-specific datasheet for TLV table memory mapping
#define CAL_ADC_15T85  *((uint16_t *)0x1A1C)   // Temperature Sensor Calibration-85 C for 1V5

void ADC_INIT(void)
{
  // Configure ADC10 - Pulse sample mode; ADC10SC trigger
  ADC10CTL0 = ADC10SHT_8 + ADC10ON;         // 16 ADC10CLKs; ADC ON,temperature sample period>30us
  ADC10CTL1 = ADC10SHP + ADC10CONSEQ_0;     // s/w trig, single ch/conv
  ADC10CTL2 = ADC10RES;                     // 10-bit conversion results

  // Configure internal reference
  while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
  REFCTL0 |= REFVSEL_0+REFON;               // Select internal ref = 1.5V
                                            // Internal Reference ON
  ADC10IE |=ADC10IE0;                       // enable the Interrupt request for a completed ADC10_B conversion

  __delay_cycles(400);                      // Delay for Ref to settle

}

// reading internal msp430 temperature sensor
int16_t read_tempC(void)
{
    // int32_t needed because otherwise pre1 and 2 will overflow
    int32_t adcValue = read_adc_ch(ADC10INCH_10);
    int32_t pre1 = (adcValue - (int32_t)CAL_ADC_15T30)* (850 - 300);
    int32_t pre2 = ((int32_t)CAL_ADC_15T85 - (int32_t)CAL_ADC_15T30);
    int16_t temperature =  pre1 / pre2 + 300;  // in deciDegC

    // calibrated temperature in deciDegC
    return temperature + TEMPERATURE_BIAS;
}

static int read_adc_ch(unsigned char ch) {

    unsigned int i = 0;
    /* Wait for existing conversion */
    while((ADC10CTL1  & ADC10BUSY) && ++i < 100);

    ADC10CTL0 &= ~ADC10ENC; //Disable ADC
    ADC10MCTL0 = ADC10SREF_1 + ch; // Select ADC input channel
    adc_done = 0;
    ADC10CTL0 |= ADC10ENC + ADC10SC; // Sampling and conversion start
    while(!adc_done) {
        __bis_SR_register(LPM0_bits + GIE);
        __no_operation();
    }
    adc_done = 0;

    return adc_res;
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  switch(__even_in_range(ADC10IV,12))
  {
    case  0: break;                          // No interrupt
    case  2: break;                          // conversion result overflow
    case  4: break;                          // conversion time overflow
    case  6: break;                          // ADC10HI
    case  8: break;                          // ADC10LO
    case 10: break;                          // ADC10IN
    case 12: {
        adc_res = ADC10MEM0;
        adc_done = 1;

        __bic_SR_register_on_exit(LPM0_bits);
        break;                          // Clear CPUOFF bit from 0(SR)
    }
    default: break;
  }
}

