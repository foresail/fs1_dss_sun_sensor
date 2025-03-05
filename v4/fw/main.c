#include <msp430.h>
#include <msp430fr5739.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "SPI.h"
#include "DMA.h"
#include "i2c.h"
#include "adc.h"


/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Dynamic Variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/


// Sensor raw data arrays
uint8_t x_data[256];
uint8_t y_data[256];

// SNR variables
uint16_t SNR_X = 0;
uint16_t SNR_Y = 0;

// X and Y position values
int16_t VALUE_X = 0;
int16_t VALUE_Y = 0;

// These are values used for the SNR calculations
uint16_t max_value = 0;
uint16_t min_value = 0;

// These indexes are used to save the location of the point where we went over the saturation level
uint8_t low_index = 0;
uint8_t high_index = 0;
uint8_t max_index = 0;

uint8_t INTERVAL = 3;                       // This is for the rolling average calculation. Set to 5 by default, must be UNEVEN
uint8_t SHIFT = 2;                          // Shift is used in rolling average, for the array correction. Set to 2 by default formula SHIFT = ((INTERVAL-1)/2)
uint8_t SCALE = 8;                          // This is used for the Quadratic middle calculation. Set to 8 by default

// Sleep mode indicator flag. Sleep Mode - 0, Enabled - 1
int sleep_mode = 0;
volatile int int_flag = -1;

// This flag is used to indicated whether or not data is requested by the I2C handler
volatile uint8_t dataRequested = 0;


/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    FRAM Variables
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/


// FRAM variables and constants
int16_t FRAM_VAR X_BIAS = -126;                  // This value is multiplied by 8, to compensate for the scaling factor SCALE (127.5-143.26)*8 = -126
int16_t FRAM_VAR Y_BIAS = -4;                  // This value is multiplied by 8, to compensate for the scaling factor SCALE (127.5-128.02)*8 = -4

uint16_t FRAM_VAR VALUE_Z = 1;

uint16_t FRAM_VAR INT_TIME = 3200;              // This is the integration time variable. Tint = 1/0.75MHz*INT_TIME
uint16_t FRAM_VAR SAMPLING_TIME = 3200;         // This is the amount of clock ticks it takes to sample the sensor one time MUST BE OVER 3200

// This is the saturation level of the sensor. This value is used to determine when the data is saturated and what algorithm is used to estimate the center spot
// The saturation level is calculated with the interval and value between 0-250. I.e. a saturation level of 240 would be SAT_LEVEL = 240*INTERVAL(2) = 480
uint16_t FRAM_VAR SAT_LEVEL = 100;

// This is the gain of the Sun Sensor which is saved in FRAM
uint8_t FRAM_VAR GAIN = 0;

// This is used to determine the time during which a DMA transfer should have occured
uint16_t TIMEOUT_TIME = 3500;                  // Timeout time


/*
 * Watchdog time configuration
 *
 * Select SMCLK (6MHz) clock as the clock source WDT is cleared only in the heartbeat interrupt,
 * so the reset time must be greater than maximum heartbeat period.
 *
 * WDTIS_0 = 2^31 clock cycles = 358 seconds aka ~6min @6MHz
 * WDTIS_1 = 2^27 clock cycles = 22 seconds @6MHz
 * WDTIS_2 = 2^23 clock cycles = 1.4 seconds @6MHz
 */


#ifdef USE_WDT
#define RESET_WDT() (WDTCTL = WDTPW + WDTSSEL_0 + WDTCNTCL + WDTIS_0)
#else
#define RESET_WDT() (WDTCTL = WDTPW + WDTHOLD) // Disabled!
#endif


/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Timer interrupts
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/


#pragma vector=TIMER0_A0_VECTOR
__interrupt void INTEGRATION_TIMER_ISR(void)
{
    INTEGRATION_TIMER_DISABLE();

    switch (int_flag)
    {
    case 0: {

        // Set INTEGRATION period and clear timer
        TA0CCR0 = INT_TIME;
        TA0CTL |= TACLR;

        // Pull start pins up
        START_PINS_UP();

        // Enable the integration timer
        INTEGRATION_TIMER_ENABLE();

        int_flag = 1;

        break;
    }

    case 1: {
        if (dataRequested == 1) {

            // Clear DMA interrupts
            DMA0CTL &= ~DMAIFG;
            DMA1CTL &= ~DMAIFG;

            // Reset SPI and DMA
            SPI_RESET();
            DMA_INIT();

            dataRequested = 2;
        }

        // Set INTEGRATION period and clear timer
        TA0CCR0 = TIMEOUT_TIME;
        TA0CTL |= TACLR;

        // Pull start pins down
        START_PINS_DOWN();

        INTEGRATION_TIMER_ENABLE();

        int_flag = 2;

        break;
    }
    case 2: {
        if (dataRequested == 2) dma_timeout = 1;

        ST_SIGNAL_ENABLE();

        break;
    }

    default:{
        // Disable the integration timer
        INTEGRATION_TIMER_DISABLE();

        break;
    }
    }
}

/*
 * Timer B0 interrupt (triggering every 50ms) - Used to wakeup the processor and check any new events
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B0(void)
{
    // Wake up the main loop
    __bic_SR_register_on_exit(LPM0_bits + SMCLKOFF);
}


/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Main Loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/


int main(void)
{
    // Initialize WDT
    RESET_WDT();

    /* Basic initalization */
    CLOCK_INIT();
    ADC_INIT();
    SPI_INIT();
    I2C_INIT();
    IO_INIT();
    DMA_INIT();                                 //Initialize DMA for SPI data transfer
    INTEGRATION_TIMER_INIT();
    HB_TIMER_INIT();

    __enable_interrupt();                       //Enable interrupts


    //Boost converter off
    BOOST_DISABLE();

    // Set Sun Sensor Gain to low
    ss_gain(GAIN);

    // Enable the sensor start signal
    ST_SIGNAL_ENABLE();

    unsigned int idle_counter = ~0;

    // Prevent IO from dying
    PM5CTL0 &= ~LOCKLPM5;

#ifdef DEBUG
    // Set Power Indication LED on
    LED1_ON();
#endif

    while(1) {

        // Goto sleep with interrupts and SMCLK enabled
        __bis_SR_register(LPM0_bits + GIE);
        __no_operation();

        if(new_message){
            // Stop the HB timer during command handling
            HB_TIMER_DISABLE();

            // Check for new commands
            handle_command();

            // Command handled - Reset flags
            new_message = 0;
            idle_counter = 0;

            // Start the HB timer
            HB_TIMER_ENABLE();
        }

        // Reset the watchdog timer
        RESET_WDT();

        TB0CTL |= TBCLR;        // Clear interrupt

        // Processor wakes up every 40ms --> Goes to sleep after 4ms*2000 = 8s
        if (idle_counter > 2000 && !sleep_mode) {
            // Goto "deepsleep" if I2C is not actively used
            sleep();
        }
        else {
            idle_counter++;
            // Resets itself after 4ms*10000 = 40s
            if (idle_counter >= 10000) {

                // Trigger Power-On-Reset (POR) after ~40 seconds of idling
                PMMCTL0 |= PMMSWPOR;

                while(1);
            }
        }
    }
}


/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/


int SAMPLE_SENSOR()
{

    dataRequested = 1;

    while(1){
        if((DMA_x_flag == 1 && DMA_y_flag == 1) || dma_timeout){

            // Reset data request variable
            dataRequested = 0;

            // Disable the DMA and SPI controllers
            SPI_DISABLE();
            DMA_DISABLE();

            // Set DMA x and y flags to zero, so that they're ready for the next read
            DMA_x_flag = 0;
            DMA_y_flag = 0;

            // Check to see if we sampled the sensor or if a timeout occured
            if (dma_timeout != 0){
            	// Reset DMA timeout flag
                dma_timeout = 0;
                return 0;
            }

            // Reset DMA timeout flag
            dma_timeout = 0;
            return 1;
        }
    }
}


/* Start signal enable function. This function is used to send the start signals continuously to the sensor.*/
void ST_SIGNAL_ENABLE(void)
{
    // Initialise interrupt flag
    int_flag = 0;

    // Set SAMPLING period and clear timer
    TA0CCR0 = SAMPLING_TIME;
    TA0CTL |= TACLR;

    // Disable DMA interrupt
    DMA0CTL &= ~DMAIE;                                   //DMA interrupt disable
    DMA1CTL &= ~DMAIE;                                   //DMA interrupt disable

    // Pull start pins down
    START_PINS_DOWN();

    // Start SAMPLING TIMER
    INTEGRATION_TIMER_ENABLE();
}


void ST_SIGNAL_DISABLE(void)
{
    // Disable the integration timer
    INTEGRATION_TIMER_DISABLE();

    // Set the interrupt switch case variable to -1
    int_flag = -1;
}


#ifdef DEBUG
uint16_t sat_calc_middle(uint16_t *arr, char axis)
{
	// Calculate the appropriate SNR value and adjust for the INTERVAL summation from the rolling filter
	if (axis == 'x') {
		SNR_X = (max_value - min_value)/INTERVAL;
	}
	else if (axis == 'y') {
		SNR_Y = (max_value - min_value)/INTERVAL;
	}
	else {
		return 0;
	}

    // Initialize variables
    uint32_t total_xy = 0;
    uint32_t total_y = 0;
    unsigned int i;

    // Calculate the mass totals - The indexes were calculated in rolling_average() so we can use them as limits automatically
    for (i = low_index; i <= high_index; i++) {
		total_xy = total_xy + i * arr[i];
		total_y = total_y + arr[i];
    }

    // Prevent division by zero error
    if (total_xy == 0 || total_y == 0) return 0;

    // Compute the center bin
    uint16_t ret = (total_xy) / (total_y);

    // Return the corrected center bin calculation
    if (axis=='x') return (ret + X_BIAS);
    else if (axis=='y') return (ret + Y_BIAS);

    // Axis was not defined --> return 0
    return 0;
}
#endif


// Function used to estimate the center bin location through interpolation
// For a detailed description on the operation of this filter see: https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
int16_t quadratic_middle(uint16_t *arr, char axis)
{
	// Calculate the appropriate SNR value and adjust for the INTERVAL summation from the rolling filter
	if (axis == 'x') {
		SNR_X = (max_value - min_value)/INTERVAL;
	}
	else if (axis == 'y') {
		SNR_Y = (max_value - min_value)/INTERVAL;
	}
	else {
	    // Axis was not defined --> return -1
		return -1;
	}

	// If the index of the maximum value is not in the specified range (I.E. the light spot is on the sensor edge), return -1
	if ((max_index < 2) || (max_index > 253)) {
	    return -1;
	}

    // Get array values from around the peak
    int16_t y1 = arr[max_index - 1];
    int16_t y2 = arr[max_index];
    int16_t y3 = arr[max_index + 1];

    // Max value for d is 255*INTERVAL(3)*SCALE(8)/SUM(1) = ±6120
    int16_t d = 0;

    // Perform a summation of the data
    int16_t sum = 2*(2*y2 - y1 - y3);

    // Handle division with 0
    if(sum == 0) {
        return DIVISION_ZERO;
    }

    // Calculate the location of the center bin and scale the result
    d = ((y3 - y1) * SCALE) / sum;

    int16_t center = max_index * SCALE + d;

    // Return the corrected center bin calculation
    if (axis=='x') {
        VALUE_X = center + X_BIAS;
    }
    else if (axis=='y'){
    	VALUE_Y = center + Y_BIAS;
    }

    return CALC_OK;
}


// This is the rolling filter, which filters the raw data. It also calculated the low, max and high indexes as well as the minimum and maximum values of the data
uint8_t rolling_filter(const uint8_t *arr, uint16_t *filtered_arr){

	// Helper variables
    uint16_t sum = 0;
    uint8_t ret = 0;
    unsigned int i;

    // Initialize all indexes to zero
    low_index = 0;
    high_index = 0;
    max_index = 0;

    // The max and min values must be set to 1 and 65535 (max for a uint16_t variable).
    max_value = 1;
    min_value = 65535;

    for(i = 0; i < 256 + SHIFT; i++){

    	if(i < 256) sum += arr[i];

        if(i >= INTERVAL) sum -= arr[i - INTERVAL];

        if(i >= SHIFT) {
        	// Add the summed variable to memory
        	filtered_arr[i - SHIFT] = sum;

            // Save the low and high indexes of the data
            if ((sum > SAT_LEVEL) && (low_index == 0)){
            	low_index = i-SHIFT;

            	// Set the return value to 1 to indicate a saturation event
            	ret = 1;
            }
            else if ((sum < SAT_LEVEL) && (low_index != 0) && (high_index == 0)) high_index = i-SHIFT;

            // Record the min and max values from the data
            if ((sum < min_value) && (sum > 0)) min_value = sum;            // Check to see if the sum is larger than 0 to prevent division by zero
            else if (sum > max_value){
            	max_value = sum;
            	max_index = i-SHIFT;
            }
        }
    }

    // If the sensor is saturated in such a manner, that the saturation threshold isn't passed by the end of the array end - set high index to last element in array
    if((low_index != 0) && (high_index == 0)) high_index = 255;

    return ret;
}


void ss_gain(uint8_t gain)
{
    switch (gain){
    	// If 1 set gain to high
        case 1:
            PJOUT |= BIT1;
            break;
        // If 0 set gain to low
        case 0:
            PJOUT &= ~BIT1;
            break;
    }
}


#ifdef DEBUG
int16_t angle(uint16_t middle)
{
    int sign = 0;
    int16_t angle_out;
    if(middle < 1024) middle = (1024 - middle);
    else {
        middle -= 1024;
        sign = 1;
    }
    middle = 2*middle;
    angle_out = lt[middle];

    if (!sign) angle_out = -angle_out;

    return angle_out;
}
#endif


void sleep()
{
    // Disable boost converter (AKA shutdown profile sensor)
    BOOST_DISABLE();

    // Disable Start Signal
    ST_SIGNAL_DISABLE();

#ifdef DEBUG
    // Set LEDs off
    LED1_OFF();
    LED2_OFF();
#endif

    // Set sleep mode flag on
    sleep_mode = 1;
}

void wakeup()
{
    // Enable boost converter
    BOOST_ENABLE();

    // Enable Start Signal
    ST_SIGNAL_ENABLE();

#ifdef DEBUG
    // Set LED on
    LED1_ON();
#endif

    // Set sleep mode flag
    sleep_mode = 0;
}


/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Initialization functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
void CLOCK_INIT(){
    CSCTL0_H = 0xA5;                                    //Password to enable clock manipulation

    //Configure internal crystal (24MHz) as main clock
    CSCTL1 |= DCOFSEL_3 | DCORSEL;                      // Set max. DCO setting 24MHz
    CSCTL2 = SELA_3 + SELS_3 + SELM_3;                  // set ACLK = DC0 = ; MCLK = SMCLK = DCO
    CSCTL3 = DIVA_1 + DIVS_2 + DIVM_0;                  // set all dividers ACLK/2=12MHz, SMCLK/4=6MHz, MCLK/1=24MHz
}

void INTEGRATION_TIMER_INIT(){
    //ACLK set to 12MHz
    //ACLK, UP mode, divide by 1 and 2 --> timer frequency = 12MHz/2 = 6MHz, counter up mode
    TA0CTL = ID_0 + TASSEL_1 + MC_1;
    TA0CCR0 = INT_TIME;                                 // Set the timer/integration time
    TA0EX0 = 0x01;                                      //Divide by 2
    TA0CCTL0 = CCIE;                                    //TACCR1 interrupt enable
    TA0CTL |= TACLR;
}

void HB_TIMER_INIT(){
    /*
     * Configure heartbeat timer to TimerB0
     *
     * SMCLK = DCO / 1
     * Timer frequency = SMCLK (6MHz) / (8) = 0.75 MHz
     * Timer interrupt = TB0CCR0/Timer frequency (30000/0.75MHz = ~40ms)
     */

    TB0CTL = ID_3 + TBSSEL_2 + MC_1;    // SMCLK, UP mode, divide by 8
    TB0CCR0 = 3000;                     // Count up to this value
    TB0EX0 = TBIDEX_0;                  // Divide by 1
    TB0CCTL0 = CCIE;                    // TBCCR0 interrupt enabled
    TB0CTL |= TBCLR;                    // Clear interrupt
    CSCTL4 &= ~SMCLKOFF;
    TB0R = 0;
}

void IO_INIT(){

#ifdef DEBUG
    //LED 1
    P1SELC &= ~BIT4;                //Select pin 1.4 to I/O
    P1DIR |= BIT4;                  //Select pin 1.4 to output
    P1OUT |= BIT4;

    //LED 2
    PJSELC &= ~BIT0;                //Select pin J.0 to I/O
    PJDIR |= BIT0;                  //Select pin J.0 to output
    PJOUT |= BIT0;
#endif

    //Start signal X (PJ.4)
    PJSELC &= ~BIT4;                //Select pin J.4 to I/O
    PJDIR |= BIT4;                  //Select pin J.4 to output
    PJOUT |= BIT4;                  //Set start signal high

    //Start signal Y (P3.7)
    P3SELC &= ~BIT7;                //Select pin 3.7 to I/O
    P3DIR |= BIT7;                  //Select pin 3.7 to output
    P3OUT |= BIT7;                  //Set start signal high

    //Set SS clock to SMCLK
    P3SELC |= BIT4;
    P3DIR |= BIT4;

    //Start Boost enable (P2.6)
    P2SELC &= ~BIT6;                //Select pin 2.6 to I/O
    P2DIR |= BIT6;                  //Select pin 2.6 to output
    P2OUT &= ~BIT6;                 //Set Boost off

    // Gain selection (PJ.1)
    PJSELC &= ~BIT1;                //Select pin J.0 to I/O
    PJDIR |= BIT1;                  //Select pin J.0 to output
    PJOUT |= BIT1;                 //Set gain output to low (default)
}

/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Lookup table
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

#ifdef DEBUG
const uint16_t lt[LUT_SIZE] = {
       0,   14,   28,   42,   56,   70,   84,   98,
     112,  126,  140,  154,  168,  182,  196,  210,
     224,  238,  252,  266,  280,  294,  308,  322,
     336,  350,  364,  378,  392,  406,  420,  434,
     448,  462,  476,  490,  504,  518,  532,  546,
     560,  573,  587,  601,  615,  629,  643,  657,
     671,  685,  699,  713,  727,  741,  755,  769,
     783,  797,  811,  825,  839,  853,  867,  881,
     895,  909,  923,  937,  951,  965,  979,  993,
    1007, 1021, 1035, 1049, 1063, 1077, 1091, 1105,
    1119, 1133, 1147, 1161, 1175, 1189, 1203, 1217,
    1231, 1245, 1259, 1273, 1287, 1301, 1315, 1329,
    1343, 1357, 1371, 1385, 1399, 1413, 1427, 1440,
    1454, 1468, 1482, 1496, 1510, 1524, 1538, 1552,
    1566, 1580, 1594, 1608, 1622, 1636, 1650, 1664,
    1678, 1692, 1706, 1720, 1734, 1748, 1762, 1776,
    1790, 1804, 1818, 1832, 1846, 1860, 1874, 1888,
    1902, 1916, 1930, 1944, 1958, 1972, 1986, 2000,
    2013, 2027, 2041, 2055, 2069, 2083, 2097, 2111,
    2125, 2139, 2153, 2167, 2181, 2195, 2209, 2223,
    2237, 2251, 2265, 2279, 2293, 2307, 2321, 2335,
    2349, 2363, 2377, 2391, 2405, 2419, 2432, 2446,
    2460, 2474, 2488, 2502, 2516, 2530, 2544, 2558,
    2572, 2586, 2600, 2614, 2628, 2642, 2656, 2670,
    2684, 2698, 2712, 2726, 2740, 2754, 2768, 2781,
    2795, 2809, 2823, 2837, 2851, 2865, 2879, 2893,
    2907, 2921, 2935, 2949, 2963, 2977, 2991, 3005,
    3019, 3033, 3047, 3061, 3074, 3088, 3102, 3116,
    3130, 3144, 3158, 3172, 3186, 3200, 3214, 3228,
    3242, 3256, 3270, 3284, 3298, 3312, 3325, 3339,
    3353, 3367, 3381, 3395, 3409, 3423, 3437, 3451,
    3465, 3479, 3493, 3507, 3521, 3535, 3548, 3562,
    3576, 3590, 3604, 3618, 3632, 3646, 3660, 3674,
    3688, 3702, 3716, 3730, 3744, 3757, 3771, 3785,
    3799, 3813, 3827, 3841, 3855, 3869, 3883, 3897,
    3911, 3925, 3938, 3952, 3966, 3980, 3994, 4008,
    4022, 4036, 4050, 4064, 4078, 4092, 4105, 4119,
    4133, 4147, 4161, 4175, 4189, 4203, 4217, 4231,
    4245, 4259, 4272, 4286, 4300, 4314, 4328, 4342,
    4356, 4370, 4384, 4398, 4412, 4425, 4439, 4453,
    4467, 4481, 4495, 4509, 4523, 4537, 4551, 4564,
    4578, 4592, 4606, 4620, 4634, 4648, 4662, 4676,
    4690, 4703, 4717, 4731, 4745, 4759, 4773, 4787,
    4801, 4815, 4828, 4842, 4856, 4870, 4884, 4898,
    4912, 4926, 4940, 4953, 4967, 4981, 4995, 5009,
    5023, 5037, 5051, 5064, 5078, 5092, 5106, 5120,
    5134, 5148, 5162, 5176, 5189, 5203, 5217, 5231,
    5245, 5259, 5273, 5286, 5300, 5314, 5328, 5342,
    5356, 5370, 5384, 5397, 5411, 5425, 5439, 5453,
    5467, 5481, 5494, 5508, 5522, 5536, 5550, 5564,
    5578, 5591, 5605, 5619, 5633, 5647, 5661, 5675,
    5688, 5702, 5716, 5730, 5744, 5758, 5772, 5785,
    5799, 5813, 5827, 5841, 5855, 5868, 5882, 5896,
    5910, 5924, 5938, 5951, 5965, 5979, 5993, 6007,
    6021, 6034, 6048, 6062, 6076, 6090, 6104, 6117,
    6131, 6145, 6159, 6173, 6187, 6200, 6214, 6228,
    6242, 6256, 6270, 6283, 6297, 6311, 6325, 6339,
    6352, 6366, 6380, 6394, 6408, 6422, 6435, 6449,
    6463, 6477, 6491, 6504, 6518, 6532, 6546, 6560,
    6573, 6587, 6601, 6615, 6629, 6642, 6656, 6670,
    6684, 6698, 6711, 6725, 6739, 6753, 6767, 6780,
    6794, 6808, 6822, 6836, 6849, 6863, 6877, 6891,
    6905, 6918, 6932, 6946, 6960, 6973, 6987, 7001,
    7015, 7029, 7042, 7056, 7070, 7084, 7097, 7111,
    7125, 7139, 7153, 7166, 7180, 7194, 7208, 7221,
    7235, 7249, 7263, 7276, 7290, 7304, 7318, 7332,
    7345, 7359, 7373, 7387, 7400, 7414, 7428, 7442,
    7455, 7469, 7483, 7497, 7510, 7524, 7538, 7552,
    7565, 7579, 7593, 7607, 7620, 7634, 7648, 7662,
    7675, 7689, 7703, 7716, 7730, 7744, 7758, 7771,
    7785, 7799, 7813, 7826, 7840, 7854, 7868, 7881,
    7895, 7909, 7922, 7936, 7950, 7964, 7977, 7991,
    8005, 8018, 8032, 8046, 8060, 8073, 8087, 8101,
    8114, 8128, 8142, 8156, 8169, 8183, 8197, 8210,
    8224, 8238, 8251, 8265, 8279, 8293, 8306, 8320,
    8334, 8347, 8361, 8375, 8388, 8402, 8416, 8430,
    8443, 8457, 8471, 8484, 8498, 8512, 8525, 8539,
    8553, 8566, 8580, 8594, 8607, 8621, 8635, 8648,
    8662, 8676, 8689, 8703, 8717, 8730, 8744, 8758,
    8771, 8785, 8799, 8812, 8826, 8840, 8853, 8867,
    8881, 8894, 8908, 8922, 8935, 8949, 8963, 8976,
    8990, 9004, 9017, 9031, 9044, 9058, 9072, 9085,
    9099, 9113, 9126, 9140, 9154, 9167, 9181, 9194,
    9208, 9222, 9235, 9249, 9263, 9276, 9290, 9303,
    9317, 9331, 9344, 9358, 9372, 9385, 9399, 9412,
    9426, 9440, 9453, 9467, 9480, 9494, 9508, 9521,
    9535, 9548, 9562, 9576, 9589, 9603, 9617, 9630,
    9644, 9657, 9671, 9684, 9698, 9712, 9725, 9739,
    9752, 9766, 9780, 9793, 9807, 9820, 9834, 9848,
    9861, 9875, 9888, 9902, 9915, 9929, 9943, 9956,
    9970, 9983, 9997, 10010, 10024, 10038, 10051, 10065,
    10078, 10092, 10105, 10119, 10132, 10146, 10160, 10173,
    10187, 10200, 10214, 10227, 10241, 10254, 10268, 10281,
    10295, 10309, 10322, 10336, 10349, 10363, 10376, 10390,
    10403, 10417, 10430, 10444, 10457, 10471, 10484, 10498,
    10512, 10525, 10539, 10552, 10566, 10579, 10593, 10606,
    10620, 10633, 10647, 10660, 10674, 10687, 10701, 10714,
    10728, 10741, 10755, 10768, 10782, 10795, 10809, 10822,
    10836, 10849, 10863, 10876, 10890, 10903, 10917, 10930,
    10944, 10957, 10971, 10984, 10998, 11011, 11025, 11038,
    11051, 11065, 11078, 11092, 11105, 11119, 11132, 11146,
    11159, 11173, 11186, 11200, 11213, 11227, 11240, 11253,
    11267, 11280, 11294, 11307, 11321, 11334, 11348, 11361,
    11374, 11388, 11401, 11415, 11428, 11442, 11455, 11469,
    11482, 11495, 11509, 11522, 11536, 11549, 11563, 11576,
    11589, 11603, 11616, 11630, 11643, 11657, 11670, 11683,
    11697, 11710, 11724, 11737, 11750, 11764, 11777, 11791,
    11804, 11817, 11831, 11844, 11858, 11871, 11884, 11898,
    11911, 11925, 11938, 11951, 11965, 11978, 11992, 12005,
    12018, 12032, 12045, 12058, 12072, 12085, 12099, 12112,
    12125, 12139, 12152, 12165, 12179, 12192, 12206, 12219,
    12232, 12246, 12259, 12272, 12286, 12299, 12312, 12326,
    12339, 12352, 12366, 12379, 12392, 12406, 12419, 12432,
    12446, 12459, 12473, 12486, 12499, 12513, 12526, 12539,
    12553, 12566, 12579, 12592, 12606, 12619, 12632, 12646,
    12659, 12672, 12686, 12699, 12712, 12726, 12739, 12752,
    12766, 12779, 12792, 12805, 12819, 12832, 12845, 12859,
    12872, 12885, 12899, 12912, 12925, 12938, 12952, 12965,
    12978, 12992, 13005, 13018, 13031, 13045, 13058, 13071,
    13084, 13098, 13111, 13124, 13138, 13151, 13164, 13177,
    13191, 13204, 13217, 13230, 13244, 13257, 13270, 13283,
    13297, 13310, 13323, 13336, 13350, 13363, 13376, 13389,
    13403, 13416, 13429, 13442, 13456, 13469, 13482, 13495,
    13508, 13522, 13535, 13548, 13561, 13575, 13588, 13601,
    13614, 13627, 13641, 13654, 13667, 13680, 13693, 13707,
    13720, 13733, 13746, 13759, 13773, 13786, 13799, 13812,
    13825, 13839, 13852, 13865, 13878, 13891, 13905, 13918,
    13931, 13944, 13957, 13970, 13984, 13997, 14010, 14023,
    14036, 14049, 14063, 14076, 14089, 14102, 14115, 14128,
    14142, 14155, 14168, 14181, 14194, 14207, 14220, 14234,
    14247, 14260, 14273, 14286, 14299, 14312, 14326, 14339,
    14352, 14365, 14378, 14391, 14404, 14417, 14431, 14444,
    14457, 14470, 14483, 14496, 14509, 14522, 14535, 14549,
    14562, 14575, 14588, 14601, 14614, 14627, 14640, 14653,
    14666, 14680, 14693, 14706, 14719, 14732, 14745, 14758,
    14771, 14784, 14797, 14810, 14823, 14836, 14850, 14863,
    14876, 14889, 14902, 14915, 14928, 14941, 14954, 14967,
    14980, 14993, 15006, 15019, 15032, 15045, 15058, 15071,
    15085, 15098, 15111, 15124, 15137, 15150, 15163, 15176,
    15189, 15202, 15215, 15228, 15241, 15254, 15267, 15280,
    15293, 15306, 15319, 15332, 15345, 15358, 15371, 15384,
    15397, 15410, 15423, 15436, 15449, 15462, 15475, 15488,
    15501, 15514, 15527, 15540, 15553, 15566, 15579, 15592,
    15605, 15618, 15631, 15644, 15657, 15670, 15683, 15696,
    15709, 15722, 15735, 15748, 15760, 15773, 15786, 15799,
    15812, 15825, 15838, 15851, 15864, 15877, 15890, 15903,
    15916, 15929, 15942, 15955, 15968, 15980, 15993, 16006,
    16019, 16032, 16045, 16058, 16071, 16084, 16097, 16110,
    16123, 16136, 16148, 16161, 16174, 16187, 16200, 16213,
    16226, 16239, 16252, 16265, 16277, 16290, 16303, 16316,
    16329, 16342, 16355, 16368, 16380, 16393, 16406, 16419,
    16432, 16445, 16458, 16471, 16483, 16496, 16509, 16522,
    16535, 16548, 16561, 16573, 16586, 16599, 16612, 16625,
    16638, 16650, 16663, 16676, 16689, 16702, 16715, 16727,
    16740, 16753, 16766, 16779, 16792, 16804, 16817, 16830,
    16843, 16856, 16868, 16881, 16894, 16907, 16920, 16933,
    16945, 16958, 16971, 16984, 16997, 17009, 17022, 17035,
    17048, 17060, 17073, 17086, 17099, 17112, 17124, 17137,
    17150, 17163, 17175, 17188, 17201, 17214, 17226, 17239,
    17252, 17265, 17278, 17290, 17303, 17316, 17329, 17341,
    17354, 17367, 17380, 17392, 17405, 17418, 17430, 17443,
    17456, 17469, 17481, 17494, 17507, 17520, 17532, 17545,
    17558, 17570, 17583, 17596, 17609, 17621, 17634, 17647,
    17659, 17672, 17685, 17697, 17710, 17723, 17736, 17748,
    17761, 17774, 17786, 17799, 17812, 17824, 17837, 17850,
    17862, 17875, 17888, 17900, 17913, 17926, 17938, 17951,
    17964, 17976, 17989, 18002, 18014, 18027, 18040, 18052,
    18065, 18078, 18090, 18103, 18115, 18128, 18141, 18153,
    18166, 18179, 18191, 18204, 18216, 18229, 18242, 18254,
    18267, 18280, 18292, 18305, 18317, 18330, 18343, 18355,
    18368, 18380, 18393, 18406, 18418, 18431, 18443, 18456,
    18469, 18481, 18494, 18506, 18519, 18531, 18544, 18557,
    18569, 18582, 18594, 18607, 18619, 18632, 18645, 18657,
    18670, 18682, 18695, 18707, 18720, 18732, 18745, 18757,
    18770, 18783, 18795, 18808, 18820, 18833, 18845, 18858,
    18870, 18883, 18895, 18908, 18920, 18933, 18945, 18958,
    18970, 18983, 18995, 19008, 19020, 19033, 19045, 19058,
    19070, 19083, 19095, 19108, 19120, 19133, 19145, 19158,
    19170, 19183, 19195, 19208, 19220, 19233, 19245, 19258,
    19270, 19283, 19295, 19307, 19320, 19332, 19345, 19357,
    19370, 19382, 19395, 19407, 19420, 19432, 19444, 19457,
    19469, 19482, 19494, 19507, 19519, 19531, 19544, 19556,
    19569, 19581, 19594, 19606, 19618, 19631, 19643, 19656,
    19668, 19680, 19693, 19705, 19718, 19730, 19742, 19755,
    19767, 19780, 19792, 19804, 19817, 19829, 19841, 19854,
    19866, 19879, 19891, 19903, 19916, 19928, 19940, 19953,
    19965, 19977, 19990, 20002, 20015, 20027, 20039, 20052,
    20064, 20076, 20089, 20101, 20113, 20126, 20138, 20150,
    20163, 20175, 20187, 20200, 20212, 20224, 20237, 20249,
    20261, 20273, 20286, 20298, 20310, 20323, 20335, 20347,
    20360, 20372, 20384, 20396, 20409, 20421, 20433, 20446,
    20458, 20470, 20482, 20495, 20507, 20519, 20532, 20544,
    20556, 20568, 20581, 20593, 20605, 20617, 20630, 20642,
    20654, 20666, 20679, 20691, 20703, 20715, 20728, 20740,
    20752, 20764, 20776, 20789, 20801, 20813, 20825, 20838,
    20850, 20862, 20874, 20886, 20899, 20911, 20923, 20935,
    20947, 20960, 20972, 20984, 20996, 21008, 21021, 21033,
    21045, 21057, 21069, 21082, 21094, 21106, 21118, 21130,
    21142, 21155, 21167, 21179, 21191, 21203, 21215, 21228,
    21240, 21252, 21264, 21276, 21288, 21300, 21313, 21325,
    21337, 21349, 21361, 21373, 21385, 21398, 21410, 21422,
    21434, 21446, 21458, 21470, 21482, 21494, 21507, 21519,
    21531, 21543, 21555, 21567, 21579, 21591, 21603, 21615,
    21628, 21640, 21652, 21664, 21676, 21688, 21700, 21712,
    21724, 21736, 21748, 21760, 21772, 21785, 21797, 21809,
    21821, 21833, 21845, 21857, 21869, 21881, 21893, 21905,
    21917, 21929, 21941, 21953, 21965, 21977, 21989, 22001,
    22013, 22025, 22037, 22049, 22061, 22073, 22085, 22097,
    22109, 22121, 22133, 22145, 22157, 22169, 22181, 22193,
    22205, 22217, 22229, 22241, 22253, 22265, 22277, 22289,
    22301, 22313, 22325, 22337, 22349, 22361, 22373, 22385,
    22397, 22409, 22421, 22433, 22445, 22457, 22469, 22481,
    22493, 22505, 22516, 22528, 22540, 22552, 22564, 22576,
    22588, 22600, 22612, 22624, 22636, 22648, 22660, 22671,
    22683, 22695, 22707, 22719, 22731, 22743, 22755, 22767,
    22779, 22790, 22802, 22814, 22826, 22838, 22850, 22862,
    22874, 22886, 22897, 22909, 22921, 22933, 22945, 22957,
    22969, 22980, 22992, 23004, 23016, 23028, 23040, 23052,
    23063, 23075, 23087, 23099, 23111, 23123, 23134, 23146,
    23158, 23170, 23182, 23194, 23205, 23217, 23229, 23241,
    23253, 23264, 23276, 23288, 23300, 23312, 23323, 23335,
    23347, 23359, 23371, 23382, 23394, 23406, 23418, 23429,
    23441, 23453, 23465, 23477, 23488, 23500, 23512, 23524,
    23535, 23547, 23559, 23571, 23582, 23594, 23606, 23618,
    23629, 23641, 23653, 23665, 23676, 23688, 23700, 23712,
    23723, 23735, 23747, 23758, 23770, 23782, 23794, 23805,
    23817, 23829, 23840, 23852, 23864, 23875, 23887, 23899,
    23911, 23922, 23934, 23946, 23957, 23969, 23981, 23992,
    24004, 24016, 24027, 24039, 24051, 24062, 24074, 24086,
    24097, 24109, 24121, 24132, 24144, 24156, 24167, 24179,
    24191, 24202, 24214, 24225, 24237, 24249, 24260, 24272,
    24284, 24295, 24307, 24318, 24330, 24342, 24353, 24365,
    24376, 24388, 24400, 24411, 24423, 24434, 24446, 24458,
    24469, 24481, 24492, 24504, 24516, 24527, 24539, 24550,
    24562, 24573, 24585, 24597, 24608, 24620, 24631, 24643,
    24654, 24666, 24677, 24689, 24701, 24712, 24724, 24735,
    24747, 24758, 24770, 24781, 24793, 24804, 24816, 24827,
    24839, 24850, 24862, 24874, 24885, 24897, 24908, 24920,
    24931, 24943, 24954, 24966, 24977, 24989, 25000, 25012,
    25023, 25035, 25046, 25057, 25069, 25080, 25092, 25103,
    25115, 25126, 25138, 25149, 25161, 25172, 25184, 25195,
    25207, 25218, 25229, 25241, 25252, 25264, 25275, 25287,
    25298, 25309, 25321, 25332, 25344, 25355, 25367, 25378,
    25389, 25401, 25412, 25424, 25435, 25447, 25458, 25469,
    25481, 25492, 25504, 25515, 25526, 25538, 25549, 25560,
    25572, 25583, 25595, 25606, 25617, 25629, 25640, 25651,
    25663, 25674, 25686, 25697, 25708, 25720, 25731, 25742,
    25754, 25765, 25776, 25788, 25799, 25810, 25822, 25833,
    25844, 25856, 25867, 25878, 25890, 25901, 25912, 25924,
    25935, 25946, 25958, 25969, 25980, 25991, 26003, 26014,
    26025, 26037, 26048, 26059, 26071, 26082, 26093, 26104,
    26116, 26127, 26138, 26150, 26161, 26172, 26183, 26195,
    26206, 26217, 26228, 26240, 26251, 26262, 26273, 26285,
    26296, 26307, 26318, 26330, 26341, 26352, 26363, 26374,
    26386, 26397, 26408, 26419, 26431, 26442, 26453, 26464,
    26475, 26487, 26498, 26509, 26520, 26531, 26543, 26554
};
#endif
