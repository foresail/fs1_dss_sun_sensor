#include <msp430fr5739.h>
#include <string.h>
#include "i2c.h"
#include "main.h"
#include "adc.h"


/* Default slave I2C Address: */
static const char FRAM_VAR I2C_ADDRESS = 0x49;

uint16_t FRAM_VAR filtered_arr[256];
uint16_t x, y;

volatile int new_message = 0;

unsigned char FRAM_VAR received_message[BUFFER_LENGTH];
unsigned char FRAM_VAR transmit_message[BUFFER_LENGTH];
unsigned int receive_len, transmit_len, transmit_idx;


/*
 * Enhanced Universal Serial Communication Interface (eUSCI) � I2C Mode
 * http://www.ti.com/lit/ug/slau425f/slau425f.pdf
 */

void I2C_INIT() {

    P1SEL1 |= BIT6 | BIT7;                  // Pins for I2C
    P1SEL0 &= ~BIT6 & ~BIT7;

#if 1
    // Internal pull-ups for testing
    P1REN |= BIT2 | BIT3;
    P1OUT |= BIT2 | BIT3;
#endif

    // eUSCI configuration for I2C
    UCB0CTLW0 |= UCSWRST;                   // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 + UCSYNC;         // I2C slave mode, sync mode, SMCLK
    UCB0I2COA0 = I2C_ADDRESS + UCOAEN;      // Own address + enable
    UCB0CTLW1 |= UCCLTO_3;                  // Clock low timeout ?ms
    UCB0CTLW0 &= ~UCSWRST;                  // Clear reset register

    // Enable transmit and stop interrupts for slave
    UCB0IE = UCALIE | UCRXIE0 | UCTXIE0 | UCSTTIE | UCSTPIE; // | UCCLTOIE;

    receive_len = 0;
    transmit_len = 0;
    transmit_idx = 0;
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void)
{
    //P1OUT ^= BIT4;
    switch (__even_in_range(UCB0IV, 0x1E))
    {
    case 0x00: break;                   // Vector 0: No interrupts break;
    case 0x02:                          // Vector 2: ALIFG aka arbitration lost
        break;
    case 0x04: break;                   // Vector 4: NACKIFG aka not-acknowledgment
    case 0x06:                          // Vector 6: STTIFG aka Start


        if (UCB0CTLW0 & UCTR) { // Our transmission
            if (transmit_len) {
                UCB0TXBUF = transmit_message[0];
                transmit_idx = 1;
            }
            else {
                transmit_idx = 0;
                //UCB0CTLW0 |= UCTXNACK;
                UCB0TXBUF = 0xFF;
            }
        }
        else { // Receiving starts
            new_message = 0;
            receive_len = 0;
        }

        break;

    case 0x08:                          // Vector 8: STPIFG aka stop

        if (UCB0CTLW0 & UCTR) { /* Transmitting stopped */
            // Mark that the write packet has been sent (maybe unnecessary)
            transmit_len = 0;
        }
        else { /* Receiving stopped */
            // Wake up the main program to process the message
            new_message = 1;
            LPM0_EXIT;          // Exit Low power mode 0
            //__bic_SR_register_on_exit(LPM0_bits);
        }

        UCB0IFG &= ~UCSTPIFG; // Clear interrupt
        break;

    case 0x0a: break;                   // Vector 10: RXIFG3
    case 0x0c: break;                   // Vector 14: TXIFG3
    case 0x0e: break;                   // Vector 16: RXIFG2
    case 0x10: break;                   // Vector 18: TXIFG2
    case 0x12: break;                   // Vector 20: RXIFG1
    case 0x14: break;                   // Vector 22: TXIFG1
    case 0x16:                          // Vector 24: RXIFG0 aka Receive (slave 0)

        if (receive_len < BUFFER_LENGTH)
            received_message[receive_len++] = UCB0RXBUF;
        else {
            /* Master tries to write too much. Send NACK! */
            UCB0CTLW0 |= UCTXNACK;
            UCB0IFG &= ~UCRXIFG0;
        }
        break;

    case 0x18:                          // Vector 26: TXIFG0 aka Transmit (slave 0)

        if (transmit_idx < transmit_len) {
            UCB0TXBUF = transmit_message[transmit_idx++];
        }
        else {
            /* Master tries and we have no data! Send dummy bytes because NACKing is not possible... */
            UCB0TXBUF = 0xFF;
            UCB0IFG &= ~UCTXIFG0;
        }
        break;

    case 0x1a: break;                   // Vector 28: BCNTIFG aka byte counter interrupt
    case 0x1c:                          // Vector 30: Clock Low Timeout

        // We should never get here!
        // But if nothing is done here the whole bus will get stuck!

        UCB0IFG &= ~UCCLTOIFG;
        UCB0CTLW0 |= UCTXNACK;
        UCB0TXBUF = 0xFF;

        break;

    case 0x1e: break;                    // Vector 32: 9th bit break;
    default: break;
    }
}


inline void set_response(unsigned char status) {
    transmit_message[0] = status;
    transmit_len = 1;
}

// Function used to return NULL result
void set_null_response(unsigned char command, uint8_t len)
{
    // Set raw_data to zero
    memset(x_data, 0, 256);
    memset(y_data, 0, 256);

    transmit_message[0] = command;

    memset(transmit_message + 1, 0, len);
    transmit_len = len;
}


void handle_command()
{
    if (receive_len == 0)
        return;

    switch (received_message[0]) {

        case CMD_STATUS: {
            /*
             * General status/test command
             */

            set_response(RESPONSE_OK);
            break;
        }

        case CMD_GET_RAW: {
            /*
             * Get raw measurements
             */

            // Get which part is to be returned
            uint8_t part = received_message[1];
            transmit_message[0] = CMD_GET_RAW;

            if (part == 0) {
                memcpy(transmit_message + 1, x_data, sizeof(uint8_t)*128);
            }
            else if (part == 1) {
                memcpy(transmit_message + 1, x_data + 129, sizeof(uint8_t)*128);
            }
            else if (part == 2) {
                memcpy(transmit_message + 1, y_data, sizeof(uint8_t)*128);
            }
            else if (part == 3) {
                memcpy(transmit_message + 1, y_data + 129, sizeof(uint8_t)*128);
            }
            else {
                set_response(RESPONSE_INVALID_PARAM);
                break;
            }

            // Length (cmd+data) = 1+128
            transmit_len = 129;
            break;
        }

        case CMD_GET_POSITION: {
            /*
             * Get the position of the light spot
             */

        	// Wakeup sensor if it's in sleep mode
            if(sleep_mode){
                wakeup();
                set_null_response(CMD_GET_POSITION, 9);
                break;
            }

            if (!SAMPLE_SENSOR()){
                set_response(RESPONSE_SAMPLING_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(x_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'x') != CALC_OK) {
                set_null_response(CMD_GET_POSITION, 9);
                break;
            }

            rolling_filter(y_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'y') != CALC_OK) {
                set_null_response(CMD_GET_POSITION, 9);
                break;
            }

            transmit_message[0] = CMD_GET_POSITION;

            I2C_16(VALUE_X, 1);
            I2C_16(VALUE_Y, 3);
            I2C_16(SNR_X, 5);
            I2C_16(SNR_Y, 7);

            transmit_len = 9;
            break;
        }

        case CMD_GET_VECTOR: {
            /*
             * This function returns a vector in the format [x_value, y_value, z_value, SNR_X, SNR_Y]
             */

            if(sleep_mode){
                wakeup();
                set_null_response(CMD_GET_VECTOR, 11);
                break;
            }

            if (!SAMPLE_SENSOR()){
                set_response(RESPONSE_SAMPLING_ERROR);
                break;
            }

#ifdef DEBUG
            LED2_ON();
#endif

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(x_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'x') != CALC_OK) {
                set_null_response(CMD_GET_VECTOR, 11);
                break;
            }

            rolling_filter(y_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'y') != CALC_OK) {
                set_null_response(CMD_GET_VECTOR, 11);
                break;
            }

            transmit_message[0] = CMD_GET_VECTOR;

            I2C_16(VALUE_X, 1);
            I2C_16(VALUE_Y, 3);
            I2C_16(VALUE_Z, 5);
            I2C_16(SNR_X, 7);
            I2C_16(SNR_Y, 9);

            transmit_len = 11;

#ifdef DEBUG
            LED2_OFF();
#endif
            break;
        }
#ifdef DEBUG
        case CMD_GET_ANGLES: {
            /*
             * Get sun angle
             */

            if(sleep_mode){
                wakeup();
            }

            if (!SAMPLE_SENSOR()){
                set_response(RESPONSE_SAMPLING_ERROR);
                break;
            }

            uint32_t VALUE_X = 0, VALUE_Y = 0;

            rolling_filter(x_data, filtered_arr);
            VALUE_X = quadratic_middle(filtered_arr, 'x');

            if (VALUE_X == 0){
                set_response(RESPONSE_DIVISION_ZERO);
                break;
            }

            VALUE_X = angle(VALUE_X + X_BIAS);                   // Correct for X_BIAS

            rolling_filter(y_data, filtered_arr);
            VALUE_Y = quadratic_middle(filtered_arr, 'y');

            if (VALUE_Y == 0){
                set_response(RESPONSE_DIVISION_ZERO);
                break;
            }

            VALUE_Y = angle(VALUE_Y + Y_BIAS);                   // Correct for Y_BIAS

            transmit_message[0] = CMD_GET_ANGLES;

            I2C_16(VALUE_X, 1);                                 // Transmit angle
            I2C_16(VALUE_Y, 5);                                 // Transmit angle

            I2C_16(SNR_X, 3);                                   // Transmit SNR
            I2C_16(SNR_Y, 7);                                   // Transmit SNR

            transmit_len = 9;
            break;
        }

        case CMD_GET_ALL: {
            /*
             * Get all the measurement data (mainly for testing purposes)
             */

            transmit_message[0] = CMD_GET_ALL;

            if(sleep_mode){
                wakeup();
            }

            // Get temperature
            int temp = read_tempC();
            transmit_message[0] = CMD_GET_TEMPERATURE;
            I2C_16(temp, 1);

            // Get angles
            if (!SAMPLE_SENSOR()){
                set_response(RESPONSE_SAMPLING_ERROR);
                break;
            }

            uint32_t VALUE_X = 0, VALUE_Y = 0;

            rolling_filter(x_data, filtered_arr);
            VALUE_X = quadratic_middle(filtered_arr, 'x');
            VALUE_X = angle(VALUE_X + X_BIAS);                   // Correct for X_BIAS

            rolling_filter(y_data, filtered_arr);
            VALUE_Y = quadratic_middle(filtered_arr, 'y');
            VALUE_Y = angle(VALUE_Y - Y_BIAS);                   // Correct for Y_BIAS

            transmit_message[0] = CMD_GET_ANGLES;

            I2C_16(VALUE_X, 1);                                 // Transmit angle
            I2C_16(SNR_X, 3);                                   // Transmit max value

            I2C_16(VALUE_Y, 5);                                 // Transmit angle
            I2C_16(SNR_Y, 7);                                   // Transmit max value

            // Get raw
            memcpy(transmit_message + 1, x_data, sizeof(x_data));
            memcpy(transmit_message + 257, y_data, sizeof(y_data));

            // Set length of message to be transmitted
            transmit_len = sizeof(x_data) + sizeof(y_data) + 14;
            break;
        }
#endif

        case CMD_GET_TEMPERATURE: {
            /*
             * Return MCU temperature reading
             */

            int temp = read_tempC();

            transmit_message[0] = CMD_GET_TEMPERATURE;
            I2C_16(temp, 1);

            transmit_len = 3;
            break;
        }

        case CMD_GET_CONFIG: {

            switch(received_message[1]) {
                case CMD_GET_BIAS: {
                    /*
                     * Get the sensor bias values for the X and Y center position of the light spot
                     */

                    transmit_message[0] = CMD_GET_BIAS;
                    I2C_16(X_BIAS, 1);
                    I2C_16(Y_BIAS, 3);
                    I2C_16(VALUE_Z, 5);

                    transmit_len = 7;
                    break;
                }

                case CMD_GET_SAT_LEVEL: {
                    /*
                     * Get Sensor saturation level calibration value
                     */

                    transmit_message[0] = CMD_GET_SAT_LEVEL;
                    I2C_16(SAT_LEVEL, 1);

                    transmit_len = 3;
                    break;
                }

                case CMD_GET_INT: {
                    /*
                     * Get Sensor integration time variable
                     */

                    transmit_message[0] = CMD_GET_INT;
                    I2C_16(INT_TIME, 1);

                    transmit_len = 3;
                    break;
                }

                case CMD_GET_SAMPLING: {
                    /*
                     * Get Sensor sampling time variable
                     */

                    transmit_message[0] = CMD_GET_SAMPLING;
                    I2C_16(SAMPLING_TIME, 1);

                    transmit_len = 3;
                    break;
                }

                case CMD_GET_GAIN: {
                    /*
                     * Get Sensor GAIN variable
                     */

                	transmit_message[0] = CMD_GET_GAIN;
                	transmit_message[1] = 0;
                	transmit_message[2] = GAIN;

                	transmit_len = 3;
                    break;
                }
                default:
                    /* Unknown command */
                    set_response(RESPONSE_NO_COMMAND);
                    break;
            }

            break;
        }

        case CMD_SET_CONFIG: {

            switch(received_message[1]) {
                case CMD_SET_BIAS: {
                    /*
                     * Set the sensor bias values for the X and Y center position of the light spot
                     */

                    if (receive_len != 8){
                        set_response(RESPONSE_INVALID_PARAM);
                        break;
                    }

                    memcpy(&X_BIAS, received_message + 2, 2);
                    memcpy(&Y_BIAS, received_message + 4, 2);
                    memcpy(&VALUE_Z, received_message + 6, 2);

                    transmit_message[0] = CMD_SET_BIAS;

                    transmit_len = 1;
                    break;
                }

                case CMD_SET_SAT_LEVEL: {
                    /*
                     * Set sensor saturation level calibration value
                     */

                	memcpy(&SAT_LEVEL, received_message + 2, 2);

                    transmit_message[0] = CMD_SET_SAT_LEVEL;
                    transmit_len = 1;

                    break;
                }

                case CMD_SET_INT: {
                    /*
                     * Set the integration time of the sensor. This is a uint16_t value.
                     */

                    if (receive_len != 4){
                        set_response(RESPONSE_INVALID_PARAM);
                        break;
                    }

                    uint16_t temp_int_time;

                    memcpy(&temp_int_time, received_message + 2, 2);

                    // Prevent user from setting a too low integration time
                    if (temp_int_time < 3200) {
                        set_response(RESPONSE_INVALID_PARAM);
                        break;
                    }

                    INT_TIME = temp_int_time;

                    transmit_message[0] = CMD_SET_INT;

                    transmit_len = 1;
                    break;
                }

                case CMD_SET_SAMPLING: {
                    /*
                     * Set the sensor sampling time. Setting a sampling value of less than 3100, use at your own risk. The sampling period should be over 3000.
                     * Anything less than this might result in unpredicted behavior of the DSS.
                     */

                    if (receive_len != 4){
                        set_response(RESPONSE_INVALID_PARAM);
                        break;
                    }

                    uint16_t temp_sampling_time;

                    memcpy(&temp_sampling_time, received_message + 2, 2);

                    // Prevent user from setting a too low sampling time
                    if (temp_sampling_time < 3200) {
                    	set_response(RESPONSE_INVALID_PARAM);
                    	break;
                    }

                    SAMPLING_TIME = temp_sampling_time;

                    transmit_message[0] = CMD_SET_SAMPLING;

                    transmit_len = 1;
                    break;
                }

                case CMD_SET_GAIN: {
                    /*
                     * Set the profile sensors gain --> High gain = 1 and Low gain = 0
                     */

                    //received_message[2] contains new gain setting (0 or 1)
                    uint8_t value = received_message[2];

                    // Set Profile Sensor Gain
                    if ((value != 0x00) && (value != 0x01)){
                    	set_response(RESPONSE_INVALID_PARAM);
                    	break;
                    }

                    GAIN = value;
                    ss_gain(GAIN);

                    transmit_message[0] = CMD_SET_GAIN;

                    transmit_len = 1;
                    break;
                }

                default:
                    /* Unknown command */
                    set_response(RESPONSE_NO_COMMAND);
                    break;
            }
            break;
        }

        default:
            /* Unknown command */
            set_response(RESPONSE_NO_COMMAND);
            break;
        }
        receive_len = 0;
}

// Function sends a 32bit number via I2C
void I2C_32(uint32_t message, int start)
{
    uint8_t x1, x2, x3, x4;

    x1 = (message & 0xff000000UL) >> 24;
    x2 = (message & 0x00ff0000UL) >> 16;
    x3 = (message & 0x0000ff00UL) >>  8;
    x4 = (message & 0x000000ffUL)      ;

    transmit_message[start] = x1;
    transmit_message[start+1] = x2;
    transmit_message[start+2] = x3;
    transmit_message[start+3] = x4;
}

void I2C_16(uint16_t message, int start)
{
    uint8_t x1, x2;

    x1 = message >>  8;
    x2 = message & 0xff;

    transmit_message[start] = x1;
    transmit_message[start+1] = x2;
}

