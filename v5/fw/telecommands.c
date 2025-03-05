#include "telecommands.h"

#include <msp430.h>

#include "platform/debug.h"
#include "main.h"
#include "adc.h"
#include "calc.h"

#ifdef DEBUG
#define SAMPLING_LED_ON()  LED2_ON()
#define SAMPLING_LED_OFF() LED2_OFF()
#else
#define SAMPLING_LED_ON()
#define SAMPLING_LED_OFF()
#endif

uint16_t FRAM_VAR filtered_arr[256];


////////////////////////////////////////////////////////////////////////////////
/// Application command handling
////////////////////////////////////////////////////////////////////////////////

static void respond_with_status_code(BusFrame* rsp, uint8_t status_code) {
    rsp->cmd = RSP_STATUS;
    rsp->len = 1;
    rsp->data[0] = status_code;
}

static unsigned char wakeup_sensor(BusFrame *rsp){
    if(sleep_mode){
        wakeup();
        respond_with_status_code(rsp, RSP_STATUS_SLEEP);
    }
    return sleep_mode;
}

void handle_command(const BusFrame* cmd, BusFrame* rsp) {
    // Stop the HB timer during command handling
    //HB_TIMER_DISABLE();

	rsp->dst = cmd->src;

	switch (cmd->cmd) {

	    case CMD_GET_STATUS: {
            /*
             * General status/test command
             */

	        respond_with_status_code(rsp, sleep_mode ? RSP_STATUS_SLEEP : RSP_STATUS_OK);
            break;
        }

        case CMD_GET_RAW: {
            /*
             * DOES NOT SAMPLE SENSOR
             * Get raw current measurements
             */

            // respond with the part number first
            memcpy(rsp->data, cmd->data, sizeof(uint8_t)*1);

            switch (cmd->data[0]){// switch according to part is to be returned
                case 0: {
                    memcpy(rsp->data+1, x_data, sizeof(uint8_t)*128);
                    break;
                }
                case 1: {
                    memcpy(rsp->data+1, x_data + 128, sizeof(uint8_t)*128);
                    break;
                }
                case 2: {
                    memcpy(rsp->data+1, y_data, sizeof(uint8_t)*128);
                    break;
                }
                case 3: {
                    memcpy(rsp->data+1, y_data + 128, sizeof(uint8_t)*128);
                    break;
                }
                default: {
                    respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                    break;
                }
            }

            // Length (x_data/2) = 128
            rsp->cmd = RSP_RAW;
            rsp->len = 128+1;
            break;
        }

        case CMD_GET_POSITION: {
            /*
             * Get position of the light spot
             */

            SAMPLING_LED_ON();
            // Wakeup sensor if it's in sleep mode
            if(wakeup_sensor(rsp)) break;


            if (!SAMPLE_SENSOR()){
                respond_with_status_code(rsp, RSP_STATUS_SAMPLING_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(x_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'x') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(y_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'y') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }


            SAMPLING_LED_OFF();

            rsp->cmd = RSP_POSITION;

            memcpy(rsp->data, &VALUE_X, sizeof(VALUE_X));
            memcpy(rsp->data + sizeof(VALUE_X), &VALUE_Y, sizeof(VALUE_Y));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y), &SNR_X, sizeof(SNR_X));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y) + sizeof(SNR_X), &SNR_Y, sizeof(SNR_Y));

            rsp->len = sizeof(VALUE_X)+sizeof(VALUE_Y)+sizeof(SNR_X)+sizeof(SNR_Y);
            break;
        }

        case CMD_GET_VECTOR: {
            /*
             * This function returns a vector in the format [x_value, y_value, z_value, SNR_X, SNR_Y]
             */

            // Wakeup sensor if it's in sleep mode
            if(wakeup_sensor(rsp)) break;

            if (!SAMPLE_SENSOR()){
                respond_with_status_code(rsp, RSP_STATUS_SAMPLING_ERROR);
                break;
            }

            SAMPLING_LED_ON();
            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(x_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'x') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(y_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'y') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            SAMPLING_LED_OFF();

            rsp->cmd = RSP_VECTOR;

            memcpy(rsp->data, &VALUE_X, sizeof(VALUE_X));
            memcpy(rsp->data + sizeof(VALUE_X), &VALUE_Y, sizeof(VALUE_Y));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y), &VALUE_Z, sizeof(VALUE_Z));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y) + sizeof(VALUE_Z), &SNR_X, sizeof(SNR_X));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y) + sizeof(VALUE_Z) + sizeof(SNR_X), &SNR_Y, sizeof(SNR_Y));

            rsp->len = sizeof(VALUE_X)+sizeof(VALUE_Y)+sizeof(VALUE_Z)+sizeof(SNR_X)+sizeof(SNR_Y);

            break;
        }
#ifdef CALC_ANGLES
        case CMD_GET_ANGLES: {
            /*
             * Get sun angle
             */

            if(sleep_mode){
                wakeup();
            }

            if (!SAMPLE_SENSOR()){
                respond_with_status_code(rsp, RSP_STATUS_SAMPLING_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(x_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'x') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(y_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'y') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            VALUE_X = angle(VALUE_X + X_BIAS);                   // Correct for X_BIAS
            VALUE_Y = angle(VALUE_Y + Y_BIAS);                   // Correct for Y_BIAS

            rsp->cmd = RSP_ANGLES;

            memcpy(rsp->data, &VALUE_X, sizeof(VALUE_X));
            memcpy(rsp->data + sizeof(VALUE_X), &VALUE_Y, sizeof(VALUE_Y));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y), &SNR_X, sizeof(SNR_X));
            memcpy(rsp->data + sizeof(VALUE_X) + sizeof(VALUE_Y) + sizeof(SNR_X), &SNR_Y, sizeof(SNR_Y));

            rsp->len = sizeof(VALUE_X)+sizeof(VALUE_Y)+sizeof(SNR_X)+sizeof(SNR_X);
            break;
        }

        case CMD_GET_ALL: {
            /*
             * Get all the measurement data (mainly for testing purposes)
             */

            rsp->cmd = RSP_ALL;

            if(sleep_mode){
                wakeup();
            }

            // Get temperature
            int temp = read_tempC();
            memcpy(rsp->data, &temp, sizeof(temp));

            // Get angles
            if (!SAMPLE_SENSOR()){
                respond_with_status_code(rsp, RSP_STATUS_SAMPLING_ERROR);
                break;
            }

            uint32_t VALUE_X = 0, VALUE_Y = 0;

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(x_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'x') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            // Rolling filter checks if the sensor is saturated or not. If not perform quadratic middle calculation, if yes estimate with calc_middle
            rolling_filter(y_data, filtered_arr);
            if (quadratic_middle(filtered_arr, 'y') != CALC_OK) {
                respond_with_status_code(rsp, RSP_STATUS_CALC_ERROR);
                break;
            }

            VALUE_X = angle(VALUE_X + X_BIAS);                   // Correct for X_BIAS
            VALUE_Y = angle(VALUE_Y + Y_BIAS);                   // Correct for Y_BIAS


            memcpy(rsp->data + sizeof(temp), &VALUE_X, sizeof(VALUE_X));
            memcpy(rsp->data + sizeof(temp) + sizeof(VALUE_X), &VALUE_Y, sizeof(VALUE_Y));
            memcpy(rsp->data + sizeof(temp) + sizeof(VALUE_X) + sizeof(VALUE_Y), &SNR_X, sizeof(SNR_X));
            memcpy(rsp->data + sizeof(temp) + sizeof(VALUE_X) + sizeof(VALUE_Y) + sizeof(SNR_X), &SNR_Y, sizeof(SNR_Y));

            rsp->len = sizeof(temp)+sizeof(VALUE_X)+sizeof(VALUE_Y)+sizeof(SNR_X)+sizeof(SNR_X);

            // Get raw
            //memcpy(transmit_message + 1, x_data, sizeof(x_data));
            //memcpy(transmit_message + 257, y_data, sizeof(y_data));

            // Set length of message to be transmitted
            //rsp->len = sizeof(x_data) + sizeof(y_data) + 14;
            break;
        }
#endif

        case CMD_GET_TEMPERATURE: {
            /*
             * Return MCU temperature reading
             */

            int16_t temp = read_tempC();

            rsp->cmd = RSP_TEMPERATURE;
            memcpy(rsp->data, &temp, sizeof(temp));

            rsp->len = sizeof(temp);
            break;
        }

        case CMD_GET_CONFIG: {

            switch(cmd->data[0]) {
                case CMD_CONFIG_CALIBRATION: {
                    /*
                     * Get the sensor bias values for the X, Y and Z center position of the light spot
                     * Also get Temp bias
                     */

                    rsp->cmd = RSP_CONFIG;
                    rsp->data[0] = CMD_CONFIG_CALIBRATION;
                    memcpy(rsp->data+1, &X_BIAS, sizeof(X_BIAS));
                    memcpy(rsp->data+1 + sizeof(X_BIAS), &Y_BIAS, sizeof(Y_BIAS));
                    memcpy(rsp->data+1 + sizeof(X_BIAS) + sizeof(Y_BIAS), &VALUE_Z, sizeof(VALUE_Z));
                    memcpy(rsp->data+1 + sizeof(X_BIAS) + sizeof(Y_BIAS) + sizeof(VALUE_Z), &TEMPERATURE_BIAS, sizeof(TEMPERATURE_BIAS));

                    rsp->len = sizeof(X_BIAS)+sizeof(Y_BIAS)+sizeof(VALUE_Z)+sizeof(TEMPERATURE_BIAS)+1;

                    break;
                }

                case CMD_CONFIG_SAT_LEVEL: {
                    /*
                     * Get Sensor saturation level calibration value
                     */

                    rsp->cmd = RSP_CONFIG;
                    rsp->data[0] = CMD_CONFIG_SAT_LEVEL;
                    memcpy(rsp->data+1, &SAT_LEVEL, sizeof(SAT_LEVEL));

                    rsp->len = sizeof(SAT_LEVEL)+1;
                    break;
                }

                case CMD_CONFIG_INT: {
                    /*
                     * Get Sensor integration time variable
                     */

                    rsp->cmd = RSP_CONFIG;
                    rsp->data[0] = CMD_CONFIG_INT;
                    memcpy(rsp->data+1, &INT_TIME, sizeof(INT_TIME));

                    rsp->len = sizeof(INT_TIME)+1;
                    break;
                }

                case CMD_CONFIG_SAMPLING: {
                    /*
                     * Get Sensor sampling time variable
                     */

                    rsp->cmd = RSP_CONFIG;
                    rsp->data[0] = CMD_CONFIG_SAMPLING;
                    memcpy(rsp->data+1, &SAMPLING_TIME, sizeof(SAMPLING_TIME));

                    rsp->len = sizeof(SAMPLING_TIME)+1;
                    break;
                }

                case CMD_CONFIG_GAIN: {
                    /*
                     * Get Sensor GAIN variable
                     */

                    rsp->cmd = RSP_CONFIG;
                    rsp->data[0] = CMD_CONFIG_GAIN;
                    memcpy(rsp->data +1, &GAIN, sizeof(GAIN));

                    rsp->len = sizeof(GAIN)+1;

                    break;
                }
                default:
                    /* Unknown command */
                    respond_with_status_code(rsp, RSP_STATUS_UNKNOWN_COMMAND);
                    break;
            }

            break;
        }

        case CMD_SET_CONFIG: {

            switch(cmd->data[0]) {
                case CMD_CONFIG_CALIBRATION: {
                    /*
                     * Set the sensor bias values for the X and Y center position of the light spot
                     */

                    if (cmd->len != 9){
                        respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                        break;
                    }

                    memcpy(&X_BIAS, cmd->data + 1, sizeof(X_BIAS));
                    memcpy(&Y_BIAS, cmd->data + 3, sizeof(Y_BIAS));
                    memcpy(&VALUE_Z, cmd->data + 5, sizeof(VALUE_Z));
                    memcpy(&TEMPERATURE_BIAS, cmd->data + 7, sizeof(TEMPERATURE_BIAS));

                    respond_with_status_code(rsp,RSP_STATUS_OK);
                    break;
                }

                case CMD_CONFIG_SAT_LEVEL: {
                    /*
                     * Set sensor saturation level calibration value
                     */

                    memcpy(&SAT_LEVEL, cmd->data + 1, sizeof(SAT_LEVEL));

                    respond_with_status_code(rsp,RSP_STATUS_OK);

                    break;
                }

                case CMD_CONFIG_INT: {
                    /*
                     * Set the integration time of the sensor. This is a uint16_t value.
                     */

                    if (cmd->len != 3){
                        respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                        break;
                    }

                    uint16_t temp_int_time;

                    memcpy(&temp_int_time, cmd->data + 1, sizeof(temp_int_time));

                    // Prevent user from setting a too low integration time
                    if (temp_int_time < 3200) {
                        respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                        break;
                    }

                    INT_TIME = temp_int_time;

                    respond_with_status_code(rsp,RSP_STATUS_OK);
                    break;
                }

                case CMD_CONFIG_SAMPLING: {
                    /*
                     * Set the sensor sampling time. Setting a sampling value of less than 3100, use at your own risk. The sampling period should be over 3000.
                     * Anything less than this might result in unpredicted behavior of the DSS.
                     */

                    if (cmd->len != 3){
                        respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                        break;
                    }

                    uint16_t temp_sampling_time;

                    memcpy(&temp_sampling_time, cmd->data + 1, sizeof(temp_sampling_time));

                    // Prevent user from setting a too low sampling time
                    if (temp_sampling_time < 3200) {
                        respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                        break;
                    }

                    SAMPLING_TIME = temp_sampling_time;

                    respond_with_status_code(rsp,RSP_STATUS_OK);
                    break;
                }

                case CMD_CONFIG_GAIN: {
                    /*
                     * Set the profile sensors gain --> High gain = 1 and Low gain = 0
                     */

                    //received_message[2] contains new gain setting (0 or 1)
                    // Set Profile Sensor Gain
                    if ((cmd->data[1] != 0x00) && (cmd->data[1] != 0x01)){
                        respond_with_status_code(rsp, RSP_STATUS_INVALID_PARAM);
                        break;
                    }

                    GAIN = cmd->data[1];
                    ss_gain(GAIN);

                    respond_with_status_code(rsp,RSP_STATUS_OK);
                    break;
                }

                default:
                    /* Unknown command */
                    respond_with_status_code(rsp, RSP_STATUS_UNKNOWN_COMMAND);
                    break;
            }
            break;
        }

        default:
            /* Unknown command */
            respond_with_status_code(rsp, RSP_STATUS_UNKNOWN_COMMAND);
            break;
        }

	// Command handled - Reset flags
	reset_idle_counter();

    // Start the HB timer
    //HB_TIMER_ENABLE();

}
