#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>
#include "main.h"


/* Command codes: */

// Get commands
#define CMD_STATUS          0x01
#define CMD_GET_RAW         0x02
#define CMD_GET_POSITION    0x03
#define CMD_GET_VECTOR      0x04
#define CMD_GET_ALL         0x05
#define CMD_GET_GAIN        0x06
#define CMD_GET_BIAS        0x07
#define CMD_GET_TEMPERATURE 0x08
#define CMD_GET_SAT_LEVEL   0x09
#define CMD_GET_INT         0x0A
#define CMD_GET_SAMPLING    0x0B

#ifdef DEBUG
#define CMD_GET_ANGLES      0x0C
#endif

// Set Commands
#define CMD_SET_GAIN        0x20
#define CMD_SET_BIAS        0x21
#define CMD_SET_SAT_LEVEL   0x22
#define CMD_SET_INT         0x23
#define CMD_SET_SAMPLING    0x24

// GET/SET Config commands
#define CMD_GET_CONFIG      0x30
#define CMD_SET_CONFIG      0x31

/* Response codes: */
#define RESPONSE_OK                 0xF0
#define RESPONSE_SLEEP              0xF1
#define RESPONSE_SAMPLING_ERROR     0xF2
#define RESPONSE_DIVISION_ZERO      0xF3
#define RESPONSE_NOT_ODD            0xF4
#define RESPONSE_NO_COMMAND         0xFD
#define RESPONSE_INVALID_PARAM      0xFE
#define RESPONSE_ERROR              0xFF


// Max I2C packet length (Must me a bit longer than telemetry frame!)
#define BUFFER_LENGTH   513


// I2C bus
extern volatile int new_message; // Flag to indicate a new received command
extern unsigned char FRAM_VAR received_message[BUFFER_LENGTH];
extern unsigned char FRAM_VAR transmit_message[BUFFER_LENGTH];
extern unsigned int receive_len, transmit_len, transmit_idx;


/*
 * Initialize I2C slave driver
 */
void I2C_INIT();

/*
 * Handle received command
 */
void handle_command();

// Functions
void I2C_32(uint32_t message, int start);
void I2C_16(uint16_t message, int start);


#endif /* __I2C_H__*/
