#include <msp430.h>
#include <stdint.h>

#include "main.h"

#include "telecommands.h"


#include "SPI.h"
#include "DMA.h"
#include "adc.h"

#include "calc.h"

static volatile int interrupt_pending = 0;

////////////////////////////////////////////////////////////////////////////////
/// Platform bus code
////////////////////////////////////////////////////////////////////////////////

#define RS485_PRI_DIR_TX() PJOUT |= BIT3
#define RS485_PRI_DIR_RX() PJOUT &= ~BIT3

typedef struct {
	int active_bus;

	const uint8_t* tx_buf;
	size_t tx_idx, tx_len;

	int slave_rxed;
} BusDriver;

#define BUS_ID_PRIMARY 0

BusFrame* bus_slave_receive(BusHandle* self) {
	BusDriver* driver = (BusDriver*)self->driver;
	if (driver->slave_rxed) {
		driver->slave_rxed = 0;
		return &self->frame_rx;
	}
	return NULL;
}

void bus_slave_send(BusHandle* self, BusFrame* rsp) {
	BusDriver* driver = (BusDriver*)self->driver;

	// Make sure interrupts are disabled. Technically they should
	// be by this point, because bus_slave_send() is called _ONLY_
	// after the slave has received and handled a frame
	UCA1IE = 0;

	// Prepare for transmitting
	const BusFrame* tx_frame = bus_prepare_tx_frame(rsp);
	driver->tx_buf = tx_frame->buf;
	driver->tx_len = tx_frame->len + BUS_OVERHEAD;
	driver->tx_idx = 0;

	// Begin transfer
	// NOTE: TX empty buffer flag needs to be set manually
	if (driver->active_bus == BUS_ID_PRIMARY) {
		RS485_PRI_DIR_TX();
		UCA1IFG = UCTXIFG;
		UCA1IE = UCTXIE;
	}
}

static BusHandle bus_adcs;

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_B0_VECTOR
__interrupt void bus_rx_timeout_irq()
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_B0_VECTOR))) bus_rx_timeout_irq()
#else
#error Compiler not supported!
#endif
{
	TB2CTL &= ~MC__UPDOWN; // Put into stop mode

	bus_adcs.receive_timeouts++;

	bus_reset_rx(&bus_adcs);

	// Enable RX on bus
	RS485_PRI_DIR_RX();
	UCA1IE = UCRXIE;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void bus_primary_irq()
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) bus_primary_irq()
#else
#error Compiler not supported!
#endif
{
	BusDriver* driver = (BusDriver*)bus_adcs.driver;

    switch(__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG)) {
    	case USCI_NONE: break;
		case USCI_UART_UCRXIFG: { // Receive buffer full
			driver->active_bus = BUS_ID_PRIMARY;

			// Enable receiver timeout timer
			TB2CTL |= MC__UP | TACLR;
			TB2CCTL0 = CCIE;

			if (bus_handle_rx_byte(&bus_adcs, UCA1RXBUF)) {
				// Disable timer
				TB2CTL &= ~MC__UPDOWN;
				TB2CCTL0 = 0;

				// After receiving successfully a frame appointed to our device,
				// disable receiver to make sure rx buffer won't get corrupted.
				// Next function to be called is bus_slave_send().
				UCA1IE = 0;

				// Wake up the main thread.
				driver->slave_rxed = 1;
				interrupt_pending = 1;
				__bic_SR_register_on_exit(LPM0_bits);
			}
		} break;
		case USCI_UART_UCTXIFG: { // Transmit buffer empty
			if (driver->tx_idx < driver->tx_len) {
				UCA1TXBUF = driver->tx_buf[driver->tx_idx++];
			} else {
				UCA1IFG &= ~UCTXCPTIE; // NOTE: must be cleared or ISR will trigger on the previous byte
				UCA1IE = UCTXCPTIE;
			}
		} break;
		case USCI_UART_UCSTTIFG: { // Start bit received
			UCA1IFG &= ~UCSTTIFG;
		} break;
		case USCI_UART_UCTXCPTIFG: { // Transmit complete
			UCA1IE = 0;
			UCA1IFG &= ~UCTXCPTIE;

			// Go to receiver mode on bus
			RS485_PRI_DIR_RX();
			UCA1IE = UCRXIE;
		} break;
		default: break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Platform initialization and main loop
////////////////////////////////////////////////////////////////////////////////

volatile uint16_t sys_ticks = 0;

/*
 * Watchdog time configuration
 *
 * Select master clock as the clock source WDT is cleared only in the heartbeat interrupt,
 * so the reset time must be greater than maximum heartbeat period.
 * VLOCLK_frq = 10kHz
 * WDTSSEL__ACLK = 12MHz
 * Reset Time [sec] = (2^X) / CLK_SOURCE
 * See WDTIS for values of X
 * WDTIS__128M (00:00:11 at 12 MHz)
 * WDTIS__8192K (0.699 s at 12 MHz)
 */


#ifdef USE_WDT
#define RESET_WDT() (WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS__8192K)
#else
#define RESET_WDT() (WDTCTL = WDTPW + WDTHOLD) // Disabled!
#endif

unsigned int idle_counter = ~0;
void reset_idle_counter(){
    idle_counter = 0;
}

// Sleep mode indicator flag. Sleep Mode - 0, Enabled - 1
uint8_t sleep_mode = 1;
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


void INTEGRATION_TIMER_INIT(){
    //ACLK set to 12MHz
    //ACLK, UP mode, divide by 1 and 2 --> timer frequency = 12MHz/2 = 6MHz, counter up mode
    TA0CTL = ID__1 + TASSEL__ACLK + MC__UP;
    TA0CCR0 = INT_TIME;                                 // Set the timer/integration time
    TA0EX0 = 0x01;                                      //Divide by 2
    TA0CCTL0 = CCIE;                                    //TACCR1 interrupt enable
    TA0CTL |= TACLR;
}

void HB_TIMER_INIT(){
    /*
     * Configure TimerB0
     *
     * SMCLK = DCO / 1
     * Timer frequency(Hz) = SMCLK / (TB0CCR0 * 8 * 8)
     * TB0CCR0 = SMCLK / (timer_frequency(Hz)*8*8)
     */

    TB0CTL = MC__UP + TBSSEL__SMCLK + ID__8;    // SMCLK, UP mode, divide by 8
    TB0CCR0 = 3000;                             // Count up to this value
    TB0EX0 = TBIDEX__8;                         // Divide by 8
    TB0CCTL0 = CCIE;                            // TBCCR0 interrupt enabled
    TB0CTL |= TBCLR;                            // Clear interrupt
    //CSCTL4 &= ~SMCLKOFF;
    TB0R = 0;
}


/*
 * Timer B0 interrupt (triggering every 16ms) - Used to wakeup the processor and check any new events
 */
#pragma vector=TIMER0_B0_VECTOR
__interrupt void Timer_B0(void)
{
    LED1_TOGGLE();
    sys_ticks++; // sys_tick every 16 ms
    // Wake up the main loop
    //__bic_SR_register_on_exit(LPM0_bits + SMCLKOFF);
    RESET_WDT();
    __bic_SR_register_on_exit(LPM0_bits);
}

static void platform_init() {
	__disable_interrupt();

	WDTCTL = WDTPW | WDTHOLD;

	// Pin configuration
	{
        PADIR=0xffff; PAOUT=0x0000; // Ports 1 and 2
        PBDIR=0xffff; PBOUT=0x0000; // Ports 3 and 4

        //LED 1
        P1SELC &= ~BIT3;                //Select pin 1.3 to I/O
        P1DIR |= BIT3;                  //Select pin 1.3 to output
        P1OUT |= BIT3;

        //LED 2
        P1SELC &= ~BIT4;                //Select pin 1.4 to I/O
        P1DIR |= BIT4;                  //Select pin 1.4 to output
        P1OUT |= BIT4;

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

        //Start 5V Boost enable (P3.6)
        P3SELC &= ~BIT6;                //Select pin 3.6 to I/O
        P3DIR |= BIT6;                  //Select pin 3.6 to output
        P3OUT &= ~BIT6;                 //Set Boost off

        // Gain selection (PJ.1)
        PJSELC &= ~BIT1;                //Select pin J.0 to I/O
        PJDIR |= BIT1;                  //Select pin J.0 to output
        PJOUT |= BIT1;                 //Set gain output to low (default)

        // P2.5 = TXD (SEL1.5=1, SEL0.5=0)
        // P2.6 = RXD (SEL1.6=1, SEL0.6=0)
		P2SEL1 |= BIT5 | BIT6;
        P2SEL0 &= ~(BIT5 | BIT6);
	
        // Initialize the RS485 DE/RE pin
        PJDIR |= BIT3;
        PJOUT &= ~BIT3;

		// Disable the GPIO power-on default high-impedance mode to activate
		// previously configured port settings
		PM5CTL0 &= ~LOCKLPM5;
	}

	// Clock configuration
	{
		// Configure one FRAM waitstate as required by the device datasheet for MCLK
		// operation beyond 8MHz _before_ configuring the clock system.
		//FRCTL0 = FRCTLPW | NWAITS_1;

		CSCTL0_H = CSKEY_H; // Unlock

		CSCTL0 = CSKEY;                                        // Password to enable clock manipulation

        //Configure internal crystal (24MHz) as main clock
        CSCTL1 |= DCOFSEL_3 | DCORSEL;                          // Set max. DCO setting 24MHz
        CSCTL2 = SELA__DCOCLK | SELS__DCOCLK | SELM__DCOCLK;    // set ACLK = DC0; MCLK = SMCLK = DCO
        CSCTL3 = DIVA__2 | DIVS__2 | DIVM__1;                   // set all dividers ACLK/2=12MHz, SMCLK/2=12MHz, MCLK/1=24MHz

		CSCTL0_H = 0; // Lock
	}

	// UART reception timeout timer configuration
	{
		TB2CTL = TBSSEL__SMCLK | ID__8 | MC__UP;
		TB2CCTL0 = 0;
		TB2R = 0;
		TB2CCR0 = 399; // 0.2 msec
	}

	// UART configuration
	{
		// UCA1 (primary)
        UCA1CTLW0 = UCSWRST;                    // Set the state machine to reset
        UCA1CTLW0 |= UCSSEL__SMCLK;             // SMCLK (12MHz)

        // 12 MHz 115200 (see msp430fr5739 User's Guide page 491)
        UCA1MCTLW |= UCOS16;                    // Oversampling enabled

        // Proper UART 115200 baud rate
        //UCA1BRW = 6;                            // Set the baud rate to 115200 (12MHz)
        //UCA1MCTLW |= UCBRF_8 | 0x2000;          // Modulation UCBRSx=0x20, UCBRFx=8

        // Baud rate of OBC is 3% (111607) slower AND MESSES UP EVERYTHING
        UCA1BRW = 6;                            // Set the baud rate to 115200 (12MHz)
        UCA1MCTLW |= UCBRF_10 | 0x6B00;          // Modulation UCBRSx=0x6B, UCBRFx=10

        UCA1CTLW0 &= ~UCSWRST;                  // Release the reset
        UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt

		// Enable RX
		RS485_PRI_DIR_RX();
		UCA1IE = UCRXIE;
	}

    // Initialize WDT
    RESET_WDT();

    /* Basic initalization */
    ADC_INIT();
    SPI_INIT();
    DMA_INIT();                                 //Initialize DMA for SPI data transfer
    INTEGRATION_TIMER_INIT();
    HB_TIMER_INIT();

    __bis_SR_register(GIE); //Enable interrupts
    __no_operation();       // First chance for the interrupt to fire!

    //Boost converter off
    BOOST_DISABLE();

    // Set Sun Sensor Gain to low
    ss_gain(GAIN);

    // Enable the sensor start signal
    ST_SIGNAL_ENABLE();

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}



/*
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    Main Loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

static void platform_loop() {
	BusDriver bus_driver = { 0 };
	bus_adcs.driver = &bus_driver;

	for (;;) {

        RESET_WDT();

		// Make sure that all interrupts are serviced before going to sleep
		__disable_interrupt();
		if (!interrupt_pending) {
			__bis_SR_register(LPM0_bits | GIE);
		}
		interrupt_pending = 0;
		__enable_interrupt();

		// Update slave bus
		{
			BusFrame* cmd = bus_slave_receive(&bus_adcs);
			if (cmd != NULL) {
				BusFrame* rsp = bus_get_tx_frame(&bus_adcs);
				handle_command(cmd, rsp);
				bus_slave_send(&bus_adcs, rsp);
			}
		}

        TB0CTL |= TBCLR; // reset hb timer, but is it needed????????????????

        // Processor wakes up every 16ms --> Goes to sleep after 16ms*250 = 4s
        if (idle_counter > 250 && !sleep_mode) {
            // Goto "deepsleep" if UART is not actively used
            sleep();
        }
        else {
            idle_counter++;
            // Resets itself after 16ms*1250 = 20s
            if (idle_counter >= 1250) {

                // Trigger Power-On-Reset (POR) after ~20 seconds of idling
                PMMCTL0 |= PMMSWPOR;
            }
        }
	}
}

int main() {
    platform_init();
    platform_loop();

    return 0;
}
