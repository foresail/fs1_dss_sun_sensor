#include <msp430.h>
#include <msp430fr5739.h>
#include "main.h"


void SPI_INIT() {

    //UCA0CTLW0 register control
    UCA0CTLW0 |= UCSWRST;           //Disable eUSCI module (SPI)
    UCA0CTLW0 |= UCMSB + UCSYNC;    //Enable slave in 3 pin SPI
    UCA0CTLW0 |= UCMSB;             //Set Most significant bit first
    UCA0CTLW0 |= UCCKPL;            //Clock polarity select (inactive high)
    UCA0CTLW0 |= UCCKPH;            //Read data on falling edge

    //Set P1.5 to SPI_A0 clock
    P1SEL1 |= BIT5;
    P1SEL0 &= ~BIT5;

    //Set P2.0 to SPI_A0 SIMO
    P2SEL1 |= BIT0;
    P2SEL0 &= ~BIT0;

    //UCA1CTLW0 register control
    UCA1CTLW0 |= UCSWRST;           //Disable eUSCI module (SPI)
    UCA1CTLW0 |= UCMSB + UCSYNC;    //Enable slave in 3 pin SPI
    UCA1CTLW0 |= UCMSB;             //Set Most significant bit first
    UCA1CTLW0 |= UCCKPL;            //Clock polarity select (inactive high)
    UCA1CTLW0 |= UCCKPH;            //Read data on falling edge

    //Set P2.4 to SPI_A1 clock
    P2SEL1 |= BIT4;
    P2SEL0 &= ~BIT4;

    //Set P2.5 to SPI_A1 SIMO
    P2SEL1 |= BIT5;
    P2SEL0 &= ~BIT5;

    //Enable eUSCI module (SPI)
    UCA0CTLW0 &= ~UCSWRST;          //Enable eUSCI module (SPI) (X-axis)
    UCA1CTLW0 &= ~UCSWRST;          //Enable eUSCI module (SPI) (Y-axis)
}

void SPI_RESET() {
    UCA0CTLW0 |= UCSWRST;           //Disable eUSCI module (SPI)
    UCA1CTLW0 |= UCSWRST;           //Disable eUSCI module (SPI)
    UCA0CTLW0 &= ~UCSWRST;          //Enable eUSCI module (SPI) (X-axis)
    UCA1CTLW0 &= ~UCSWRST;          //Enable eUSCI module (SPI) (Y-axis)
}

void SPI_DISABLE(void)
{
    UCA0CTLW0 |= UCSWRST;           //Disable eUSCI module (SPI)
    UCA1CTLW0 |= UCSWRST;           //Disable eUSCI module (SPI)
}
