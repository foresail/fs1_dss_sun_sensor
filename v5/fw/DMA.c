#include <msp430.h>
#include "DMA.h"
#include "main.h"
#include "calc.h"

volatile unsigned char DMA_x_flag;
volatile unsigned char DMA_y_flag;
volatile unsigned char dma_timeout = 0;

// DMA interrupt service routine
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
    switch(__even_in_range(DMAIV, DMAIV_DMA2IFG))       //__even_in_range(register, range)
      {
        case DMAIV_NONE: break;                         // Vector 0 - No interrupt
        case DMAIV_DMA0IFG:                             // Vector 2 - DMA channel 0 interrupt
            DMA0CTL &= ~DMAIFG;
            DMA_x_flag = 1;
            break;
        case DMAIV_DMA1IFG:                             // Vector 4 - DMA channel 1 interrupt
            DMA1CTL &= ~DMAIFG;
            DMA_y_flag = 1;
            break;
        case DMAIV_DMA2IFG:                             // Vector 6 - DMA channel 2 interrupt
            //DMA2CTL &= ~DMAIFG;
            //DMA_uart_flag = 1;
            break;
        default:
            break;
    }
}

/*Initialize DMA (Direct Memory Access) controller to move data from sensor to array.*/
void DMA_INIT(){

    DMA0CTL &= ~DMAEN;                                      //Disable DMA controller
    DMA1CTL &= ~DMAEN;                                      //Disable DMA controller
    //DMA2CTL &= ~DMAEN;                                      //Disable DMA controller

    //X and Y data
    DMACTL0 = DMA0TSEL__UCA0RXIFG | DMA1TSEL__UCB0RXIFG0;   // Trigger 14 and 18 on Channel 0/1 corresponds to UCA0RX and UCB0RX

    //DMACTL1 = DMA2TSEL__UCA1RXIFG;                          // Trigger 16 on Channel 2 corresponds to UCA1RX

    DMA0CTL |= DMAEN;                                       //Enable DMA controller
    DMA1CTL |= DMAEN;                                       //Enable DMA controller
    //DMA2CTL |= DMAEN;                                       //Enable DMA controller

    /*DMA channel 0 source address*/
    //DMA0SA = (unsigned short)&UCA0RXBUF;                  // Src = SPI RX (x data), A0
    DMA0SAL = (unsigned short)&UCA0RXBUF;

    /*DMA channel 0 destination address*/
    //DMA0DA = (unsigned short)x_data;                      // Dest single address
    DMA0DAL = (unsigned short)x_data;

    /*DMA channel 0 transfer block size*/
    DMA0SZ = 256;                                           // Block size

    /*Transfer is in repeated single byte*/
    /*DMA channel0 Destination address increment by 1 */
    /*transfer is byte wise   */
    /*DMA channel0 INT disabled     */
    /*DMA channel0 is  disabled      */
    DMA0CTL |= DMADT_4 + DMADSTINCR_3 + DMASBDB + DMALEVEL; // inc src, enable, byte access

    /*DMA channel 1 source address*/
    //DMA1SA = (unsigned short)&UCB0RXBUF;                  // Src = SPI RX (y data), B0
    DMA1SAL = (unsigned short)&UCB0RXBUF;

    /*DMA channel 1 destination address*/
    //DMA1DA = (unsigned short)y_data;                      // Dest single address
    DMA1DAL = (unsigned short)y_data;

    /*DMA channel 1 transfer block size*/
    DMA1SZ = 256;                                           // Block size

    /*Transfer is in repeated single byte*/
    /*DMA channel1 Destination address increment by 1 */
    /*transfer is byte wise   */
    /*DMA channel1 INT disabled     */
    /*DMA channel1 is  disabled      */
    DMA1CTL |= DMADT_4 + DMADSTINCR_3 + DMASBDB + DMALEVEL; // inc src, enable, byte access

    /*DMA channel 2 source address*/
    //DMA2SA = (unsigned short)&UCB0RXBUF;                  // Src = UART RX, A1
    //DMA2SAL = (unsigned short)&UCA1RXBUF;

    /*DMA channel 2 destination address*/
    //DMA2DA = (unsigned short)x_data;                      // Dest single address
    //DMA2DAL = (unsigned short)x_data;

    /*DMA channel 2 transfer block size*/
    //DMA2SZ = 256;                                           // Block size

    /*Transfer is in repeated single byte*/
    /*DMA channel2 Destination address increment by 1 */
    /*transfer is byte wise   */
    /*DMA channel2 INT disabled     */
    /*DMA channel2 is  disabled      */
    //DMA2CTL |= DMADT_4 + DMADSTINCR_3 + DMASBDB + DMALEVEL; // inc src, enable, byte access


    DMA0CTL |= DMAIE;                                   //DMA interrupt enable
    DMA1CTL |= DMAIE;                                   //DMA interrupt enable
    //DMA2CTL |= DMAIE;                                   //DMA interrupt enable

    DMA_x_flag = 0;
    DMA_y_flag = 0;
}

void DMA_DISABLE(void)
{

    // Disable DMA Interrupt
    DMA0CTL &= ~DMAIE;                                   //DMA interrupt disable
    DMA1CTL &= ~DMAIE;                                   //DMA interrupt disable

    // Disable DMA Controller
    DMA0CTL &= ~DMAEN;                                   //Disable DMA controller
    DMA1CTL &= ~DMAEN;                                   //Disable DMA controller
}
