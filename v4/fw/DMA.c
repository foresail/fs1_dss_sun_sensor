#include <msp430fr5739.h>
#include "DMA.h"
#include "main.h"

volatile int DMA_x_flag;
volatile int DMA_y_flag;
volatile int dma_timeout = 0;

// DMA interrupt service routine
#pragma vector = DMA_VECTOR
__interrupt void DMA_ISR(void)
{
    switch(__even_in_range(DMAIV, 4))           //__even_in_range(register, range)
      {
        case 0: break;                          // Vector 0 - No interrupt
        case 2:                                 // Vector 2 - DMA channel 0 interrupt
            DMA0CTL &= ~DMAIFG;
            DMA_y_flag = 1;
            break;
        case 4:                                 // Vector 4 - DMA channel 1 interrupt
            DMA1CTL &= ~DMAIFG;
            DMA_x_flag = 1;
            break;
        default:
            break;
    }
}

/*Initialize DMA (Direct Memory Access) controller to move data from sensor to array.*/
void DMA_INIT(){

    DMA0CTL &= ~DMAEN;                                   //Disable DMA controller
    DMA1CTL &= ~DMAEN;                                   //Disable DMA controller

    //X and Y data
    DMACTL0 = DMA0TSEL_14 | DMA1TSEL_16;                // Trigger 16 and 18 on Channel 0/1 corresponds to UCA0RX and UCA1RX

    DMA0CTL |= DMAEN;                                   //Enable DMA controller
    DMA1CTL |= DMAEN;                                   //Enable DMA controller

    /*DMA channel 0 source address*/
    DMA0SA = (unsigned short)&UCA0RXBUF;                // Src = SPI RX (y data)

    /*DMA channel 0 destination address*/
    DMA0DA = (unsigned short)y_data;                    // Dest single address

    /*DMA channel 0 transfer block size*/
    DMA0SZ = 256;                                      // Block size

    /*Transfer is in repeated single byte*/
    /*DMA channel0 Destination address increment by 1 */
    /*transfer is byte wise   */
    /*DMA channel0 INT disabled     */
    /*DMA channel0 is  disabled      */
    DMA0CTL |= DMADT_4 + DMADSTINCR_3 + DMASBDB + DMALEVEL;// inc src, enable, byte access

    /*DMA channel 1 source address*/
    DMA1SA = (unsigned short)&UCA1RXBUF;                // Src = SPI RX (x data)

    /*DMA channel 1 destination address*/
    DMA1DA = (unsigned short)x_data;                    // Dest single address

    /*DMA channel 1 transfer block size*/
    DMA1SZ = 256;                                      // Block size

    /*Transfer is in repeated single byte*/
    /*DMA channel1 Destination address increment by 1 */
    /*transfer is byte wise   */
    /*DMA channel1 INT disabled     */
    /*DMA channel1 is  disabled      */
    DMA1CTL |= DMADT_4 + DMADSTINCR_3 + DMASBDB + DMALEVEL;// inc src, enable, byte access

    DMA0CTL |= DMAIE;                                   //DMA interrupt enable
    DMA1CTL |= DMAIE;                                   //DMA interrupt enable

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
