#ifndef DMA_H_
#define DMA_H_


// Global Flags
extern volatile unsigned char DMA_x_flag;
extern volatile unsigned char DMA_y_flag;
extern volatile unsigned char dma_timeout;


// Function Definitions
void DMA_INIT(void);
void DMA_DISABLE(void);

#endif /* DMA_H_ */
