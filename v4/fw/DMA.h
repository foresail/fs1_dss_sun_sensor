#ifndef DMA_H_
#define DMA_H_


// Global Flags
extern volatile int DMA_x_flag;
extern volatile int DMA_y_flag;
extern volatile int dma_timeout;


// Function Definitions
void DMA_INIT(void);
void DMA_DISABLE(void);

#endif /* DMA_H_ */
