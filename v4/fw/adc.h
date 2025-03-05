#ifndef ADC_H_
#define ADC_H_

//Variables
extern volatile int16_t temperature;

//Functions
void ADC_INIT(void);
int read_tempC(void);
static int read_adc_ch(unsigned char ch);

#endif /* ADC_H_ */
