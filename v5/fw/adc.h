#ifndef ADC_H_
#define ADC_H_

//Functions
void ADC_INIT(void);
int16_t read_tempC(void);
static int read_adc_ch(unsigned char ch);

#endif /* ADC_H_ */
