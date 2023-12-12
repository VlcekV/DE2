#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

// Define the ADC reference voltage
#define ADC_REFERENCE_VOLTAGE 5.0

// Function prototypes
void ADC_init();
uint16_t ADC_read(uint8_t channel);
float ADC_convertToVoltage(uint16_t adcValue);

#endif // ADC_H