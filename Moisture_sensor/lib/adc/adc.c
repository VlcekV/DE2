#include "adc.h"

void ADC_init() {
    // Set the ADC reference voltage to AVCC
    ADMUX |= (1 << REFS0);
    
    // Enable the ADC and set the prescaler to 128 for a 16MHz clock
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_read(uint8_t channel) {
    // Clear the previous channel selection and set the new channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    // Start the ADC conversion
    ADCSRA |= (1 << ADSC);

    // Wait for the conversion to complete
    while (ADCSRA & (1 << ADSC));

    // Return the ADC result
    return ADC;
}

float ADC_convertToVoltage(uint16_t adcValue) {
    return (adcValue * ADC_REFERENCE_VOLTAGE) / 1023.0;
}