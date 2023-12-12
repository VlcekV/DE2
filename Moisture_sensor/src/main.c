/***********************************************************************
*
 * Smart plant watering system
 * Tůma, Vlček, Bárta, Bartoň
 * Arduino UNO, PlatformIO, I2C
 * Capacitive soil moisture sensor v1.2
 * Temperature and humidity sensor DHT12
 *
 **********************************************************************/

/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <stdlib.h>         // C library. Needed for number conversions
#include <oled.h>           // library for the work with OLED 
#include <twi.h>            // I2C/TWI library for AVR-GCC

/* Defines of the adresses for I2C -----------------------------------*/
#define SENSOR_ADR 0x5c
#define SENSOR_TEMP_MEM 2

/* Global variables --------------------------------------------------*/
/* Struct construction for value from moisture sensor ----------------*/
struct sensors {
    uint16_t x;
} sens;

// Declaration of "dht12" variable with structure "DHT_values_structure"
struct DHT_values_structure {
    uint8_t temp_int;
    uint8_t temp_dec;
    uint8_t checksum;
} dht12;

// Flag for printing new data from sensor
volatile uint8_t new_sensor_data = 0;
volatile uint8_t update = 0;

/* Function definitions ----------------------------------------------*/
int main(void)
{
    char string[4];
    int x;
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    // TWI
    twi_init();

    // Initialize USART to asynchronous, 8N1, 9600
     uart_init(UART_BAUD_SELECT(115200, F_CPU));
    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX = ADMUX | (1<<REFS0);
    // Select input channel ADC0 (voltage divider pin)
    ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
    // Enable ADC module
    ADCSRA = ADCSRA | (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA = ADCSRA | (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA = ADCSRA | (1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);

    // Timers 0 and 1 
    // Timer 1 is responsible for ADC conversion
    TIM1_OVF_4SEC
    TIM1_OVF_ENABLE
    
    // Timer 0 is responsible for printing values on OLED and UART   
    TIM0_OVF_16MS
    TIM0_OVF_ENABLE
   
    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1)
    {
    if (update == 1){
        // Boolean variable for is there any update? -> if yes, the print values
        update = 0;

        // Integers of ranges for moisture
        int rng_max = 920;
        int rng_min = 700;

        // Main formula for calculation of moisture 
        x = 100-(((sens.x-rng_min)*100)/(rng_max+70-rng_min));

        // Integer to Array, moisture
        itoa(x, string, 10);

        // OLED display of values
        oled_charMode(NORMALSIZE);
        oled_gotoxy(1, 1);
        oled_puts("Soil moisture");
        oled_gotoxy(1,2);
        oled_puts("---------------------");
        oled_gotoxy(1,4);
        oled_charMode(DOUBLESIZE);
        oled_puts(string);
        oled_gotoxy(5, 4);
        oled_puts("% ");
              
        // Integer to Array, temperature
        itoa(dht12.temp_int, string, 10);
        oled_charMode(NORMALSIZE);
        oled_gotoxy(1, 7);
        oled_puts("Room temp:");
        oled_gotoxy(13, 7);
        oled_puts(string);
        oled_gotoxy(15, 7);
        oled_puts(" °C");
        oled_display();

        // Relay switch on if moisture is below 50%
        DDRB |= (1<<DDB0);
        if(x >= 50){
            PORTB |= (1<<PORTB0);
        }
        else{
            PORTB &= ~(1<<PORTB0);
        }
    }
}
    // Will never reach this
    return 0;
}

/* Interrupt service routines ----------------------------------------*/
// Overflows of Timers 0 and 1
ISR(TIMER1_OVF_vect)
{   static uint16_t no_of_overflows = 0;

    no_of_overflows++;
    // Do ADC conversion every 4 seconds
    if (no_of_overflows >= 1) {
    // Start ADC conversion
    ADCSRA = ADCSRA | (1<<ADSC);
    uint8_t value;
    char string[8];
    // Null OVF
    no_of_overflows = 0;
    }
}

ISR(TIMER0_OVF_vect){
    static uint16_t no_of_overflows = 0;

    no_of_overflows++;

    // Do not read via TWI if oled is getting update
    // Do reading every 240*16ms = 3.84 seconds
    if (no_of_overflows >= 240 || update == 1) {  
        
        twi_start();
        if (twi_write((SENSOR_ADR<<1) | TWI_WRITE) == 0) {
            // Set internal memory location
            twi_write(SENSOR_TEMP_MEM);
            twi_stop();
            // Read data from internal memory
            twi_start();
            twi_write((SENSOR_ADR<<1) | TWI_READ);
            dht12.temp_int = twi_read(TWI_ACK);
            dht12.temp_dec = twi_read(TWI_NACK);
        }
        twi_stop();

        // Null OVF
        no_of_overflows = 0;
        char string[4];
    
        // Integers of ranges for moisture
        int rng_max = 920;
        int rng_min = 700;

        // Main formula for calculation of moisture 
        int x = 100-(((sens.x-rng_min)*100)/(rng_max+70-rng_min));
        // Integer to Array, moisture
        itoa(x, string, 10);

        // UART display of values
        uart_puts(string);
        uart_puts(" ");

        // Integer to Array, temperature
        itoa(dht12.temp_int, string, 10);

        uart_puts(string);
        uart_puts(" ");
        update = 1;
    }
}

// ADC completed interrupt 
ISR(ADC_vect)
{
    uint16_t value;
    char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    value = ADC;

    sens.x = (value); 
    update = 1;
}