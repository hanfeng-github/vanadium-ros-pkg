/******************************************************************************
 * SG-1 Gripper MCB
 * 
 * Copyright (c) 2011-2012, Michael E. Ferguson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holders nor the names of
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <avr/io.h>
#include <stdint.h>
#include "ax_device.h"

// current sense is ADC3 (PC3)

/* LED */
#define DDR_LED         DDRD
#define PIN_LED         3

/* Motors OC1A/OC1B */
#define DDR_MOTORS      DDRB
#define PIN_OC1A        1
#define PIN_OC1B        2

/* AS5045 */
#define DDR_SPI         DDRB
#define PORT_SPI        PORTB
#define PIN_CS          0
#define PIN_MISO        4
#define PIN_SCK         5

void init_as5045(){
    DDR_SPI |= (1<<PIN_CS)|(1<<PIN_MISO)|(1<<PIN_SCK);
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

void read_as5045(){
    PORT_SPI |= (1<<PIN_CS);

    SPDR = 0x00;    // dummy tranmission
    while(!SPSR & (1<<SPIF));
    spi_data[0] = SPDR;

    SPDR = 0x00;
    while(!SPSR & (1<<SPIF));
    spi_data[1] = SPDR;


    SPDR = 0x00;
    while(!SPSR & (1<<SPIF));
    spi_data[2] = SPDR;

    PORT_SPI &= 0xff-(1<<SPIF);
}

int read_analog(const uint8_t address)
{
    ADMUX &= ~0x1F;                 // clear channel selection (low 5 bits)
    ADMUX |= address-REG_ANALOG;    // select specified channel
    
    delayus(100);
    ADCSRA |= (1<<ADSC);            // ADC start conversion
    while ( ADCSRA&(1<<ADIF) )
        ;

    delayus(100);
    ADCSRA |= (1<<ADSC);            // first is crap!
    while ( ADCSRA&(1<<ADIF) )
        ;

    return ADC/4;
}

void write_led(const uint8_t address, const uint8_t value){
    if(value > 0)
        PORT_LED &= 0xff-(1<<LED_PIN);
    else
        PORT_LED |= 1<<LED_PIN;
}

int main(){
    // initialize ADC    
    ADMUX = 0x00;     // Aref(5V)
    ADCSRA = 0x86;    // enable, no auto trigger, no interrupt, clk/64

    // initialize bus
    ax_device_init();
    sei();

    // setup ports
    DDR_LED |= (1<<LED_PIN);
    PORT_LED |= (1<<LED_PIN);
    init_as5045();

    // register callbacks
    ax_device_register_callbacks(AX_GOAL_POSITION_L, 0, &write_goal);
    ax_device_register_callbacks(AX_GOAL_POSITION_L+1, 0, &write_goal);
    ax_device_register_callbacks(AX_PRESENT_POSITION_L, &read_position, 0);
    ax_device_register_callbacks(AX_PRESENT_POSITION_L+1, &read_position, 0);
    ax_device_register_callbacks(AX_LED, 0, &write_led);
    
    // process
    while(1){
        ax_device_process();
    }

    return 0;
}

