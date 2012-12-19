/*******************************************************************************
*   $FILE:  main.c
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  touch@atmel.com
******************************************************************************/

/*  License
*   Copyright (c) 2010, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*----------------------------------------------------------------------------
                            compiler information
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                include files
----------------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
/* #define __delay_cycles(n)     __builtin_avr_delay_cycles(n)
#define __enable_interrupt()  sei() */

#include "touch_api.h"
#include "touch.h"

// Debug
#include <stdio.h>

/*----------------------------------------------------------------------------
                            manifest constants
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/

#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                prototypes
----------------------------------------------------------------------------*/
extern void touch_measure();
extern void touch_init( void );
extern void init_system( void );
extern void init_timer_isr(void);
extern void set_timer_period(uint16_t);
/*----------------------------------------------------------------------------
                            Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = QT_MEASUREMENT_PERIOD_MS;
uint16_t time_ms_inc=0;
/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
volatile uint8_t time_to_measure_touch = 0u;

/* current time, set by timer ISR */
volatile uint16_t current_time_ms_touch = 0u;

/* STDIO */
static int usart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

static int usart_putchar(char a_inChar, FILE *stream)
{
	if (a_inChar == '\n')
	{
		usart_putchar('\r', stream);
	}
	
	// Wait for the transmit buffer to be empty
	while ( !( USARTE0_STATUS & USART_DREIF_bm) );
	
	// Put our character into the transmit buffer
	USARTE0_DATA = a_inChar;
	
	return 0;
}

ISR(TCD0_OVF_vect)
{
	printf("? %d\n", current_time_ms_touch);
}

#define TIMER_KEEPALIVE TCD0

/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :   main code entry point
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

int main( void )
{  
	cli();
	
	/* initialize host app, pins, watchdog, etc */
    init_system();

	/* Initialize Touch sensors */
	touch_init();

	/* configure timer ISR to fire regularly */
	init_timer_isr();

	/* Redirect stream to standard output */
	stdout = &mystdout;

	/* Send welcome message */
	printf("Lightning Wall Sensor\n");
	/* Write address */
	printf("Address: %#x\n", (PORTB_IN & 0x0F));
	
	printf("PMIC.CTRL: %#x\n", PMIC_CTRL);
	printf("TCC0.INTCTRLB: %#x\n", TCC0.INTCTRLB);
	printf("TCC0.PER: %#x\n", TCC0.PER);
	
	/********************************************//**
	 * Timer configuration section
	 ***********************************************/
	
	/* Enable the 4sec timer */
	
	TIMER_KEEPALIVE.CTRLB		= (TIMER_KEEPALIVE.CTRLB & ~TC0_WGMODE_gm) | TC_WGMODE_NORMAL_gc;
	TIMER_KEEPALIVE.CTRLFSET	= 0;																	// set UP direction
	TIMER_KEEPALIVE.PER			= 31250;												// set defined period
	TIMER_KEEPALIVE.INTCTRLA	= TIMER_KEEPALIVE.INTCTRLA & ~TC0_OVFINTLVL_gm;							// set overflow interrupt
	TIMER_KEEPALIVE.INTCTRLA	= TIMER_KEEPALIVE.INTCTRLA | TC_OVFINTLVL_MED_gc;	// set low-level overflow interrupt
	TIMER_KEEPALIVE.CTRLA		= (TIMER_KEEPALIVE.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV1024_gc;

	/*  enable low lever interrupts in power manager interrupt control  */
	PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	
	sei();

    /* loop forever */
    for( ; ; )
    {
	    touch_measure();
		
		printf("Sensor[0]: %d\n", GET_SENSOR_STATE(0));
		printf("Sensor[1]: %d\n", GET_SENSOR_STATE(1));
		printf("Sensor[2]: %d\n", GET_SENSOR_STATE(2));
		printf("Sensor[3]: %d\n", GET_SENSOR_STATE(3));
		printf("Sensor[4]: %d\n", GET_SENSOR_STATE(4));
		printf("Sensor[5]: %d\n", GET_SENSOR_STATE(5));

    /*  Time Non-critical host application code goes here  */
    }
}

