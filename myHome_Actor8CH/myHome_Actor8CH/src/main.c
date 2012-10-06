/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>

/* *********************************************************************** */
/* *********************** RS-485 FIFO DEFINITION ************************ */
/* *********************************************************************** */

// Size of RS-485 receiving FIFO buffer
#define FIFO_RS485_RECEIVE_BUFFER_LENGTH	(16)

/**
 * \brief Buffer to associate with receiving FIFO buffer
 *
 * This buffer consists of \ref FIFO_RS485_RECEIVE_BUFFER_LENGTH elements
 * capable of holding a byte
 */
uint8_t fifo_rs485_receive_buffer [FIFO_RS485_RECEIVE_BUFFER_LENGTH];

/**
 * \brief RS-485 receiving FIFO buffer descriptor
 *
 * This descriptor contains information about the location of the FIFO buffer,
 * its size and where to read from or write to upon the next buffer pull or
 * push. This is required to access the FIFO buffer via the FIFO service.
 *
 * \pre The descriptor must be initialized with \ref fifo_init() before the FIFO
 * buffer can be used.
 *
 * \note The FIFO buffer should be used with only one of its supported datatypes
 * at a time, or the buffered values will be corrupted unless special conditions
 * are met.
 */
fifo_desc_t fifo_rs485_receive_buffer_desc;


/* *********************************************************************** */
/* ************************ FUNCTION DEFINITIONS ************************* */
/* *********************************************************************** */

/**
 * \brief Timer Counter Overflow interrupt callback function
 *
 * This function is called when an overflow interrupt has occurred on
 * TIMER_EXAMPLE and toggles LED0.
 */
static void heartbeat_ovf_irq_callback(void)
{
	// TODO: put heartbeat timer functionality here
}	// heartbeat_ovf_irq_callback()

/* *********************************************************************** */
/* *********************** MAIN LOOP STARTS HERE ************************* */
/* *********************************************************************** */
int main (void)
{
	// myHome Actor 8 Channel Relay board based on ATXMEGA16A4U chipset and custom PCB.
	// Enable interrupts
	pmic_init();
	// Currently no special board initialization actions are specified.
	board_init();
	// System clock initialization
	sysclk_init();
	// Sleep Manager initialization
	sleepmgr_init();

	// Enable interrupts
	cpu_irq_enable();
	
	/* *********************************************************************** */
	/* ************************* TIMER CONFIGURATION ************************* */
	/* *********************************************************************** */
	
	// Unmask clock
	tc_enable( &TIMER_HEARTBEAT);
	// Configure interrupts callback functions for system heartbeat timer.
	tc_set_overflow_interrupt_callback( &TIMER_HEARTBEAT,
										heartbeat_ovf_irq_callback );
	
	// Configure TC in normal mode and configure period
	tc_set_wgm( &TIMER_HEARTBEAT, TC_WG_NORMAL );
	tc_write_period( &TIMER_HEARTBEAT, TIMER_HEARTBEAT_PERIOD );

	// Enable TC overflow interrupt
	tc_set_overflow_interrupt_level( &TIMER_HEARTBEAT, TC_INT_LVL_LO );
	// Run system heartbeat timer at 10Hz resolution
	tc_set_resolution( &TIMER_HEARTBEAT, TIMER_HEARTBEAT_PERIOD );

	/* *********************************************************************** */
	/* ********************* EOF TIMER CONFIGURATION ************************* */
	/* *********************************************************************** */

	// TODO: retrieve channels state from NVM and make a set?
	
	/* *********************************************************************** */
	/* ********************** RS-485 FIFO CONFIGURATION ********************** */
	/* *********************************************************************** */
	
	// Initialize FIFO for RS-485 transmission
	fifo_init( &fifo_rs485_receive_buffer_desc,
			   &fifo_rs485_receive_buffer[0],
			   FIFO_RS485_RECEIVE_BUFFER_LENGTH );
	
	/* *********************************************************************** */
	/* ************************* USART CONFIGURATION ************************* */
	/* *********************************************************************** */
	
	// Initialize USART
	// USART options
	static usart_rs232_options_t USART_RS485_OPTIONS =
	{
		.baudrate	= USART_RS485_BAUDRATE,
		.charlength = USART_RS485_CHAR_LENGTH,
		.paritytype = USART_RS485_PARITY,
		.stopbits	= USART_RS485_STOP_BIT
	};
	
	// Initialize USART driver in RS232 mode
	usart_init_rs232(USART_RS485, &USART_RS485_OPTIONS);

	
	// TODO: Initialize RS-485 transceiver
	
	// TODO: Read status of GPIOs which control relays

	// Start infinite main loop, go to sleep and wait for interruption
	while( 1 )
	{
		/* Go to sleep, everything is handled by interrupts. */
		sleepmgr_enter_sleep();
	};


}	// main()
