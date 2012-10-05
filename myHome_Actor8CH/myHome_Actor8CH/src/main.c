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
	
	// TODO: Initialize FIFO for RS-485 transmission.
	
	// TODO: Initialize USART
	
	// TODO: Initialize RS-485 transceiver
	
	// TODO: Read status of GPIOs which control relays

	// Start infinite main loop, go to sleep and wait for interruption
	while( 1 )
	{
		/* Go to sleep, everything is handled by interrupts. */
		sleepmgr_enter_sleep();
	};


}	// main()
