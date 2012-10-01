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

int main (void)
{
	// myHome Actor 8 Channel Relay board based on ATXMEGA16A4U chipset and custom PCB.
	// Currently no special board initialization actions specified.
	board_init();
	
	// Insert application code here, after the board has been initialized.
	
	// TODO: retrieve channels state from NVM and make a set?
	
	// TODO: Initialize FIFO for RS-485 transmission.
	
	// TODO: Initialize USART
	
	// TODO: Initialize RS-485 transceiver
	
	// TODO: Read status of GPIOs which control relays

	// TODO: start main loop, go to sleep and wait for interruption
}	// main()
