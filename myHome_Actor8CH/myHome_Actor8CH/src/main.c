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
	// TODO: retrieve channels state from NVM?
	board_init();
	
	// Initialize FIFO for RS-485 transmission.


	// Insert application code here, after the board has been initialized.
}
