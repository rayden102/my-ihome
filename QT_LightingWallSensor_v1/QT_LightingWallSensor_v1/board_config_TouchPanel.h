/*
 * board_config_TouchPanel.h
 *
 * Created: 2012-12-17 18:17:42
 *  Author: Tomasz Fidecki
 */ 


#ifndef BOARD_CONFIG_TOUCHPANEL_H_
#define BOARD_CONFIG_TOUCHPANEL_H_

/* USARTD0 used for RS-485 transmission */
#define USART_RS485					&USARTD0
#define USART_RS485_CHAR_SIZE		USART_CHSIZE_8BIT_gc
#define USART_RS485_PARITY			USART_PMODE_DISABLED_gc
#define USART_RS485_STOP_BIT		0
#define USART_RS485_CMODE			USART_CMODE_ASYNCHRONOUS_gc

/* 19200 baud rate defines: BSEL and BSCALE			*/
/* with Double Transmission Speed (CLK2X): disabled */
/* Selection for CPU clock: 2MHz					*/
/* #define USART_RS485_BSEL_19200		705
#define USART_RS485_BSCALE_19200	-7
#define USART_RS485_BSEL_9600		1539
#define USART_RS485_BSCALE_9600		-7 */

/* Selection for CPU clock: 8MHz					*/
#define USART_RS485_BSEL_19200	3205
#define USART_RS485_BSCALE_19200	-7
#define USART_RS485_BSEL_9600	3269
#define USART_RS485_BSCALE_9600	-6

/* Selection for CPU clock: 32MHz					*/
/* #define USART_RS485_BSEL_19200		3301
#define USART_RS485_BSCALE_19200	-5
#define USART_RS485_BSEL_9600		3317
#define USART_RS485_BSCALE_9600		-4 */

/* RS-485 transceiver transmission direction control pin */
#define RS485_DRIVER_PORT			PORTD
#define RS584_DRIVER_CTRL_bm		0x01	// PD0: RS-485 driver bit mask
#define RS584_DRIVER_CTRL_bp		0		// PD0: RS-485 driver bit position

/* USARTE0 used for terminal */
#define USART_TERMINAL				&USARTE0
#define USART_TERMINAL_CHAR_SIZE	USART_CHSIZE_8BIT_gc
#define USART_TERMINAL_PARITY		USART_PMODE_DISABLED_gc
#define USART_TERMINAL_STOP_BIT		0
#define USART_TERMINAL_CMODE		USART_CMODE_ASYNCHRONOUS_gc
/* Selection for CPU clock: 32MHz					*/
/* #define USART_TERMINAL_BSEL_19200	3301
#define USART_TERMINAL_BSCALE_19200	-5
#define USART_TERMINAL_BSEL_9600	3317
#define USART_TERMINAL_BSCALE_9600	-4 */

/* Selection for CPU clock: 8MHz					*/
#define USART_TERMINAL_BSEL_19200	3205
#define USART_TERMINAL_BSCALE_19200	-7
#define USART_TERMINAL_BSEL_9600	3269
#define USART_TERMINAL_BSCALE_9600	-6

/* Selection for CPU clock: 2MHz					*/
/* #define USART_TERMINAL_BSEL_19200		705
#define USART_TERMINAL_BSCALE_19200	-7
#define USART_TERMINAL_BSEL_9600		1539
#define USART_TERMINAL_BSCALE_9600		-7 */

#endif /* BOARD_CONFIG_TOUCHPANEL_H_ */