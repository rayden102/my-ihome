/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

// Main system timer definition
#define TIMER_HEARTBEAT	TCC0

// The board runs at default 2MHz system clock
// Wakeup timeout occurs 10 times per second (10Hz/100ms)
// The prescaler value is 64, thus clock = (2 000 000 / 64) = 31250
// To achieve 10Hz resolution the period value must be set to 3125
#define TIMER_HEARTBEAT_PERIOD		(3125)

#define TIMER_HEARTBEAT_RESOLUTION	(31250)

// Heartbeat timer frequency
#define A8CH_SYSTEM_TICK_FREQ		(10)

// Number of relay channels present on the board
#define A8CH_RELAY_CHANNELS_COUNT	(8)

// Enable USARTD0
#define CONF_BOARD_ENABLE_USARTD0

// MCU Port with output channels
#define A8CH_OUTPUT_PORT	PORTA

#endif // CONF_BOARD_H