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
#define TIMER_HEARTBEAT_PERIOD	(3125)

// Enable USARTD0
#define CONF_BOARD_ENABLE_USARTD0

#endif // CONF_BOARD_H