/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

// Enable USARTD0 - [RS-485 bus]
#define CONF_BOARD_ENABLE_USARTD0

// Enable USARTC1 - [RS-232 <=> PC]
#define CONF_BOARD_ENABLE_USARTC1

// Enable USARTC0 - [DMA test gateway]
// #define CONF_BOARD_ENABLE_USARTC0

/** \brief Host to Bridge DMA channel to use */
#define DMA_CHANNEL_HOST2BRIDGE	0

/** \brief Bridge To Host DMA channel to use */
#define DMA_CHANNEL_BRIDGE2HOST	1

/** \brief Timer used to spontaneously send message via RS-232 to Host */
#define TIMER_KEEPALIVE TCC0

/** Period used in DMA fixed read/trigger/callback test */
#define TIMER_KEEPALIVE_PERIOD 31250

/** \note Timer resolution */
#define TIMER_KEEPALIVE_RESOLUTION 31250

#endif // CONF_BOARD_H
