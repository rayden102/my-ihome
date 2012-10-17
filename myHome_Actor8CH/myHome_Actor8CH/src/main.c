/**
 * \file
 *
 * \brief Interrupt-based code flow model.
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>

/*
 * Include header file for handling all communication details.
 */
#include "myHome_Comm.h"

/* *********************************************************************** */
/* ******************** INTERRUPT EVENT DEFINITIONS ********************** */
/* *********************************************************************** */
#define EVENT_USARTD0_RXC_bm		(1 << 0)
#define EVENT_HEARTBEAT_TIMER_bm	(1 << 1)

// Bitmasked flags that describe what interrupt has occurred
volatile uint16_t gInterruptEvents = 0;

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
/* ********************** RS-485 PROTOCOL RELATED ************************ */
/* *********************************************************************** */
// +----------------------------------------------+
// | RS-485 control pin |        Function		  |
// |--------------------|-------------------------|
// | LOW				| Receiver output enabled |
// |--------------------|-------------------------|
// | HIGH				| Driver output enabled   |
// +----------------------------------------------+
#define RS485_DRIVER_CONTROL_GPIO IOPORT_CREATE_PIN(PORTD,0)

/**
 * \brief Enable Receiver output in RS-485 chip
 *
 * This function enables receiver (RO) output in RS-485
 * transceiver device.
 *
 * \param none.
 *
 * \retval none
 */
static inline void rs485_receiver_enable(void)
{
	ioport_set_pin_low(RS485_DRIVER_CONTROL_GPIO);
}	// rs485_receiver_enable()

/**
 * \brief Enable Driver output in RS-485 chip
 *
 * This function enables driver (DE) output in RS-485
 * transceiver device.
 *
 * \param none.
 *
 * \retval none
 */
static inline void rs485_driver_enable(void)
{
	ioport_set_pin_high(RS485_DRIVER_CONTROL_GPIO);
}	// rs485_driver_enable()

/* *********************************************************************** */
/* ********************** RELAYS CONTROL RELATED ************************* */
/* *********************************************************************** */
volatile uint8_t g_u8_relays_state = 0;

/* *********************************************************************** */
/* ************************ SELF CONFIGURATION *************************** */
/* *********************************************************************** */
// -----------------
//  1    2    3    4
// PD4, PD5, PD6, PD7
// ------------------
#define A8CH_ADDRESS1_GPIO IOPORT_CREATE_PIN(PORTD,4)
#define A8CH_ADDRESS2_GPIO IOPORT_CREATE_PIN(PORTD,5)
#define A8CH_ADDRESS3_GPIO IOPORT_CREATE_PIN(PORTD,6)
#define A8CH_ADDRESS4_GPIO IOPORT_CREATE_PIN(PORTD,7)

/**
 *  \brief Returns 8-bit device identifier comprised of
 *		   4 higher bits group (nibble) of device type and
 *         4 lower bits  group (nibble) of encoded address.
 *
 *  \return Identifier
 */
static uint8_t get_device_ID(void)
{
	uint8_t u8Address = 0;
	
	// Configure pins (4-7) group of PORTD as inputs
	ioport_configure_group(IOPORT_PORTD, 0xF0, IOPORT_DIR_INPUT);
	
	// Put in device type into higher nibble
	u8Address = ((A8CH_DEVICE_TYPE << 4) & 0xF0);
	
	// Read all 4 pins at once
	u8Address |= (((PORTD.IN & 0xF0) >> 4) & 0x0F);
	// bPinState = ioport_get_value(A8CH_ADDRESS1_GPIO);
	
	return u8Address;
};	// get_device_ID()

// Variable to store device identifier configured by value of DIP switch and
// pre-defined device type
volatile uint8_t g_u8_device_id = 0;

/* *********************************************************************** */
/* ****************** MY_HOME COMMUNICATION SECTION ********************** */
/* *********************************************************************** */
data_frame_desc_t g_comm_data_frame_desc;

// Retrieve communication command from data descriptor
static uint8_t getCommCommand(data_frame_desc_t *a_pDataDesc)
{
	Assert(NULL != a_pDataDesc);
	
	return (uint8_t)(a_pDataDesc->eCommand);
}

// Processing routines prototypes
bool processCommand_Status(void);
bool processCommand_Set(void);
bool processCommand_GroupSet(void);

// Establish table of pointers to processing functions
bool (* processingFunctions[])(void) = { processCommand_Status, processCommand_Set, processCommand_GroupSet };

/* *********************************************************************** */
/* ************************* INTERRUPT HANDLERS ************************** */
/* *********************************************************************** */

/**
 * \brief Timer Counter Overflow interrupt callback function
 *
 * This function is called when an overflow interrupt has occurred on
 * TIMER_EXAMPLE and toggles LED0.
 */
static void heartbeat_ovf_irq_callback(void)
{
	// Be as quick as possible and only set the flag of corresponding event.
	gInterruptEvents |= EVENT_HEARTBEAT_TIMER_bm;
}	// heartbeat_ovf_irq_callback()

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTD0_RXC_vect)
{
	// Be as quick as possible and only set the flag of corresponding event.
	gInterruptEvents |= EVENT_USARTD0_RXC_bm;
}

/* *********************************************************************** */
/* *************************** EVENT HANDLERS **************************** */
/* *********************************************************************** */
static void HandleUSARTD0RXC(void)
{
	// Retrieve received data from DATA register. It will automatically clear RXCIF.
	uint8_t u8Data = (USART_RS485)->DATA;
	
	// Put new element into FIFO. No check, the buffer should have enough capacity to hold it.
	fifo_push_uint8_nocheck(&fifo_rs485_receive_buffer_desc, u8Data);
	
	// Clear corresponding event flag
	gInterruptEvents &= ~EVENT_USARTD0_RXC_bm;
}

static void HandleHeartbeatTimer(void)
{
	// Check if complete data frame was already received
	uint8_t u8FifoSize = fifo_get_used_size(&fifo_rs485_receive_buffer_desc);
		
	// Process if complete data frame received
	if (A8CH_DATA_FRAME_SIZE == u8FifoSize)
	{
		// Translate received data to myHome Communication Data format
		myHome_Comm_Data_CopyFrom(&fifo_rs485_receive_buffer_desc,
								  &g_comm_data_frame_desc);
			
		// Check if message is intended for this device
		// TODO: what if MPCM is ON? is it just a double check?
		if ( g_u8_device_id == g_comm_data_frame_desc.u8DeviceID)
		{
			// There is a job to do
			processingFunctions[getCommCommand(&g_comm_data_frame_desc)]();
		}
			
		// Flush FIFO
		fifo_flush(&fifo_rs485_receive_buffer_desc);
	}

	// Clear corresponding event flag
	gInterruptEvents &= ~EVENT_HEARTBEAT_TIMER_bm;
}

/*! \brief Function to retrieve current state of output channels.
 *
 * 8 output channels are mapped into 8 consequents GPIOs of PORTA.
 * Since mapping is 1:1 then it's just current state of entire PORTA.
 * which is returned.
 
 *  \return Byte representing PORTA state. 
 */
static inline uint8_t get_output_channels_state(void)
{
	return (PORTA_OUT);
};	// get_output_channels_state()



/* *********************************************************************** */
/* *********************** MAIN LOOP STARTS HERE ************************* */
/* *********************************************************************** */
int main (void)
{
	// Variable to read interrupt event flags
	uint16_t u16EventFlags;
	
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
	/* ********************** RS-485 FIFO CONFIGURATION ********************** */
	/* *********************************************************************** */
	
	// *** Initialize FIFO for RS-485 transmission ***
	fifo_init( &fifo_rs485_receive_buffer_desc,
			   &fifo_rs485_receive_buffer[0],
			   FIFO_RS485_RECEIVE_BUFFER_LENGTH );
	
	/* *********************************************************************** */
	/* ************************* COMMUNICATION DATA  ************************* */
	/* *********************************************************************** */
	myHome_Comm_Init(&g_comm_data_frame_desc);
	
	/* *********************************************************************** */
	/* ************************* SELF CONFIGURATION  ************************* */
	/* *********************************************************************** */
	// Read device ID
	g_u8_device_id = get_device_ID();
	
	/* *********************************************************************** */
	/* ************************* USART CONFIGURATION ************************* */
	/* *********************************************************************** */
	
	// *** Initialize USART ****
	// USART options
	// TODO: if MPCM then use suitable configuration
	static usart_rs232_options_t USART_RS485_OPTIONS =
	{
		.baudrate	= USART_RS485_BAUDRATE,
		.charlength = USART_RS485_CHAR_LENGTH,
		.paritytype = USART_RS485_PARITY,
		.stopbits	= USART_RS485_STOP_BIT
	};
	
	// Initialize USART driver in RS232 mode
	usart_init_rs232(USART_RS485, &USART_RS485_OPTIONS);

	// *** Initialize RS-485 transceiver ***
	// Initially go to LOW and this will enable receiver output
	ioport_configure_pin(RS485_DRIVER_CONTROL_GPIO, (IOPORT_INIT_LOW | IOPORT_DIR_OUTPUT));
	
	/* *********************************************************************** */
	/* ********************** OUTPUT CONFIGURATION *************************** */
	/* *********************************************************************** */
	// Configure all PORTA pins as outputs
	ioport_configure_group(IOPORT_PORTA, 0b11111111, IOPORT_DIR_OUTPUT);

	// TODO: retrieve channels state from NVM and make a set?

	// Read current status of GPIOs which control relays PA[0..7]
	g_u8_relays_state = get_output_channels_state();

	// Start infinite main loop, go to sleep and wait for interruption
	while( 1 )
	{
		// Atomic interrupt safe read of global variable storing event flags
		u16EventFlags = gInterruptEvents;
		
		while (u16EventFlags)
		{
			// Each handler will clear the relevant bit in global variable gInterruptEvents
			
			if (u16EventFlags & EVENT_USARTD0_RXC_bm)
			{
				// USART D0 receive interrupt fired up
				HandleUSARTD0RXC();
			}
			
			if (u16EventFlags & EVENT_HEARTBEAT_TIMER_bm)
			{
				// Heartbeat timer overflow interrupt fired up
				HandleHeartbeatTimer();
			}
			
			u16EventFlags = gInterruptEvents;
		}
		
		// Read the event register again without allowing any new interrupts
		cpu_irq_disable();
		if (0 == gInterruptEvents)
		{
			cpu_irq_enable();
			// Go to sleep, everything is handled by interrupts.
			// An interrupt will cause a wake up and run the while loop
			sleepmgr_enter_sleep();
		};
		
		cpu_irq_enable();
	};	// main while(1) loop

}	// main()
