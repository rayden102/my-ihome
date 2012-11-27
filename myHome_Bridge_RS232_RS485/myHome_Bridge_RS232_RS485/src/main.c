/**
 * \file main.c
 *
 * \brief myHome Bridge between RS-232 and RS-485 interfaces based on ATXMEGA32A4U chipset and XMega 32A4 base board_ver.2 PCB
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <stdio.h>

/* Global variable to count number of bytes transferred by DMA channel
 * used in Host to Bridge transmission.
 */
volatile uint16_t gHost2BridgeDataCounter = 0;

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

// Host to Bridge direction DMA channel configuration structure
// struct dma_channel_config hostToBridgeDmaConfig;

// Bridge to Host direction DMA channel configuration structure
// struct dma_channel_config bridgeToHostDmaConfig;

/**
 * \brief Setup DMA Channel transmitting data from Host (RS-232 side) to Bridge (RS-485 side)
 *
 * This function sets up DMA channel used for data flow between two USART peripherals.
 * One is responsible for RS-232 transmission (Host to Bridge) and latter
 * controls RS-485 side (Bridge to Bus).
 * DMA channel is configured with the following settings: 
 *  - Low interrupt priority
 *  - 1 byte burst length
 *  - 1 byte for each transfer
 *  - Do not reload source and destination address at end of each transfer
 *  - Fixed source and destination address during transfer
 *  - Source address is set to \ref USART_RS232 .DATA register
 *  - Destination address is set to \ref USART_RS485 .DATA register
 *  - Single-shot transfer
 *  - Triggered by \ref USART_RS232 RXC interrupt 
 *
 * \param none.
 *
 * \retval none
 */

#ifdef ZERO
static void setupHostToBridge_DMAChannel(void)
{
	/* Make sure DMA config for this channel is all zeroed out so we don't get any stray bits */
	memset(&hostToBridgeDmaConfig, 0, sizeof(hostToBridgeDmaConfig));
	
	// Configure DMA channel	
	dma_channel_set_interrupt_level(&hostToBridgeDmaConfig,		DMA_INT_LVL_MED);
	dma_channel_set_burst_length(&hostToBridgeDmaConfig,		DMA_CH_BURSTLEN_1BYTE_gc);
	dma_channel_set_transfer_count(&hostToBridgeDmaConfig,		1);
	dma_channel_set_src_reload_mode(&hostToBridgeDmaConfig,		DMA_CH_SRCRELOAD_NONE_gc);
	dma_channel_set_dest_reload_mode(&hostToBridgeDmaConfig,	DMA_CH_DESTRELOAD_NONE_gc);
	dma_channel_set_src_dir_mode(&hostToBridgeDmaConfig,		DMA_CH_SRCDIR_FIXED_gc);
	dma_channel_set_dest_dir_mode(&hostToBridgeDmaConfig,		DMA_CH_DESTDIR_FIXED_gc);
	dma_channel_set_source_address(&hostToBridgeDmaConfig,		(uint16_t)(USART_RS232.DATA));
	dma_channel_set_destination_address(&hostToBridgeDmaConfig, (uint16_t)(USART_RS485.DATA));
	
	// Enable repeat mode and set repcnt to zero
	dma_channel_set_repeats(&hostToBridgeDmaConfig, 0);	// REPCNT = 0
	dma_channel_set_single_shot(&hostToBridgeDmaConfig); 
	
	// Set DMA channel trigger source as USART C1 Receive Complete
	dma_channel_set_trigger_source(&hostToBridgeDmaConfig, DMA_CH_TRIGSRC_USARTC1_RXC_gc);
	
	// Write DMA channel configuration to hardware
	dma_channel_write_config(DMA_CHANNEL_HOST2BRIDGE, &hostToBridgeDmaConfig);

	/* Use the configuration above by enabling the DMA channel in use. */
	dma_channel_enable(DMA_CHANNEL_HOST2BRIDGE);
}

/**
 * \brief Callback for DMA transfer complete
 *
 * \param status Status of a completed (or failed) DMA transfer
 */
static void host2bridge_dma_transfer_done(enum dma_channel_status status)
{
	/* Check DMA transfer status */
	if (DMA_CH_TRANSFER_COMPLETED == status)
	{
		// Increase data counter
		gHost2BridgeDataCounter++;
	}
}
#endif

/**
 *
 * \brief Timer overflow callback
 *
 * \note The callback sends out message to the Host using RS-232 interface.
 * Only if channel is not busy.
 */
static void timer_overflow_callback(void)
{
	// if (!dma_channel_is_busy(DMA_CHANNEL_HOST2BRIDGE))
	{
		printf("Host2Bridge data counter = %d\n", gHost2BridgeDataCounter);
	}
}

#define xmega_usart_baudrate(_usart, _bselValue, _bScaleFactor)				\
(_usart)->BAUDCTRLA =(uint8_t)_bselValue;									\
(_usart)->BAUDCTRLB =(_bScaleFactor << USART_BSCALE0_bp)|(_bselValue >> 8)

static inline void usartHostInit(void)
{
	/* Set USART transmission 9600 baud rate */
	/* BSCALE = -7		*/
	/* CLK2X = 0		*/
	/* BSEL = 1539		*/
	/* Error = 0,01%	*/
		
	/* USART initialization should use the following sequence:
		1. Set the TxD pin value high, and optionally set the XCK pin low.
		2. Set the TxD and optionally the XCK pin as output.
		3. Set the baud rate and frame format.
		4. Set the mode of operation (enables XCK pin output in synchronous mode).
		5. Enable the transmitter or the receiver, depending on the usage.
	For interrupt-driven USART operation, global interrupts should be disabled during the
	initialization. */	
	
	/* Disable global interrupts */
	cpu_irq_disable();
		
	/* PC7 (TXD0) as output - high */
	/* PC6 (RXD0) as input */
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 7), IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 6), IOPORT_DIR_INPUT);
	
	/* Enable system clock to peripheral */
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_USART1);
	
	/* Set the baud rate: use BSCALE and BSEL */
	xmega_usart_baudrate(USART_RS232, 1539, -7);
	
	/* Set frame format */
	usart_format_set(USART_RS232, USART_RS232_CHAR_LENGTH, USART_RS232_PARITY, USART_RS232_STOP_BIT);

	/* Set mode */
	usart_set_mode(USART_RS232, USART_CMODE_ASYNCHRONOUS_gc);
	
	/* Set interrupts level */
	usart_set_rx_interrupt_level(USART_RS232, USART_INT_LVL_MED);
	usart_set_tx_interrupt_level(USART_RS232, USART_INT_LVL_MED);
	
	/* Enable transmitter and receiver */
	usart_tx_enable(USART_RS232);
	usart_rx_enable(USART_RS232);
	
	/* Enable interrupts */
	cpu_irq_enable();
}	// usartHostInit()

static inline void usartBridgeInit(void)
{
	/* Set USART transmission 9600 baud rate */
	/* BSCALE = -7		*/
	/* CLK2X = 1		*/
	/* BSEL = 3205		*/
	/* Error = 0,01%	*/
	
	/* USART initialization should use the following sequence:
		1. Set the TxD pin value high, and optionally set the XCK pin low.
		2. Set the TxD and optionally the XCK pin as output.
		3. Set the baud rate and frame format.
		4. Set the mode of operation (enables XCK pin output in synchronous mode).
		5. Enable the transmitter or the receiver, depending on the usage.
	For interrupt-driven USART operation, global interrupts should be disabled during the
	initialization. */
		
	/* PC3 (TXD0) as output - high */
	/* PC2 (RXD0) as input */
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTD, 3), IOPORT_DIR_OUTPUT	| IOPORT_INIT_HIGH);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTD, 2), IOPORT_DIR_INPUT);
	
	/* Disable global interrupts */
	cpu_irq_disable();

	/* Enable system clock to peripheral */
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_USART0);
	
	/* Set the baud rate: use BSCALE and BSEL */
	xmega_usart_baudrate(USART_RS485, 1539, -7);
	
	/* Set frame format */
	usart_format_set(USART_RS485, USART_RS232_CHAR_LENGTH, USART_RS485_PARITY, USART_RS485_STOP_BIT);

	/* Set mode */
	usart_set_mode(USART_RS485, USART_CMODE_ASYNCHRONOUS_gc);
	
	/* Set interrupts level */
	usart_set_rx_interrupt_level(USART_RS485, USART_INT_LVL_MED);
	usart_set_tx_interrupt_level(USART_RS485, USART_INT_LVL_MED);
	
	/* Enable transmitter and receiver */
	usart_tx_enable(USART_RS485);
	usart_rx_enable(USART_RS485);
	
	/* Enable interrupts */
	cpu_irq_enable();

}	// usartBridgeInit()

static inline void usartDMATestInit(void)
{
	/* Set USART transmission 9600 baud rate */
	/* BSCALE = -7		*/
	/* CLK2X = 0		*/
	/* BSEL = 1539		*/
	/* Error = 0,01%	*/
		
	/* USART initialization should use the following sequence:
		1. Set the TxD pin value high, and optionally set the XCK pin low.
		2. Set the TxD and optionally the XCK pin as output.
		3. Set the baud rate and frame format.
		4. Set the mode of operation (enables XCK pin output in synchronous mode).
		5. Enable the transmitter or the receiver, depending on the usage.
	For interrupt-driven USART operation, global interrupts should be disabled during the
	initialization. */	
	
	/* Disable global interrupts */
	cpu_irq_disable();
	
	/* PC3 (TXD0) as output - high */
	/* PC2 (RXD0) as input */
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 3), IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(IOPORT_CREATE_PIN(PORTC, 2), IOPORT_DIR_INPUT);
	
	/* Enable system clock to peripheral */
	sysclk_enable_module(SYSCLK_PORT_C, SYSCLK_USART0);
	
	/* Set the baud rate: use BSCALE and BSEL */
	xmega_usart_baudrate(USART_OUT, 1539, -7);

	/* Set frame format */
	usart_format_set(USART_OUT, USART_OUT_CHAR_LENGTH, USART_OUT_PARITY, USART_OUT_STOP_BIT);

	/* Set mode */
	usart_set_mode(USART_OUT, USART_CMODE_ASYNCHRONOUS_gc);
	
	/* Set interrupts level */
	usart_set_rx_interrupt_level(USART_OUT, USART_INT_LVL_MED);
	usart_set_tx_interrupt_level(USART_OUT, USART_INT_LVL_MED);
	
	/* Enable transmitter and receiver */
	usart_tx_enable(USART_OUT);
	usart_rx_enable(USART_OUT);
	
	/* Enable interrupts */
	cpu_irq_enable();
}

static int uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);

static int uart_putchar(char a_inChar, FILE *stream)
{
	if (a_inChar == '\n')
	{
		uart_putchar('\r', stream);
	}	
	
	// Wait for the transmit buffer to be empty
	while ( !( USARTC1_STATUS & USART_DREIF_bm) );
	
	// Put our character into the transmit buffer
	USARTC1_DATA = a_inChar;
	
	return 0;
}

#define AVR_ENTER_CRITICAL_REGION( ) uint8_t volatile saved_sreg = SREG; \
cli();

#define AVR_LEAVE_CRITICAL_REGION( ) SREG = saved_sreg;

/*! \brief CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critical
 *  operation of writing to the registers.
 *
 *  \param address A pointer to the address to write to.
 *  \param value   The value to put in to the register.
 */
static void CCPWrite(volatile uint8_t * address, uint8_t value)
{
#ifdef __ICCAVR__
	// Store global interrupt setting in scratch register and disable interrupts.
    asm("in  R1, 0x3F \n"
	    "cli"
	    );
	// Move destination address pointer to Z pointer registers.
	asm("movw r30, r16");
#ifdef RAMPZ
	asm("ldi  R16, 0 \n"
        "out  0x3B, R16"
	    );
#endif
	asm("ldi  r16,  0xD8 \n"
	    "out  0x34, r16  \n"
#if (__MEMORY_MODEL__ == 1)
	    "st     Z,  r17  \n");
#elif (__MEMORY_MODEL__ == 2)
	    "st     Z,  r18  \n");
#else /* (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5) */
	    "st     Z,  r19  \n");
#endif /* __MEMORY_MODEL__ */

	// Restore global interrupt setting from scratch register.
        asm("out  0x3F, R1");

#elif defined __GNUC__
	AVR_ENTER_CRITICAL_REGION( );
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);

	AVR_LEAVE_CRITICAL_REGION( );
#endif
}

static inline void xmega_clocks_init(void)
{
	/** System Clocks initialization */
	uint8_t u8temp;
	
	/** Use internal 2MHz oscillator initialization */
	OSC.CTRL |= OSC_RC2MEN_bm;
	
	/** Wait for the internal oscillator to stabilize */
	while ((OSC.STATUS & OSC_RC2MRDY_bm) == 0);
	
	/** System Clock prescaler A division factor: 1
		System Clock prescalers B & C division factors: B:1, C:1
		ClkPer4: 2MHz
		ClkPer2: 2MHz
		ClkPer:  2MHz
		ClkCPU:  2MHz */
	u8temp = (CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
			  CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	
	/** Enable the change of a protected register */
	CCPWrite(&CLK.PSCTRL, u8temp);
	
	/** Select the system clock source: Internal 2 MHz RC Oscillator */
	u8temp = (CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC2M_gc;
	
	/** Enable the change of a protected register */
	CCPWrite(&CLK.CTRL, u8temp);
	
	/** Disable the unused oscillators: 32 MHz, internal 32 kHz, PLL, external */
	OSC.CTRL &= ~(OSC_RC32MEN_bm | OSC_RC32KEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);
	
	/** PLL Control Register
	    - clock source: 2MHz internal oscillator
		- PLL divided output: disabled
		- Multiplication factor: x1
	*/
	u8temp = (OSC.PLLCTRL & ~(OSC_PLLSRC_gm)) | OSC_PLLSRC_RC2M_gc;
	u8temp &= ~(OSC_PLLDIV_bm);	// disable divided output
	u8temp &= ~(OSC_PLLFAC_gm);	// set multiplication factor as x1
	OSC.PLLCTRL = u8temp;
	
	/** Peripheral Clock output: Disabled */
	PORTCFG.CLKEVOUT = (PORTCFG.CLKEVOUT & (~PORTCFG_CLKOUT_gm)) | PORTCFG_CLKOUT_OFF_gc;
}

/** Interrupt system initialization */
static inline void xmega_pmic_init(void)
{
	uint8_t u8temp;
	
	// Disable all interrupts
	cli();
	
	/** Configure PMIC
		Low level interrupt: On
		Round-robin scheduling for low level interrupt: Off
		Medium level interrupt: On
		High level interrupt: On
		The interrupt vectors will be placed at the start of the Application FLASH section
	*/
	u8temp = (PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
			 PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	
	/** Enable the change of a protected register */
	CCPWrite(&PMIC.CTRL, u8temp);
	
	/** The low-priority vector, INTPRI, must be set to 0 when round-robin
	  * scheduling is disabled to return to default interrupt priority order. */
	PMIC.INTPRI = 0;
}

/* ISR( USARTC0_RXC_vect, ISR_BLOCK )
{
	printf("\nRXC = %c\n", (USART_OUT)->DATA);
} */

static void xmega_dma_H2B_setup(void)
{
	/************************************************************************/
	/* DMA to transfer data from USART(PC) to USART(RS-485 bus)             */
	/************************************************************************/
	
	/* Enable clock for DMA module */
	sysclk_enable_module(SYSCLK_PORT_GEN, SYSCLK_DMA);
	
	/* Configure Sleep Manager */
	sleepmgr_lock_mode(SLEEPMGR_IDLE);

	/* Reset DMAC */
	DMA_CTRL = 0x00;
	DMA_CTRL = DMA_RESET_bm;
	
	/* Wait until reset is completed */
	while (DMA_CTRL & DMA_RESET_bm);
	
	/* Enable DMAC */
	DMA_CTRL = DMA_ENABLE_bm;
	
	/* Configure DMA Controller: no double buffer and round-robin priority */
	DMA_CTRL |= DMA_DBUFMODE_DISABLED_gc | DMA_PRIMODE_RR0123_gc;
	
	/* DMA Channel software reset */
	DMA_CH0_CTRLA = 0x00;
	DMA_CH0_CTRLA = DMA_CH_RESET_bm;
	
	/* Wait until reset is completed */
	while (DMA_CH0_CTRLA & DMA_CH_RESET_bm);
	
	/* Configure Control Register A: repeat mode, single-shot mode and 1 byte burst mode */
	DMA_CH0_CTRLA |= DMA_CH_REPEAT_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	
	/* Configure source and destination address: No reload, fixed */
	DMA_CH0_ADDRCTRL &= ~(DMA_CH_SRCRELOAD_gm | DMA_CH_SRCDIR_gm | DMA_CH_DESTRELOAD_gm | DMA_CH_DESTDIR_gm);
	DMA_CH0_ADDRCTRL |= (DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc | DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc);
	
	/* USARTC1 RXC interrupt as a trigger */
	DMA_CH0_TRIGSRC = DMA_CH_TRIGSRC_USARTC1_RXC_gc;
	
	/* Transfer count = 1 byte data size */
	DMA_CH0_TRFCNT = 1;
	
	/* Set Source Address as USART_RS232 DATA register */
	DMA_CH0_SRCADDR0 = (((uint16_t)(USART_RS232.DATA)) >> 0 * 8) & 0xFF;
	DMA_CH0_SRCADDR1 = (((uint16_t)(USART_RS232.DATA)) >> 1 * 8) & 0xFF;
	DMA_CH0_SRCADDR2 = 0;
	
	/* Set Destination Address as USART_RS485 DATA register */
	DMA_CH0_DESTADDR0 = (((uint16_t)(USART_RS485.DATA)) >> 0 * 8) & 0xFF;
	DMA_CH0_DESTADDR1 = (((uint16_t)(USART_RS485.DATA)) >> 1 * 8) & 0xFF;
	DMA_CH0_DESTADDR2 = 0;
	
	/* Set Repeat Counter Register to zero (unlimited repeat) */
	DMA_CH0_REPCNT = 0;
	
	/* Configure DMA Channel Control Register B */
	/* High interrupt level for transfer errors and transaction complete */
	DMA_CH0_CTRLB = (DMA_CH0_CTRLB & ~(DMA_CH_ERRINTLVL_gm | DMA_CH_TRNINTLVL_gm));
	DMA_CH0_CTRLB |= DMA_CH_ERRINTLVL_HI_gc | DMA_CH_TRNINTLVL_HI_gc;
	
	/* Enable DMA Channel */
	DMA_CH0_CTRLA |= DMA_CH_ENABLE_bm;
}

ISR(DMA_CH0_vect, ISR_BLOCK)
{
	/* TODO: Check DMA transfer status */
	/* uint8_t busy_pending    = DMA.STATUS;
	uint8_t error_completed = DMA.INTFLAGS; */

	/*
	 * Check lower and upper nibble of INTFLAGS register to find possible
	 * error or transfer completed status.
	 */
	/* error_completed &= (1 << num) | (1 << (num + 4));
	if (error_completed & (1 << (num + 4))) {
		return DMA_CH_TRANSFER_ERROR;
	} else if (error_completed & (1 << num)) {
		return DMA_CH_TRANSFER_COMPLETED;
	} */

	/*
	 * Check lower and upper nibble of STATUS register to find possible
	 * busy or pending completed status.
	 */
	/* busy_pending &= (1 << num) | (1 << (num + 4));
	if (busy_pending & (1 << (num + 4))) {
		return DMA_CH_BUSY;
	} else if (busy_pending & (1 << num)) {
		return DMA_CH_PENDING;
	}

	return DMA_CH_FREE; */
	
	/* Clear all interrupt flags by setting corresponding flag bits */
	DMA_CH0_CTRLB |= (DMA_CH_TRNIF_bm | DMA_CH_ERRIF_bm);
	
	// Increase data counter
	gHost2BridgeDataCounter++;
}

/* *********************************************************************** */
/* *********************** MAIN LOOP STARTS HERE ************************* */
/* *********************************************************************** */
int main (void)
{	
	// System clock initialization
	// sysclk_init();
	xmega_clocks_init();
	// Currently no special board initialization actions are specified.
	board_init();
	// Enable interrupts
	// pmic_init();
	xmega_pmic_init();
	// Sleep Manager initialization
	sleepmgr_init();		
		
	/* *********************************************************************** */
	/* ************************* USART CONFIGURATION ************************* */
	/* *********************************************************************** */
	
	// *** Initialize RS-485 transceiver ***
	// Initially go to HIGH and this will enable driver output
	ioport_configure_pin(RS485_DRIVER_CONTROL_GPIO, (IOPORT_INIT_HIGH | IOPORT_DIR_OUTPUT));

	// Initialize USARTs
	
	usartHostInit();
	usartBridgeInit();
	// usartDMATestInit();
	
	// Initialize DMA transmission on channel 0
	xmega_dma_H2B_setup();
	
	/* Enable DMAC and clear configuration */
	// dma_enable();
	
	/*
	 * Set callback function from completed DMA block transfers.
	 * This function is called whenever an interrupt occurs on the DMA
	 * channel this function is set to handle. A callback function is set
	 * by the dma_set_callback() function in the DMA driver interface.
	 */
	// dma_set_callback(DMA_CHANNEL_HOST2BRIDGE, host2bridge_dma_transfer_done);

	// Configure Host to Bridge DMA channel
	// setupHostToBridge_DMAChannel();		
	
	/********************************************//**
	 * Timer configuration section
	 ***********************************************/
	
	/*
	 * Enable the timer, and set it to count up.
	 * When it overflows, it triggers the message to the Host.
	 * 1Hz tick.
	 */
	tc_enable(&TIMER_KEEPALIVE);
	tc_set_overflow_interrupt_callback(&TIMER_KEEPALIVE, timer_overflow_callback);
	tc_set_wgm(&TIMER_KEEPALIVE, TC_WG_NORMAL);
	tc_set_direction(&TIMER_KEEPALIVE, TC_UP);
	tc_write_period(&TIMER_KEEPALIVE, TIMER_KEEPALIVE_PERIOD);
	tc_set_overflow_interrupt_level(&TIMER_KEEPALIVE, TC_INT_LVL_HI);
	
	tc_set_resolution(&TIMER_KEEPALIVE, TIMER_KEEPALIVE_RESOLUTION / 4);
	// tc_write_clock_source(&TIMER_KEEPALIVE, TC_CLKSEL_DIV64_gc);
	/* TIMER_KEEPALIVE.CTRLA = TIMER_KEEPALIVE.CTRLA & ~(TC0_CLKSEL_gm);
	TIMER_KEEPALIVE.CTRLA |= TC_CLKSEL_DIV64_gc; */
	
	stdout = &mystdout;
	
	/* Print out welcome message to Host */
	printf("Host2Bridge ver 1.0\n");
	
	uint32_t tc_clk_rate = sysclk_get_per_hz();
	printf("sysclk_get_per_hz = %ld\n", tc_clk_rate);
	
	cpu_irq_enable();
	
	// Start infinite main loop, go to sleep and wait for interruption
	for(;;)
	{
		sleepmgr_enter_sleep();
	};	// main for(;;) loop

}	// main()