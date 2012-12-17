/*
 * xmega_usart.h
 *
 * Created: 2012-12-17 17:10:25
 *  Author: Tomasz Fidecki
 */ 


#ifndef XMEGA_USART_H_
#define XMEGA_USART_H_

#define xmega_usart_baudrate(_usart, _bselValue, _bScaleFactor)				\
(_usart)->BAUDCTRLA =(uint8_t)_bselValue;									\
(_usart)->BAUDCTRLB =(_bScaleFactor << USART_BSCALE0_bp)|(_bselValue >> 8)

/**
 * \brief  Configure the USART frame format.
 *
 *  Sets the communication mode, frame format, Frame Size, parity mode and number of stop bits.
 *
 *  \param _usart Pointer to the USART module
 *  \param _charSize The character size. Use USART_CHSIZE_t type.
 *  \param _parityMode The parity Mode. Use USART_PMODE_t type.
 *  \param _twoStopBits Enable two stop bit mode. Use bool type.
 */
#define xmega_usart_format_set(_usart, _charSize, _parityMode, _twoStopBits)	\
(_usart)->CTRLC = (uint8_t)_charSize | (uint8_t)_parityMode | (_twoStopBits ? USART_SBMODE_bm : 0)

/**
 * \brief Set the mode the USART run in.
 *
 * Set the mode the USART run in. The default mode is asynchronous mode.
 *
 * \param _usart Pointer to the USART module register section.
 * \param _commMode Selects the USART communication mode. Use USART_CMODE_t type.
 *
 * USART modes:
 * - 0x0        : Asynchronous mode.
 * - 0x1        : Synchronous mode.
 * - 0x2        : IrDA mode.
 * - 0x3        : Master SPI mode.
 */
#define xmega_usart_set_mode(_usart, _commMode)	\
(_usart)->CTRLC = ((_usart)->CTRLC & (~USART_CMODE_gm)) | _commMode

/**
 * \brief Set USART RXD interrupt level.
 *
 * Sets the interrupt level on Receive Complete Interrupt Level (RXCINTLVL).
 *
 * \param _usart Pointer to the USART module.
 * \param _level Interrupt level of the RXD interrupt. Use USART_RXCINTLVL_t type.
 */
#define xmega_usart_set_rx_interrupt_level(_usart, _level)	\
(_usart)->CTRLA = ((_usart)->CTRLA & ~USART_RXCINTLVL_gm) | _level

/**
 * \brief Set USART TXD interrupt level.
 *
 * Sets the interrupt level on Transmit Complete Interrupt Level (TXCINTLVL).
 *
 * \param _usart Pointer to the USART module.
 * \param _level Interrupt level of the TXD interrupt. Use USART_TXCINTLVL_t type.
 */
#define xmega_usart_set_tx_interrupt_level(_usart, _level)	\
(_usart)->CTRLA = ((_usart)->CTRLA & ~USART_TXCINTLVL_gm) | _level

/**
 * \brief Set USART DRE interrupt level.
 *
 * Sets the interrupt level on Data Register Empty Interrupt Level (DREINTLVL).
 *
 * \param _usart Pointer to the USART module.
 * \param _level Interrupt level of the DRE interrupt. Use USART_DREINTLVL_t type.
 */
#define xmega_usart_set_dre_interrupt_level(_usart, _level)	\
(_usart)->CTRLA = ((_usart)->CTRLA & ~USART_DREINTLVL_gm) | _level

/**
 * \brief Enable USART transmitter.
 *
 * \param _usart Pointer to the USART module.
 */
#define xmega_usart_tx_enable(_usart)	\
(_usart)->CTRLB |= USART_TXEN_bm

/**
 * \brief Disable USART transmitter.
 *
 * \param _usart Pointer to the USART module.
 */
#define xmega_usart_tx_disable(_usart)	\
(_usart)->CTRLB &= ~USART_TXEN_bm

/**
 * \brief Enable USART receiver.
 *
 * \param _usart Pointer to the USART module
 */
#define xmega_usart_rx_enable(_usart)	\
(_usart)->CTRLB |= USART_RXEN_bm

/**
 * \brief Disable USART receiver.
 *
 * \param _usart Pointer to the USART module.
 */
#define xmega_usart_rx_disable(_usart)	\
(_usart)->CTRLB &= ~USART_RXEN_bm;

#endif /* XMEGA_USART_H_ */