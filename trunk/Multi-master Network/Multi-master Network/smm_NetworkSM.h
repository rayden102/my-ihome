/*
Serial Multi-Master Network State Machine
*/

#ifndef SMM_NETWORKSM_H_
#define SMM_NETWORKSM_H_

#include <stdbool.h>
#include <stdlib.h>
#include "nvm_driver/nvm_driver.h"

/** EVENT definitions */

//! Interrupt type events
//! Received data from serial bus
#define EVENT_IRQ_RECEIVE_COMPLETE_bm			(1 << 0)
//! Data register empty
#define EVENT_IRQ_DATA_REGISTER_EMPTY_bm		(1 << 1)
//! Frame completely transmitted
#define EVENT_IRQ_TRANSMIT_COMPLETE_bm			(1 << 2)
//! Data is ready to be sent
#define EVENT_SW_DATA_READY_TO_SEND_bm			(1 << 3)
//! Busy line timer time out
#define EVENT_IRQ_BUSY_LINE_TIMEOUT_bm			(1 << 4)
//! Waiting for response timer time out
#define EVENT_IRQ_WAIT_FOR_RESPONSE_TIMEOUT_bm	(1 << 5)
//! System heartbeat timer time out
#define EVENT_IRQ_HEARTBEAT_TIMEOUT_bm			(1 << 6)

//! Software events
//! Received data is error free
#define EVENT_SW_RECEIVE_DATA_NO_ERROR_bm		(1 << 7)
//! Received data is erroneous
#define EVENT_SW_RECEIVE_DATA_ERROR_bm			(1 << 8)
//! Communication frame data integrity error
#define EVENT_SW_COMM_FRAME_CRC_ERROR_bm		(1 << 9)
//! Communication frame complete
#define EVENT_SW_COMM_FRAME_COMPLETE_bm			(1 << 10)
//! Communication frame incomplete
#define EVENT_SW_COMM_FRAME_INCOMPLETE_bm		(1 << 11)
//! Communication frame without processing
#define EVENT_SW_COMM_FRAME_NO_PROCESSING_bm	(1 << 12)
//! Communication frame without processing
#define EVENT_SW_UNEXPECTED_EVENT_RECEIVED_bm	(1 << 13)

/* Error types */
typedef enum eNetworkError
{
	eNE_None = 0,
	eNE_MaximumRetries,
	eNE_USART_Receiver_Error,
	eNE_Frame_CRC,
	eNE_Unexpected_Event,
	
	eNE_MAX
} eNetworkError_Type;

/* Line free/busy indicators */
enum eBusyLine {FREE = 0, BUSY = 1};

/* Size of Network receiving FIFO buffer */
#define FIFO_RECEIVE_BUFFER_SIZE	(16)

/* Size of Network receiving FIFO buffer */
#define FIFO_SEND_BUFFER_SIZE		(16)

/* Communication staff temporarily here */

// DATA FRAME
// [12b]	[4b]		   [8B]  [2B]
// [Address][Control Field][DATA][CRC-16]
// [Address] = [12bits] = [DeviceType, 4bits][DeviceNumber/SystemCommand, 7bits][RemoteTransmissionRequest (RTR), 1bit]
#define MMSN_COMM_FRAME_SIZE	(12)

/** Default device logical address in the network (DeviceNumber, 7bit value).
 *  Every device has default address after startup.
 */
#define MMSN_DEFAULT_LOGICAL_NETWORK_ADDRESS	(0xFF)

// Multi-Master Serial Network Destination Address offset
//#define MMSN_DST_ADDRESS_OFFSET	(0)
// Multi-Master Serial Network Source Address offset
//#define MMSN_SRC_ADDRESS_OFFSET	(2)
// Multi-Master Serial Network data start offset
//#define MMSN_DATA_OFFSET		(4)
// Multi-Master Serial Network CRC-16 value offset
//#define MMSN_CRC16_OFFSET		(12)

// Multi-Master Serial Network data length
#define MMSN_DATA_LENGTH		(8)
// CRC-16 data length
#define MMSN_CRC_LENGTH			(2)
// Multi-Master Serial Network frame without CRC-16 value length
#define MMSN_FRAME_NOCRC_LENGTH	(MMSN_COMM_FRAME_SIZE - MMSN_CRC_LENGTH)

/**
 * \brief Structure containing the Multi-Master Serial Network Communication Data Frame
 *
 * This structure can be used to store the communication frame.
 */
struct mmsn_comm_data_frame {
	union {
		struct {
			uint16_t u16Identifier;
			uint8_t	 u8DataArray[MMSN_DATA_LENGTH];
			uint16_t u16CRC16;
		};
		uint8_t u8CommFrameArray[MMSN_COMM_FRAME_SIZE];
	};
};

typedef struct mmsn_comm_data_frame mmsn_comm_data_frame_t;

#define MMSN_ADDRESS_bm	0xFFF0	/* Multi-Master Serial Network Address bit mask */
#define MMSN_ADDRESS_bp	4		/* Multi-Master Serial Network Address bit position */
#define MMSN_DEVTYPE_bm	0xF000	/* Multi-Master Serial Network Device Type bit mask */
#define MMSN_DEVTYPE_bp	12		/* Multi-Master Serial Network Device Type bit position */
#define MMSN_DEVNUM_bm  0x0FE0	/* Multi-Master Serial Network Device Number bit mask */
#define MMSN_DEVNUM_bp  5		/* Multi-Master Serial Network Device Number bit position */
#define MMSN_RTR_bm		0x0010	/* Multi-Master Serial Network Remote Transmission Request bit mask */
#define MMSN_RTR_bp		4		/* Multi-Master Serial Network Remote Transmission Request bit position */
#define MMSN_CTRLF_bm	0x000F	/* Multi-Master Serial Network Control Field bit mask */
#define MMSN_CTRLF_bp	0		/* Multi-Master Serial Network Control Field bit position */

/* Macros to compose and decode frame */
#define get_MMSN_Address(_u16Identifier, _u16Address)	\
	_u16Address = ((_u16Identifier & MMSN_ADDRESS_bm) >> MMSN_ADDRESS_bp)

#define set_MMSN_Address(_u16Address, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_ADDRESS_bm)) | (_u16Address << MMSN_ADDRESS_bp)

#define get_MMSN_DeviceType(_u16Identifier, _u8DeviceType)	\
	_u8DeviceType = ((_u16Identifier & MMSN_DEVTYPE_bm) >> MMSN_DEVTYPE_bp)
	
#define set_MMSN_DeviceType(_u8DeviceType, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_DEVTYPE_bm)) | (_u8DeviceType << MMSN_DEVTYPE_bp)
	
#define get_MMSN_DeviceNumber(_u16Identifier, _u8DeviceNum)	\
	_u8DeviceNum = ((_u16Identifier & MMSN_DEVNUM_bm) >> MMSN_DEVNUM_bp)

#define set_MMSN_DeviceNumber(_u8DeviceNum, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_DEVNUM_bm)) | ((_u8DeviceNum & 0x7F) << MMSN_DEVNUM_bp)
	
#define get_MMSN_RTR(_u16Identifier, _u8RTR)	\
	_u8RTR = ((_u16Identifier & MMSN_RTR_bm) >> MMSN_RTR_bp)

#define set_MMSN_RTR(_u8RTR, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_RTR_bm)) | (_u8RTR << MMSN_RTR_bp)

#define get_MMSN_CTRLF(_u16Identifier, _u8CtrlF)	\
	_u8CtrlF = ((_u16Identifier & MMSN_CTRLF_bm) >> MMSN_CTRLF_bp)

#define set_MMSN_CTRLF(_u8CtrlF, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_CTRLF_bm)) | (_u8CtrlF << MMSN_CTRLF_bp)

#define	MMSN_ConfigurationUnit	(0x00)
#define	MMSN_SupervisorUnit		(0x01)
#define	MMSN_ButtonUnit			(0x02)
#define	MMSN_RelayUnit			(0x03)
#define	MMSN_InfraRedUnit		(0x04)
#define	MMSN_WiFiUnit			(0x05)
#define	MMSN_TemperatureUnit	(0x06)
#define	MMSN_DimmerUnit			(0x07)
#define	MMSN_BlindDriverUnit	(0x08)
#define	MMSN_InputUnit			(0x09)
#define	MMSN_InterfaceUnit		(0x0A)
#define	MMSN_Reserved_1			(0x0B)
#define MMSN_Reserved_2			(0x0C)
#define	MMSN_Reserved_3			(0x0D)
#define	MMSN_Reserved_4			(0x0E)
#define	MMSN_Reserved_5			(0x0F)
#define	MMSN_Reserved_6			(0x10)

enum eRemoteTransmissionRequest
{
	eRTR_DataFrame = 0,
	eRTR_RemoteFrame = 1
};

// Device types definitions of Multi-Master Serial Network
/*
enum eMMSN_DeviceType
{
	eMMSN_ConfigurationUnit = 0x00,
	eMMSN_SupervisorUnit	= 0x01,
	eMMSN_ButtonUnit		= 0x02,
	eMMSN_RelayUnit			= 0x03,
	eMMSN_InfraRedUnit		= 0x04,
	eMMSN_WiFiUnit			= 0x05,
	eMMSN_TemperatureUnit	= 0x06,
	eMMSN_DimmerUnit		= 0x07,
	eMMSN_BlindDriverUnit	= 0x08,
	eMMSN_InputUnit			= 0x09,
	eMMSN_InterfaceUnit		= 0x0A,
	eMMSN_Reserved_1		= 0x0B,
	eMMSN_Reserved_2		= 0x0C,
	eMMSN_Reserved_3		= 0x0D,
	eMMSN_Reserved_4		= 0x0E,
	eMMSN_Reserved_5		= 0x0F,
	eMMSN_Reserved_6		= 0x10,
}; */

// Processing routines prototypes
void processCommand_Status(void);

// Establish table of pointers to processing functions
// void (* processingFunctions[])(void) = { processCommand_Status };

typedef enum eTransmitMessageType
{
	NORMAL = 0,
	ACK,
	NACK
} eTransmitMessageType_t;

typedef enum eSM_State
{
	eSM_Initialize = 0,
	eSM_Idle,
	eSM_Receive,
	eSM_ProcessData,
	eSM_ExecuteCommand,
	eSM_Send,
	eSM_WaitForResend,
	eSM_Retransmission,
	eSM_WaitForResponse,
	eSM_Error
} eSM_StateType;

/* Function prototypes to handle individual state */
void fsm_Initialize(void);
void fsm_Idle(void);
void fsm_Receive(void);
void fsm_ProcessData(void);
void fsm_ExecuteCommand(void);
void fsm_Send(void);
void fsm_WaitForResend(void);
void fsm_Retransmission(void);
void fsm_WaitForResponse(void);
void fsm_Error(void);

/************************************************************************/
/* Helper functions                                                     */
/************************************************************************/

/**
 * \brief Determine if Logical Network Address was assigned to the device.
 *
 * This function checks if logical network address was already assigned and stored
 * in the EEPROM. 
 *
 * \param a_pu8LogicalNetworkAddr pointer to the variable holding logical address.
 *
 * \retval true if logical address was already assigned.
 * \retval false if logical address was not assigned.
 */
bool isLogicalNetworkAddrAssigned(uint8_t *a_pu8LogicalNetworkAddr);

/**
 * \brief Structure containing the xmega shortened serial number.
 *
 * This structure is used to store shortened (7 bytes) device serial number.
 * This would be needed when supervisor is requesting uC serial number encoded on 7 bytes.
 * Shortened serial number is comprised of lotnum4, lotnum5, ..., and coordy1 bytes.
 * These are 7 lowest bytes which are more unique for a device.
 */
typedef struct xmega_shortened_serial_number
{
	union {
		struct {
			uint8_t lotnum4;
			uint8_t lotnum5;
			uint8_t wafnum;
			uint8_t coordx0;
			uint8_t coordx1;
			uint8_t coordy0;
			uint8_t coordy1;
		};
		uint8_t u8DataArray[7];
	};
} xmega_shortened_serial_number_t;

/**
 * \brief Get the shortened XMEGA device serial number.
 *
 * This function gets the shortened version XMEGA device serial number.
 * Shortened version of complete serial number is comprised of 7 bytes
 * excluding lotnum0 - lotnum3 bytes.
 *
 * \Note Functions arguments (pointers) must be properly provided. No checks against NULL pointers are made.
 *
 * \param a_pInCompleteSerialNum	Pointer to the structure holding complete device serial number (11 bytes).
 * \param a_pOutShortenedSerialNum	Pointer to the structure holding shortened device serial number (7 bytes).
 *
 * \retval none.
 */
void xmega_get_shortened_serial_num(struct nvm_device_serial *a_pInCompleteSerialNum, xmega_shortened_serial_number_t *a_pOutShortenedSerialNum);

/**
 *  \brief Function generates random logical network address.
 *		   Function utilizes rand() function to compute a pseudo-random integer
 *		   in the range of 1 to 127.
 *
 *  \param none.
 *
 *  \return unsigned 8bit random value within range <1..127>
 */
uint8_t xmega_generate_random_logical_network_address(void);

#endif /* SMM_NETWORKSM_H_ */