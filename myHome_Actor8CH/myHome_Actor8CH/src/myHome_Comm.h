/*
 * myHome_Comm.h
 *
 * Created: 2012-10-12 20:55:37
 *  Author: fidectom
 */ 

#ifndef MYHOME_COMM_H_
#define MYHOME_COMM_H_

#include "compiler.h"
#include "fifo.h"

// Actor with 8 Relay Channels
#define  A8CH_DATA_FRAME_SIZE	(8)

// Device types
#define A8CH_DEVICE_TYPE		(0x00)

// Maximum relay number
#define A8CH_MAX_RELAY_INDEX			(7)

#define A8CH_SET_RELAY_INDEX_DATA_BYTE	(0)
#define A8CH_SET_RELAY_STATE_DATA_BYTE	(1)
#define A8CH_SET_DELAY_TIMER_DATA_BYTE	(2)
#define A8CH_SET_UPTIME1_DATA_BYTE		(3)
#define A8CH_SET_UPTIME2_DATA_BYTE		(4)
#define A8CH_SET_UPTIME3_DATA_BYTE		(5)

// Commands
typedef enum
{
	MYHOME_COMM_STATUS		= 0x0000,
	MYHOME_COMM_SET,
	MYHOME_COMM_GROUPSET,
	
	MYHOME_COMM_UNUSED		= 0xFF
} MyHomeCommCommands_t;

typedef enum
{
	MYHOME_A8CH_RELAY_OFF = 0,
	MYHOME_A8CH_RELAY_ON  = 1 
	
} MyHomeA8CHRelayStates_t;

//! Data Frame descriptor.
struct data_frame_desc
{
	uint8_t					u8DeviceID;		// device type (higher 4b) and address (lower 4b)
	MyHomeCommCommands_t	eCommand;		// command
	uint8_t					u8DataArray[6];	// 6 bytes of data
};

typedef struct data_frame_desc data_frame_desc_t;

void myHome_Comm_Init(data_frame_desc_t * a_pDataFrameDesc);

void myHome_Comm_Data_CopyFrom(fifo_desc_t *a_pFifoDesc, data_frame_desc_t *a_pDataFrameDesc);

#endif /* MYHOME_COMM_H_ */