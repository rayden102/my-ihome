/*
 * myHome_Comm.h
 *
 * Created: 2012-10-12 20:55:37
 *  Author: fidectom
 */ 

#ifndef MYHOME_COMM_H_
#define MYHOME_COMM_H_

#include "compiler.h"

// Actor with 8 Relay Channels
#define  A8CH_DATA_FRAME_SIZE	(8)

// Device types
#define A8CH_DEVICE_TYPE		(0x0000)

// Commands
typedef enum
{
	MYHOME_COMM_STATUS		= 0x0000,
	MYHOME_COMM_SET,
	MYHOME_COMM_GROUPSET,
	
	MYHOME_COMM_MAX
} MyHomeCommCommands;

//! Data Frame descriptor.
struct data_frame_desc
{
	uint8_t				u8DeviceType;
	MyHomeCommCommands	eCommand;    
	uint8_t				u8DataArray[6];
};

typedef struct data_frame_desc data_frame_desc_t;

void myHome_Comm_Init(data_frame_desc_t * a_pDataFrameDesc);

void myHome_Comm_Data_Get(fifo_desc_t * a_pFifoDesc, data_frame_desc_t * a_pDataFrameDesc);

#endif /* MYHOME_COMM_H_ */