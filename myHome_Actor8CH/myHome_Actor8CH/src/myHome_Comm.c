/*
 * myHome_Comm.c
 *
 * Created: 2012-10-12 21:27:24
 *  Author: fidectom
 */ 

#include "myHome_Comm.h"

void myHome_Comm_Init(data_frame_desc_t * a_pDataFrameDesc)
{
	// Check provided arguments
	Assert(NULL != a_pDataFrameDesc);
	
	a_pDataFrameDesc->u8DeviceType	 = 0;
	a_pDataFrameDesc->eCommand		 = 0;
	a_pDataFrameDesc->u8DataArray[0] = 0;
	a_pDataFrameDesc->u8DataArray[1] = 0;
	a_pDataFrameDesc->u8DataArray[2] = 0;
	a_pDataFrameDesc->u8DataArray[3] = 0;
	a_pDataFrameDesc->u8DataArray[4] = 0;
	a_pDataFrameDesc->u8DataArray[5] = 0;
}; // myHome_Comm_Init()

void myHome_Comm_Data_Get(fifo_desc_t * a_pFifoDesc, data_frame_desc_t * a_pDataFrameDesc)
{
	
	
} // myHome_Comm_Data_Get()