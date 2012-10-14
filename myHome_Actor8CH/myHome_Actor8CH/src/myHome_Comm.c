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
	
	a_pDataFrameDesc->u8DeviceID	 = 0;
	a_pDataFrameDesc->eCommand		 = 0;
	a_pDataFrameDesc->u8DataArray[0] = 0;
	a_pDataFrameDesc->u8DataArray[1] = 0;
	a_pDataFrameDesc->u8DataArray[2] = 0;
	a_pDataFrameDesc->u8DataArray[3] = 0;
	a_pDataFrameDesc->u8DataArray[4] = 0;
	a_pDataFrameDesc->u8DataArray[5] = 0;
}; // myHome_Comm_Init()

/**
 *  \brief Copies 8 bytes of data from FIFO and
 *		   puts into data frame descriptor. 
 *
 *  \param a_pFifoDesc		Pointer to the FIFO descriptor.
 *  \param a_pDataFrameDesc Pointer to the data frame descriptor.
 *
 *  \return nothing
 *    \retval nothing
 */
void myHome_Comm_Data_CopyFrom(fifo_desc_t *a_pFifoDesc, data_frame_desc_t *a_pDataFrameDesc)
{
	uint8_t __attribute__((unused)) u8FifoStatus;
	uint8_t u8TmpData = 0;
	
	// Check provided arguments
	Assert(NULL != a_pFifoDesc);
	Assert(NULL != a_pDataFrameDesc);
	
	// Pull out first byte with device identifier (type and address)
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DeviceID));
	
	// Pull out byte with command
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &u8TmpData);
	a_pDataFrameDesc->eCommand = (MyHomeCommCommands_t)u8TmpData;
	
	// Get 6 consecutive bytes
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DataArray[0]));
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DataArray[1]));
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DataArray[2]));
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DataArray[3]));
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DataArray[4]));
	u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDataFrameDesc->u8DataArray[5]));
	
	// We should be OK here
	Assert(FIFO_OK == u8FifoStatus);
	
} // myHome_Comm_Data_Insert()