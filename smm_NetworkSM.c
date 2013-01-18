/*
Serial Multi-Master Network State Machine
*/

#include <smm_NetworkSM.h>

/* Current state of StateMachine */
volatile uint8_t u8sm_CurrentState;
/* Previous state of StateMachine */
volatile uint8_t u8sm_PreviousState;

/* Lookup table containing a pointer to the function to call in each state */
void (*SM_stateTable[])(void) =
{
	fsm_Idle,
	fsm_Receive,
	fsm_ProcessData,
	fsm_ExecuteCommand,
	fsm_Send,
	fsm_WaitForResend,
	fsm_Retransmission,
	fsm_WaitForResponse,
	fsm_Error
};

void fsm_InitializeStateMachine(void)
{
	/* Set current state to \ref eSM_Initialize */
	u8sm_CurrentState = eSM_Initialize;
	/* Set previous state to current state */
	u8sm_PreviousState = u8sm_CurrentState;
	
	/* TODO: Clear sending and receiving FIFOs */
	
	/* TODO: reset hardware */
}

/* Check event FIFOs if there is anything to be sent */
void fsm_Idle(void)
{
	
	
	
};




void fsm_Receive(void);
void fsm_ProcessData(void);
void fsm_ExecuteCommand(void);
void fsm_Send(void);
void fsm_WaitForResend(void);
void fsm_Retransmission(void);
void fsm_WaitForResponse(void);
void fsm_Error(void);