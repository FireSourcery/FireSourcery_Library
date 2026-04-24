
#include "_State.h"
#include "_State_Node.h"
#include "State.h"
#include "Config.h"

#include <stdatomic.h>

// struct StateMachine_Part
// {
//     State_T * p_PartState;

// }  ;

// struct StateMachine_SyncInputs
// {
//     void * p_context;
//     volatile uint32_t SyncInputMask; /* Bit mask of inputs that have been set. */
//     state_value_t * p_SyncInputs;  /* mapper version needs separate buffer */
// }  ;

// static inline State_T * StateMachine_Get (struct StateMachine_Part args) { return args.p_PartState = 0; }

// static inline void __StateMachine_ProcSyncInput(struct StateMachine_SyncInputs part)
// {
//     for (uint32_t inputMask = part.SyncInputMask; inputMask != 0UL; inputMask &= (inputMask - 1))
//     {
//         state_input_t input = __builtin_ctz(inputMask);
//         // assert(input < STATE_TRANSITION_TABLE_LENGTH_MAX); /* Ensure input is within range */
//         _StateMachine_CallInput(part.p_context, input, part.p_SyncInputs[input]);
//     }
// }
