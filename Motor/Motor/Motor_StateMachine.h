#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

    This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Motor_StateMachine.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_Commutation.h"
#include "Motor.h"

#include "Framework/StateMachine/StateMachine.h"
#include "Framework/StateMachine/_StateMachine.h" /* Include the private header to allocate within Motor_State_T */

#include "System/Critical/Critical.h"
#include "System/SysTime/SysTime.h"


/******************************************************************************/
/*
    Motor State Machine
*/
/******************************************************************************/
#ifndef MOTOR_STATE_MACHINE_INIT_WAIT
#define MOTOR_STATE_MACHINE_INIT_WAIT (1500U) /* For 1S polling to run twice */
#endif

/******************************************************************************/
/*!
    @brief  Motor State IDs
*/
/******************************************************************************/
typedef enum Motor_StateId
{
    MOTOR_STATE_ID_INIT,
    MOTOR_STATE_ID_DISABLED,      /* Quiescent gateway: phases off, direction null. Operation disabled until explicit transition. Sole entry to CALIBRATION  */
    MOTOR_STATE_ID_PASSIVE,       /* Freewheel or Hold. Feedback Off + Ouput V0/VZ. */
    MOTOR_STATE_ID_RUN,           /* Feedback Loop + Ouput VPWM */
    MOTOR_STATE_ID_INTERVENTION,
    MOTOR_STATE_ID_OPEN_LOOP,     /* Torque Loop On/Off + VZ/V0/VPWM */
    MOTOR_STATE_ID_CALIBRATION,
    MOTOR_STATE_ID_FAULT,
}
Motor_StateId_T;

// extern const State_T SUPERVISOR_STATE_INIT; // statemachine handler calls entr/exit conditions

/* extern for extension */
extern const State_T MOTOR_STATE_INIT;
extern const State_T MOTOR_STATE_DISABLED;
extern const State_T MOTOR_STATE_PASSIVE;
extern const State_T MOTOR_STATE_RUN;
extern const State_T MOTOR_STATE_OPEN_LOOP;
extern const State_T MOTOR_STATE_CALIBRATION;
extern const State_T MOTOR_STATE_FAULT;
extern const State_T MOTOR_STATE_INTERVENTION;

/******************************************************************************/
/*
    Motor State Machine Inputs
*/
/******************************************************************************/
typedef enum Motor_StateInput
{
    MOTOR_STATE_INPUT_FAULT,            /* Toggle Fault */
    MOTOR_STATE_INPUT_PHASE_OUTPUT,     /* [Phase_Output_T] Map to Run/Release */
    MOTOR_STATE_INPUT_FEEDBACK_MODE,    /* [FeedbackMode_T]  */
    MOTOR_STATE_INPUT_DIRECTION,        /* [Motor_Direction_T] */
    MOTOR_STATE_INPUT_OPEN_LOOP,        /* OpenLoop Cmd */
    MOTOR_STATE_INPUT_CALIBRATION,      /* Calibration Cmd */
    // MOTOR_STATE_INPUT_STATE_CMD, /* Transition Cmd, common entry for Stop, Start, Timer, etc */
    // MOTOR_STATE_INPUT_USER_BUFFER,
    // MOTOR_STATE_INPUT_CAPTURE_ADC,
    MOTOR_TRANSITION_TABLE_LENGTH,
}
Motor_StateInput_T;

/*
    [Phase_Output_T]
    VZ/V0 -> Release
    VPWM -> Run
*/


/******************************************************************************/
/*
    Motor State Machine Definition
*/
/******************************************************************************/
extern const StateMachine_Machine_T MSM_MACHINE;

/*!
    @param p_MotorDev [const Motor_T *]
    @param MotorRuntime [Motor_State_T]
*/
#define MOTOR_STATE_MACHINE_INIT(p_MotorDev, MotorRuntime) STATE_MACHINE_INIT((p_MotorDev), &MSM_MACHINE, &((MotorRuntime).StateMachine))


static inline void _Motor_StateMachine(StateMachine_Active_T * p_active, void * p_dev)
{
    if (_StateMachine_AcquireAsyncIsr(p_active) == true)
    {
        _StateMachine_Branch_ProcSyncOutput(p_active, p_dev);
        _StateMachine_ReleaseAsyncIsr(p_active);
    }
}

static inline void _Motor_StateMachine_Thread(StateMachine_T * p_stateMachine)
{
    _Motor_StateMachine(p_stateMachine->P_ACTIVE, p_stateMachine->P_CONTEXT);
}


/******************************************************************************/
/*
*/
/******************************************************************************/
static inline Motor_StateId_T Motor_GetStateId(const Motor_State_T * p_motor) { return StateMachine_GetRootState(&p_motor->StateMachine)->ID; }

/* Host side checks Root state to parse id */
/* handle with unique handler per type */
static inline state_t _Motor_GetSubStateId(const Motor_State_T * p_motor) { return StateMachine_GetLeafState(&p_motor->StateMachine)->ID; }

/*
    StateMachine controlled values
    Set via interface functions
*/
static inline Motor_FeedbackMode_T Motor_GetFeedbackMode(const Motor_State_T * p_motor) { return p_motor->FeedbackMode; }
static inline Motor_FaultFlags_T Motor_GetFaultFlags(const Motor_State_T * p_motor) { return p_motor->FaultFlags; }

/* SubStates */
// static inline uint32_t Motor_GetControlTimer(const Motor_State_T * p_motor)                      { return p_motor->ControlTimerBase; }
// static inline Motor_OpenLoopState_T Motor_GetOpenLoopState(const Motor_State_T * p_motor)        { return p_motor->OpenLoopState; }
// static inline Motor_CalibrationState_T Motor_GetCalibrationState(const Motor_State_T * p_motor)  { return p_motor->CalibrationState; }
// static inline uint8_t Motor_GetCalibrationStateIndex(const Motor_State_T * p_motor)              { return p_motor->CalibrationStateIndex; }

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Wrap Motor_T for interface */
/* Does not include substates */
static inline bool Motor_IsState(const Motor_T * p_motor, Motor_StateId_T stateId) { return (StateMachine_IsRootStateId(p_motor->STATE_MACHINE.P_ACTIVE, stateId)); }

static inline bool Motor_IsFault(const Motor_T * p_motor) { return Motor_IsState(p_motor, MOTOR_STATE_ID_FAULT); }

/* Optionally enforce config in calibration state, rather than disabled state */
static inline bool Motor_IsConfig(const Motor_T * p_motor)
{
    return (Motor_IsState(p_motor, MOTOR_STATE_ID_DISABLED) || Motor_IsState(p_motor, MOTOR_STATE_ID_CALIBRATION) || Motor_IsState(p_motor, MOTOR_STATE_ID_FAULT));
}

// static inline bool Motor_IsDisableReady(const Motor_T * p_motor) { return StateMachine_IsRootState(p_motor->STATE_MACHINE.P_ACTIVE, &MOTOR_STATE_PASSIVE); }


/******************************************************************************/
/*
    Transition to Stop / Runtime inactive
    Stop disables inputs until next Start.
*/
/******************************************************************************/
static State_T * _Motor_InputDisable(const Motor_T * p_motor, state_value_t value)
{
    (void)value;
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0U) { return &MOTOR_STATE_DISABLED; }
    return NULL;
}

static void Motor_Disable(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_PASSIVE, .NEXT = (State_Input_T)_Motor_InputDisable };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}

static State_T * _Motor_InputEnable(const Motor_T * p_motor, state_value_t value)
{
    (void)p_motor;
    (void)value;
    return &MOTOR_STATE_PASSIVE;
}

static void Motor_Enable(const Motor_T * p_motor)
{
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_DISABLED, .NEXT = (State_Input_T)_Motor_InputEnable };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
    // { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_STATE_CMD, MOTOR_STATE_INPUT_STATE_START); }
}

// static void Motor_NormalizeToDisabled(const Motor_T * p_motor)
// {
//     Motor_Calibration_Exit(p_motor);
//     Motor_OpenLoop_Exit(p_motor);
//     Motor_Disable(p_motor);
// }


/* todo add user init ramp down */

/******************************************************************************/
/*
*/
/******************************************************************************/
/*
    FaultCmd - Encodes set and clear delta flags into a single state_value_t.
    Handlers apply both unconditionally: OR FaultSet, AND-NOT FaultClear.
*/
typedef union Motor_FaultCmd
{
    struct { uint16_t FaultSet; uint16_t FaultClear; };
    uint32_t Value;
}
Motor_FaultCmd_T;

// extern void Motor_StateMachine_SetFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags);
// extern void Motor_StateMachine_ClearFault(const Motor_T * p_motor, Motor_FaultFlags_T faultFlags);
// extern bool Motor_StateMachine_TryClearFaultAll(const Motor_T * p_motor);

static void Motor_StateMachine_SetFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_FAULT, (Motor_FaultCmd_T){ .FaultSet = faultFlags.Value }.Value);
}

static void Motor_StateMachine_ClearFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    StateMachine_Tree_InputAsyncTransition(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_FAULT, (Motor_FaultCmd_T){ .FaultClear = faultFlags.Value }.Value);
}

static bool Motor_StateMachine_TryClearFaultAll(Motor_T * p_motor)
{
    Motor_StateMachine_ClearFault(p_motor, (Motor_FaultFlags_T){ .Value = UINT16_MAX });
    return !Motor_IsFault(p_motor);
}


// static inline void Motor_StateMachine_Input(const Motor_T * p_motor, Motor_StateInput_T input, uintptr_t value) { StateMachine_ApplyInput(&p_motor->STATE_MACHINE, input, value); }