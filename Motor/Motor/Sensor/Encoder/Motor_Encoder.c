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
    @file   Motor_Encoder.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "Motor_Encoder.h"
#include "Encoder_Sensor.h"

#include "../../Motor_StateMachine.h"
#include "../../Motor.h"
#include "../../Motor_Var.h"
#include "../../Motor_FOC.h"
#include "../RotorSensor.h"


static inline Encoder_T * GetEncoder(const Motor_T * p_motor) { return &p_motor->SENSOR_TABLE.ENCODER.ENCODER; }
static inline Encoder_State_T * GetEncoderState(const Motor_T * p_motor) { return GetEncoder(p_motor)->P_STATE; }

//todo test

/******************************************************************************/
/*
    Homing States
*/
/******************************************************************************/
static void StartHoming(const Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Phase_ActivateT0(&p_motor->PHASE);
    Encoder_StartHoming(GetEncoderState(p_motor));
    Angle_ZeroCaptureState(&p_motor->P_MOTOR->OpenLoopAngle);
}

static void ProcHoming(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;

    /* alternatively openloop speed/angle ramp */
    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        angle16_t angle = Encoder_GetHomingAngle(GetEncoderState(p_motor)) * p_state->Config.PolePairs;
        Angle_IntegrateDelta(&p_state->OpenLoopAngle, angle);
        Motor_FOC_ProcAngleFeedforwardV(p_state, Angle_GetAngle16(&p_state->OpenLoopAngle), Motor_GetVAlign_Duty(p_state), 0);
        Motor_FOC_WriteDuty(p_motor);
    }
}

static State_T * HomingTransition(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (Encoder_PollHomingComplete(GetEncoderState(p_motor)) == true)
    {
        Phase_Deactivate(&p_motor->PHASE);
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    return p_nextState;
}

//todo add align
static const State_T STATE_ENCODER_HOMING =
{
    // .ID         = MOTOR_STATE_ID_CALIBRATION,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)StartHoming,
    .LOOP       = (State_Action_T)ProcHoming,
    .NEXT       = (State_Input0_T)HomingTransition,
};


void Motor_Encoder_StartHoming(const Motor_T * p_motor)
{
    StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&STATE_ENCODER_HOMING);
}

/*  */
void Motor_Encoder_CalibrateHomeOffset(const Motor_T * p_motor)
{
    Encoder_CalibrateIndexZeroRef(GetEncoderState(p_motor));
}

void Motor_Encoder_StartVirtualHome(const Motor_T * p_motor)
{
    // Motor_Calibration_StartHome(p_motor);
}

/******************************************************************************/
/*
    Open Loop - Align States
*/
/******************************************************************************/
static const State_T VALIDATE_ALIGN;
static const State_T VALIDATE_CLOSED_LOOP;

static void AlignEntry(const Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Motor_FOC_StartStartUpAlign(p_motor->P_MOTOR);
}

static void AlignLoop(const Motor_T * p_motor)
{
    Motor_FOC_ProcStartUpAlign(p_motor->P_MOTOR);
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * AlignZeroNext(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        Angle_ZeroCaptureState(&p_motor->P_MOTOR->OpenLoopAngle);
        Encoder_CaptureAlignZero(GetEncoderState(p_motor));
        p_nextState = &VALIDATE_ALIGN;
    }
    else
    {
        /* Reset the timer until speed is 0 */
        if (Encoder_ModeDT_GetScalarSpeed(GetEncoderState(p_motor)) != 0)
        {
            TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
        }
    }

    return p_nextState;
}

static const State_T ALIGN =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)AlignEntry,
    .LOOP       = (State_Action_T)AlignLoop,
    .NEXT       = (State_Input0_T)AlignZeroNext,
};


/*
    Validate Align - Run open loop, check encoder tracks
*/
static void ValidateAlign(const Motor_T * p_motor)
{
    Motor_State_T * p_state = p_motor->P_MOTOR;
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_state->Config.AlignTime_Cycles * 2U);
    Encoder_ModeDT_SetInitial(GetEncoder(p_motor));
    FOC_SetVd(&p_state->Foc, 0);
    Motor_FOC_MatchFeedbackState(p_state);
    Motor_FOC_StartOpenLoop(p_state);
    p_state->SensorState.MechanicalAngle = Encoder_GetAngle(GetEncoderState(p_motor));
}

static void ProcOpenLoop(const Motor_T * p_motor)
{
    Motor_FOC_ProcOpenLoop(p_motor->P_MOTOR);
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * ValidateAlignNext(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        if (Encoder_GetAngle(GetEncoderState(p_motor)) != p_motor->P_MOTOR->SensorState.MechanicalAngle)
        {
            p_motor->P_MOTOR->FaultFlags.PositionSensor = 1U;
            p_nextState = &MOTOR_STATE_FAULT;
        }
        else
        {
            p_nextState = &VALIDATE_CLOSED_LOOP;
        }
    }

    return p_nextState;
}

static const State_T VALIDATE_ALIGN =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)ValidateAlign,
    .LOOP       = (State_Action_T)ProcOpenLoop,
    .NEXT       = (State_Input0_T)ValidateAlignNext,
};


/*
    Validate Closed Loop - Run closed loop, check speed/voltage polarity
*/
static void ValidateClosedLoopEntry(const Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles * 2U);
    Motor_FOC_MatchFeedbackState(p_motor->P_MOTOR);
}

static void ProcAngleControl(const Motor_T * p_motor)
{
    Motor_FOC_ProcAngleControl(p_motor->P_MOTOR);
    Motor_FOC_WriteDuty(p_motor);
}

static State_T * ValidateClosedLoopTransition(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;
    Motor_State_T * p_state = p_motor->P_MOTOR;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        p_state->FeedbackMode.OpenLoop = 0U;
        Encoder_CompleteAlignValidate(GetEncoderState(p_motor));
        p_nextState = &MOTOR_STATE_OPEN_LOOP; /* return to parent */
    }
    else
    {
        /* Check speed/voltage polarity match */
        if ((Motor_GetSpeedFeedback(p_state) ^ math_sign(FOC_Vq(&p_state->Foc))) < 0)
        {
            p_state->FaultFlags.PositionSensor = 1U;
            p_nextState = &MOTOR_STATE_FAULT;
        }
    }

    return p_nextState;
}

static const State_T VALIDATE_CLOSED_LOOP =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)ValidateClosedLoopEntry,
    .LOOP       = (State_Action_T)ProcAngleControl,
    .NEXT       = (State_Input0_T)ValidateClosedLoopTransition,
};


static State_T * Cmd_Align(const Motor_T * p_motor, state_value_t null)
{
    return &ALIGN;
}

/* individual state test */
void Motor_Encoder_StartAlignZero(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)Cmd_Align, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0);
}

static State_T * Cmd_ValidateAlign(const Motor_T * p_motor, state_value_t null)
{
    return &VALIDATE_ALIGN;
}

void Motor_Encoder_StartValidateAlign(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)Cmd_ValidateAlign, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0);
}

static State_T * Cmd_ValidateClosedLoop(const Motor_T * p_motor, state_value_t null)
{
    return &VALIDATE_CLOSED_LOOP;
}

void Motor_Encoder_StartValidateClosedLoop(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_OPEN_LOOP, .NEXT = (State_Input_T)Cmd_ValidateClosedLoop, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0);
}


/******************************************************************************/
/*
    Start Up Chain
*/
/******************************************************************************/
static State_T * StartUpTransition(const Motor_T * p_motor);
static State_T * StartUpAlignTransition(const Motor_T * p_motor);
static State_T * StartUpValidateAlignTransition(const Motor_T * p_motor);
static State_T * StartUpValidateClosedLoopTransition(const Motor_T * p_motor);

static const State_T START_UP =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .NEXT       = (State_Input0_T)StartUpTransition,
};

static const State_T START_UP_ALIGN =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)AlignEntry,
    .LOOP       = (State_Action_T)AlignLoop,
    .NEXT       = (State_Input0_T)StartUpAlignTransition,
};

static const State_T START_UP_VALIDATE_ALIGN =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)ValidateAlign,
    .LOOP       = (State_Action_T)ProcOpenLoop,
    .NEXT       = (State_Input0_T)StartUpValidateAlignTransition,
};

static const State_T START_UP_VALIDATE_CLOSED_LOOP =
{
    .P_TOP      = &MOTOR_STATE_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)ValidateClosedLoopEntry,
    .LOOP       = (State_Action_T)ProcAngleControl,
    .NEXT       = (State_Input0_T)StartUpValidateClosedLoopTransition,
};

static State_T * StartUpTransition(const Motor_T * p_motor)
{
    return &START_UP_ALIGN;
}

static State_T * StartUpAlignTransition(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        Angle_ZeroCaptureState(&p_motor->P_MOTOR->OpenLoopAngle);
        Encoder_CaptureAlignZero(GetEncoderState(p_motor));
        p_nextState = &START_UP_VALIDATE_ALIGN;
    }
    else
    {
        if (Encoder_ModeDT_GetScalarSpeed(GetEncoderState(p_motor)) != 0)
        {
            TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
        }
    }

    return p_nextState;
}

static State_T * StartUpValidateAlignTransition(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        if (Encoder_GetAngle(GetEncoderState(p_motor)) != p_motor->P_MOTOR->SensorState.MechanicalAngle)
        {
            p_motor->P_MOTOR->FaultFlags.PositionSensor = 1U;
            p_nextState = &MOTOR_STATE_FAULT;
        }
        else
        {
            p_nextState = &START_UP_VALIDATE_CLOSED_LOOP;
        }
    }

    return p_nextState;
}

static State_T * StartUpValidateClosedLoopTransition(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;
    Motor_State_T * p_state = p_motor->P_MOTOR;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        p_state->FeedbackMode.OpenLoop = 0U;
        Encoder_CompleteAlignValidate(GetEncoderState(p_motor));
        p_nextState = &MOTOR_STATE_RUN;
    }
    else
    {
        if ((Motor_GetSpeedFeedback(p_state) ^ math_sign(FOC_Vq(&p_state->Foc))) < 0)
        {
            p_state->FaultFlags.PositionSensor = 1U;
            p_nextState = &MOTOR_STATE_FAULT;
        }
    }

    return p_nextState;
}

State_T * Motor_Encoder_GetStartUpState(const Motor_T * p_motor)
{
    return &START_UP;
}

static State_T * StartUpChain(const Motor_T * p_motor, state_value_t null)
{
    if (Motor_GetSpeedFeedback(p_motor->P_MOTOR) == 0) return &START_UP;
    else return NULL;
}

void Motor_Encoder_StartUpChain(const Motor_T * p_motor)
{
    static const StateMachine_TransitionCmd_T START_UP_CHAIN = { .P_START = &MOTOR_STATE_PASSIVE, .NEXT = (State_Input_T)StartUpChain, };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &START_UP_CHAIN, 0);
}


/******************************************************************************/
/*
    Direction Calibration
*/
/******************************************************************************/
static inline void StartDirection(const Motor_T * p_motor)
{
    TimerT_Periodic_Init(&p_motor->CONTROL_TIMER, p_motor->P_MOTOR->Config.AlignTime_Cycles);
    Phase_WriteDuty_Fract16(&p_motor->PHASE, Motor_GetVAlign_Duty(p_motor->P_MOTOR), 0U, 0U);
}

static inline bool ProcDirection(const Motor_T * p_motor)
{
    bool isComplete = false;

    if (TimerT_Periodic_Poll(&p_motor->CONTROL_TIMER) == true)
    {
        switch (p_motor->P_MOTOR->CalibrationStateIndex)
        {
            case 0U:
                Encoder_CaptureQuadratureReference(GetEncoderState(p_motor));
                Phase_WriteDuty_Fract16(&p_motor->PHASE, 0U, Motor_GetVAlign_Duty(p_motor->P_MOTOR), 0U);
                p_motor->P_MOTOR->CalibrationStateIndex = 1U;
                break;

            case 1U:
                Encoder_CalibrateQuadraturePositive(GetEncoderState(p_motor));
                Phase_Deactivate(&p_motor->PHASE);
                isComplete = true;
                break;
            default: break;
        }
    }

    return isComplete;
}