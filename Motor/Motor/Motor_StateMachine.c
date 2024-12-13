/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   MotorStateMachine.c
    @author FireSourcery
    @brief  MotorStateMachine
    @version V0
*/
/******************************************************************************/
#include "Motor_StateMachine.h"
#include "Motor_Calibration.h"
#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif
#include "Motor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "System/Critical/Critical.h"
#include "System/SysTime/SysTime.h"

/******************************************************************************/
/*!
    Motor Module functions depending on State
*/
/******************************************************************************/
/*
    Motor State Machine Thread Safety
    State Proc in PWM thread. User Input [Motor_ActivateControl] in Main thread.

    Async Mode - Need critical section during input.
    No Critical during transistion ->
        Prev State Proc may run before Entry.
        control proc may overwrite pid state set

    _StateMachine_ProcStateTransition inside Proc will run on PWM thread

    Inputs do not directly proc transition

    Sync Mode - Input value before Input Id, sufficient?
    Must check input flags every pwm cycle.

    CmdValue (Ramp Target) and CmdMode selectively sync inputs for StateMachine
*/
void Motor_ActivateControl(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
    _Critical_DisableIrq(); /* is this needed for setting buffers? */
    // if(p_motor->FeedbackMode.Word != mode.Word)
    { StateMachine_SetInput(&p_motor->StateMachine, MSM_INPUT_CONTROL, mode.Word); }
    _Critical_EnableIrq();
}

/* Generic array functions use */
void Motor_ActivateControl_Cast(Motor_T * p_motor, uint8_t modeWord) { Motor_ActivateControl(p_motor, Motor_FeedbackMode_Cast(modeWord)); }

/******************************************************************************/
/* Fault interface functions */
/******************************************************************************/
bool Motor_StateMachine_IsFault(const Motor_T * p_motor) { return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT); }

void Motor_StateMachine_EnterFault(Motor_T * p_motor)
{
    if(Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, 0U); }
}

/*! @return true if no fault remains */
bool Motor_StateMachine_ExitFault(Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, 0U); }
    return Motor_StateMachine_IsFault(p_motor);
}

void Motor_StateMachine_SetFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    // p_motor->FaultFlags.Value |= faultFlags.Value;
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, faultFlags.Value); }
}

/*! @return true if cleared applies, fault to non fault */
bool Motor_StateMachine_ClearFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    bool isFault = Motor_StateMachine_IsFault(p_motor);
    // p_motor->FaultFlags.Value &= ~faultFlags.Value;
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, faultFlags.Value); }
    return (Motor_StateMachine_IsFault(p_motor) != isFault);
}

/******************************************************************************/
/*!
    @brief State Machine
*/
/******************************************************************************/
static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_STOP;
static const StateMachine_State_T STATE_RUN;
static const StateMachine_State_T STATE_FREEWHEEL;
static const StateMachine_State_T STATE_OPEN_LOOP;
static const StateMachine_State_T STATE_CALIBRATION;
static const StateMachine_State_T STATE_FAULT;

const StateMachine_Machine_T MSM_MACHINE =
{
    .P_STATE_INITIAL = &STATE_INIT,
    .TRANSITION_TABLE_LENGTH = MSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(Motor_T * p_motor, statemachine_input_value_t faultFlags) { p_motor->FaultFlags.Value |= faultFlags; return &STATE_FAULT; }
static StateMachine_State_T * TransitionFreewheel(Motor_T * p_motor, statemachine_input_value_t _void)  { (void)p_motor; (void)_void; return &STATE_FREEWHEEL; }

/******************************************************************************/
/*!
    @brief States
        Proc on PWM thread, high priority
        Inputs on main thread, low priority
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Init_Entry(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_SetDirectionForward, Motor_SetDirectionForward); /* Eliminate circular inclusion from Motor_Init */
}

static void Init_Proc(Motor_T * p_motor)
{
    bool wait = true;

    // Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleVBemf, 0U); /* or Transition to STOP without hold */

    if(SysTime_GetMillis() > MOTOR_STATIC.INIT_WAIT) /* wait for Speed and Heat sensors */
    {
        wait = false;
        //(  Motor_CheckConfig() == true)    //check params
        Motor_PollAdcFaultFlags(p_motor); /* Clear the fault flags once */
        // if(p_motor->FaultFlags.Value != 0U) { wait = true; } // if wait for fault to clear
    }

    if (wait == false)
    {
        if (p_motor->FaultFlags.Value == 0U) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP); }
        else                                 { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_FAULT); }
    }
}

static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = 0U,
    [MSM_INPUT_CONTROL]         = 0U,
    [MSM_INPUT_RELEASE]         = 0U,
    [MSM_INPUT_DIRECTION]       = 0U,
    [MSM_INPUT_CALIBRATION]     = 0U,
};

static const StateMachine_State_T STATE_INIT =
{
    .ID                 = MSM_STATE_ID_INIT,
    .P_TRANSITION_TABLE = &INIT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Init_Entry,
    .LOOP               = (StateMachine_Function_T)Init_Proc,
};

/******************************************************************************/
/*!
    @brief Stop State

    Motor is in floating state with 0 speed
*/
/******************************************************************************/
/*
    Enters upon reaching 0 Speed
*/
static void Stop_Entry(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    p_motor->ControlTimerBase = 0U; /* ok to reset timer */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ClearControlState, 0U); /* Unobserved values remain 0 for user read */
}

static void Stop_Proc(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleVBemf, 0U);
    // and not hold // if(p_motor->Speed_Frac16 > 0U) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_FREEWHEEL); }
}

static StateMachine_State_T * Stop_InputDirection(Motor_T * p_motor, statemachine_input_value_t direction)
{
    if(p_motor->Speed_Frac16 == 0U)
    {
        Motor_SetCommutationModeUInt8(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, direction);
    }
    return 0U;
}

static StateMachine_State_T * Stop_InputControl(Motor_T * p_motor, statemachine_input_value_t feedbackMode)
{
    StateMachine_State_T * p_nextState;

    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    if(Motor_IsFeedbackAvailable(p_motor) == true)
    {
        Motor_ZeroSensor(p_motor);
        p_nextState = &STATE_RUN;
    }
    else
    {
        p_nextState = &STATE_OPEN_LOOP; /* ZeroSensors after OpenLoop */
    }

    return p_nextState;
}

static StateMachine_State_T * Stop_InputRelease(Motor_T * p_motor, statemachine_input_value_t _void)
{
    (void)_void;
    Phase_Float(&p_motor->Phase);
    return 0U;
}

static StateMachine_State_T * Stop_InputHold(Motor_T * p_motor, statemachine_input_value_t _void)
{
    (void)_void;
    Phase_Ground(&p_motor->Phase);
    return 0U;
}

static StateMachine_State_T * Stop_InputCalibration(Motor_T * p_motor, statemachine_input_value_t state)
{
    (void)p_motor;
    p_motor->CalibrationState = state;
    return &STATE_CALIBRATION;
}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (StateMachine_Transition_T)TransitionFault,
    [MSM_INPUT_CONTROL]     = (StateMachine_Transition_T)Stop_InputControl,
    [MSM_INPUT_RELEASE]     = (StateMachine_Transition_T)Stop_InputRelease,
    [MSM_INPUT_HOLD]        = (StateMachine_Transition_T)Stop_InputHold,
    [MSM_INPUT_DIRECTION]   = (StateMachine_Transition_T)Stop_InputDirection,
    [MSM_INPUT_CALIBRATION] = (StateMachine_Transition_T)Stop_InputCalibration,
};

static const StateMachine_State_T STATE_STOP =
{
    .ID                 = MSM_STATE_ID_STOP,
    .P_TRANSITION_TABLE = &STOP_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Stop_Entry,
    .LOOP               = (StateMachine_Function_T)Stop_Proc,
};

/******************************************************************************/
/*!
    @brief  Run State
    Active Control, FeedbackLoop is in effect
        UserCmd => RampOutput => PID => AngleControl
*/
/******************************************************************************/
static void Run_Entry(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcFeedbackMatch, 0U); /* Sync mode can match feedback here */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ActivateOutputState, 0U);
}

static void Run_Proc(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleControl, 0U/* Motor_SixStep_ProcPhaseControl */);
}

/*
    Prevent ProcAngleControl before ProcFeedbackMatch.
    alternatively transition through freewheel first
    Observed bemf may experience larger discontinuity than control voltage
*/
/* Block PWM Thread, do not proc new flags before matching output with StateMachine */
static StateMachine_State_T * Run_InputControl(Motor_T * p_motor, statemachine_input_value_t feedbackMode)
{
    StateMachine_State_T * p_nextState;

    if (feedbackMode != p_motor->FeedbackMode.Word)
    {
        Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
        p_nextState = &STATE_FREEWHEEL;
        /* Alternatively, without transition through Freewheel */
        // Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcFeedbackMatch, 0U);
        // p_nextState = &STATE_RUN; /* repeat entry function */
    }
    else
    {
        p_nextState = NULL;
    }

    return p_nextState;
}

static StateMachine_State_T * Run_InputRelease(Motor_T * p_motor, statemachine_input_value_t _void)
{
    (void)_void;
    return &STATE_FREEWHEEL;  // return (Motor_CheckSpeed(p_motor) == true) ? &STATE_FREEWHEEL : 0U; // check speed range
}

// static StateMachine_State_T * Run_InputHold(Motor_T * p_motor, statemachine_input_value_t _void)
// {
//     (void)_void;
//     StateMachine_State_T * p_nextState = NULL;
//     if(p_motor->Speed_Frac16 == 0U)
//     {
//        p_nextState = &STATE_STOP;
//     }
//     return 0U;
// }

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Transition_T)TransitionFault,
    [MSM_INPUT_RELEASE]         = (StateMachine_Transition_T)Run_InputRelease,
    [MSM_INPUT_CONTROL]         = (StateMachine_Transition_T)Run_InputControl,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Transition_T)0U, /* todo direction reverse match */
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_RUN =
{
    .ID                 = MSM_STATE_ID_RUN,
    .P_TRANSITION_TABLE = &RUN_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Run_Entry,
    .LOOP               = (StateMachine_Function_T)Run_Proc,
};

/******************************************************************************/
/*!
    @brief State
    Release Control,
*/
/******************************************************************************/
static void Freewheel_Entry(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ClearControlState, 0U);
}

static void Freewheel_Proc(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleVBemf, 0U /* Motor_SixStep_ProcPhaseObserve */);
    if (p_motor->Speed_Frac16 == 0U) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP); }
}

/* Match Feedback to ProcAngleBemf on Resume */
static StateMachine_State_T * Freewheel_InputControl(Motor_T * p_motor, statemachine_input_value_t feedbackMode)
{
    StateMachine_State_T * p_nextState;

    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);

    if (Motor_IsFeedbackAvailable(p_motor) == true) { p_nextState = &STATE_RUN; }
    else { p_nextState = NULL; /* OpenLoop does not resume */ }

    return p_nextState;
}

static const StateMachine_Transition_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Transition_T)TransitionFault,
    [MSM_INPUT_CONTROL]         = (StateMachine_Transition_T)Freewheel_InputControl,
    [MSM_INPUT_RELEASE]         = (StateMachine_Transition_T)0U,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Transition_T)0U,
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_FREEWHEEL =
{
    .ID                 = MSM_STATE_ID_FREEWHEEL,
    .P_TRANSITION_TABLE = &FREEWHEEL_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Freewheel_Entry,
    .LOOP               = (StateMachine_Function_T)Freewheel_Proc,
};

/******************************************************************************/
/*!
    @brief  State OpenLoop - OpenLoop, Align, and Start Up, Feedback Acquisition
*/
/******************************************************************************/
static void OpenLoop_Entry(Motor_T * p_motor)
{
    // switch(p_motor->Config.AlignMode)
    // {
    Motor_ProcCommutationMode(p_motor, Motor_FOC_StartAlign, 0U);
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    p_motor->OpenLoopState = MOTOR_OPEN_LOOP_STATE_ALIGN;
    // }
}

static void OpenLoop_Proc(Motor_T * p_motor)
{
    switch(p_motor->OpenLoopState)
    {
        case MOTOR_OPEN_LOOP_STATE_ALIGN:
            if(Timer_Periodic_Poll(&p_motor->ControlTimer) == false)
            {
                Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAlign, 0U);
            }
            else
            {
                // if(p_motor->Speed_Fixed32 != 0U) /* direct check sensor speed  todo */
                // {
                //     Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
                // }
                // else
                {
                    if(p_motor->Config.SensorMode == MOTOR_SENSOR_MODE_ENCODER)
                    {
                        Motor_ProcCommutationMode(p_motor, Motor_FOC_StartAlignValidate, 0U);
                        Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
                        p_motor->OpenLoopState = MOTOR_OPEN_LOOP_STATE_VALIDATE_ALIGN;
                    }
                    else if(p_motor->FeedbackMode.OpenLoop == 1U)
                    {
                        Motor_ProcCommutationMode(p_motor, Motor_FOC_StartOpenLoop, 0U /* Motor_SixStep_StartPhaseControl */);
                        p_motor->OpenLoopState = MOTOR_OPEN_LOOP_STATE_RUN;
                    }
                }
            }
            break;
        case MOTOR_OPEN_LOOP_STATE_VALIDATE_ALIGN: /* ClosedLoop Sensor Start Up - OpenLoop Feedback Mode, UserRamp as Torque or Voltage */
            if(Timer_Periodic_Poll(&p_motor->ControlTimer) == false)
            {
                Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleControl, 0U);
                if(Motor_PollAlignFault(p_motor) == true) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_FAULT); }
            }
            else
            {
                Motor_ValidateSensorAlign(p_motor);
                _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_RUN);
            }
            break;
        case MOTOR_OPEN_LOOP_STATE_RUN: /* OpenLoop */
            if(Motor_IsFeedbackAvailable(p_motor) == true)
            {
                _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_RUN);
            }
            else
            {
                Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcOpenLoop, 0U /* Motor_SixStep_ProcPhaseControl */);
            }
            break;
        default: break;
    }
}


// static StateMachine_State_T * OpenLoop_InputCmdValue(Motor_T * p_motor, statemachine_input_value_t ivCmd)
// {
//     int32_t ivCmd_Positive = math_clamp((int32_t)ivCmd, 0, (int32_t)p_motor->Config.OpenLoopPower_Percent16 / 2);
//     Motor_SetDirectionalCmd(p_motor, ivCmd_Positive);
//     return 0U;
// }

static const StateMachine_Transition_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Transition_T)TransitionFault,
    [MSM_INPUT_RELEASE]         = (StateMachine_Transition_T)TransitionFreewheel, /* No resume from OpenLoop, freewheel state check stop */
    [MSM_INPUT_CONTROL]         = (StateMachine_Transition_T)0U,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Transition_T)0U,
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_OPEN_LOOP =
{
    .ID                 = MSM_STATE_ID_OPEN_LOOP,
    .P_TRANSITION_TABLE = &OPEN_LOOP_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)OpenLoop_Entry,
    .LOOP               = (StateMachine_Function_T)OpenLoop_Proc,
};

/******************************************************************************/
/*!
    @brief Calibration State
*/
/******************************************************************************/
static void Calibration_Entry(Motor_T * p_motor)
{
    p_motor->ControlTimerBase = 0U;
    p_motor->CalibrationStateIndex = 0U;
    Phase_Ground(&p_motor->Phase);

    switch(p_motor->CalibrationState)
    {
        case MOTOR_CALIBRATION_STATE_ADC:       Motor_Calibration_StartAdc(p_motor);      break;
        case MOTOR_CALIBRATION_STATE_HALL:      Motor_Calibration_StartHall(p_motor);     break;
        case MOTOR_CALIBRATION_STATE_ENCODER:   Motor_Calibration_StartEncoder(p_motor);  break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_CALIBRATION_STATE_SIN_COS:   Motor_Calibration_StartSinCos(p_motor);   break;
        #endif
        default: break;
    }
}

static void Calibration_Proc(Motor_T * p_motor)
{
    bool isComplete = false;

    switch(p_motor->CalibrationState)
    {
        case MOTOR_CALIBRATION_STATE_ADC:       isComplete = Motor_Calibration_ProcAdc(p_motor);      break;
        case MOTOR_CALIBRATION_STATE_HALL:      isComplete = Motor_Calibration_ProcHall(p_motor);     break;
        case MOTOR_CALIBRATION_STATE_ENCODER:   isComplete = Motor_Calibration_ProcEncoder(p_motor);  break;
        #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
        case MOTOR_CALIBRATION_STATE_SIN_COS:   isComplete = Motor_Calibration_ProcSinCos(p_motor);   break;
        #endif
        default: break;
    }

    if(isComplete == true) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP); }
}

static StateMachine_State_T * Calibration_InputRelease(Motor_T * p_motor, statemachine_input_value_t _void)
{
    (void)p_motor; (void)_void;
    return &STATE_STOP;
}

static const StateMachine_Transition_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (StateMachine_Transition_T)TransitionFault,
    [MSM_INPUT_RELEASE]     = (StateMachine_Transition_T)TransitionFreewheel,
    [MSM_INPUT_CONTROL]     = (StateMachine_Transition_T)0U,
    [MSM_INPUT_DIRECTION]   = (StateMachine_Transition_T)0U,
    [MSM_INPUT_CALIBRATION] = (StateMachine_Transition_T)0U,
};

static const StateMachine_State_T STATE_CALIBRATION =
{
    .ID                 = MSM_STATE_ID_CALIBRATION,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Calibration_Entry,
    .LOOP               = (StateMachine_Function_T)Calibration_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(Motor_T * p_motor) { Phase_Float(&p_motor->Phase); }

static void Fault_Proc(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    if(p_motor->FaultFlags.Value == 0U) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP); }
}

static StateMachine_State_T * Fault_InputClearFault(Motor_T * p_motor, statemachine_input_value_t faultFlags)
{
    // p_motor->FaultFlags.Value &= ~faultFlags;
    Motor_PollAdcFaultFlags(p_motor); // p_motor->FaultFlags.Overheat = Thermistor_IsFault(&p_motor->Thermistor);
    p_motor->FaultFlags.AlignStartUp = 0U;
    return NULL; /* Transition on proc if no flags remain active */
    // return (p_motor->FaultFlags.Value == 0U) ? &STATE_STOP : 0U;
}

static StateMachine_State_T * Fault_InputAll(Motor_T * p_motor, statemachine_input_value_t _void)
{
    (void)_void;
    Phase_Float(&p_motor->Phase);
    return NULL;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (StateMachine_Transition_T)Fault_InputClearFault,
    [MSM_INPUT_CONTROL]     = (StateMachine_Transition_T)Fault_InputAll,
    [MSM_INPUT_RELEASE]     = (StateMachine_Transition_T)Fault_InputAll,
    [MSM_INPUT_HOLD]        = (StateMachine_Transition_T)Fault_InputAll,
    [MSM_INPUT_DIRECTION]   = (StateMachine_Transition_T)Fault_InputAll,
    [MSM_INPUT_CALIBRATION] = (StateMachine_Transition_T)Fault_InputAll,
};

static const StateMachine_State_T STATE_FAULT =
{
    .ID                 = MSM_STATE_ID_FAULT,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Fault_Entry,
    .LOOP               = (StateMachine_Function_T)Fault_Proc,
};



