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


/******************************************************************************/
/*!
    @brief State Machine
*/
/******************************************************************************/
const StateMachine_Machine_T MSM_MACHINE =
{
    .P_STATE_INITIAL = &MOTOR_STATE_INIT,
    .TRANSITION_TABLE_LENGTH = MSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(Motor_T * p_motor, state_machine_value_t faultFlags) { return &MOTOR_STATE_FAULT; }

/* clears fault of type adc reading */
static inline void Motor_PollAdcFaultFlags(Motor_T * p_motor) { p_motor->FaultFlags.Overheat = Thermistor_IsFault(&p_motor->Thermistor); }

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
    /* alternatively null until a direction is set */
    /* Sets Speed/I Limits using direction */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_SetDirectionForward, Motor_SetDirectionForward);
}

static void Init_Proc(Motor_T * p_motor)
{
    bool wait = true;

    // Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL);

    if (SysTime_GetMillis() > MOTOR_STATIC.INIT_WAIT) /* wait for Speed and Heat sensors */
    {
        wait = false;
        // (Motor_CheckConfig() == true)    // check params
        p_motor->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor);
        Motor_PollAdcFaultFlags(p_motor); /* Clear the fault flags once */
    }

    if (wait == false)
    {
        if (p_motor->FaultFlags.Value != 0U) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_FAULT); }
        else
        {
            if (p_motor->Speed_Fract16 > 0) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL); }
            else                            { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_STOP); }
        }
    }

}

static const StateMachine_Input_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = NULL,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const StateMachine_State_T MOTOR_STATE_INIT =
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
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ClearFeedbackState, NULL); /* Unobserved values remain 0 for user read */
    p_motor->ControlTimerBase = 0U; /* ok to reset timer */
}

static void Stop_Proc(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL);
    // if (p_motor->Speed_Fract16 > 0)
    // {
    //     if (Motor_IsHold(p_motor) == false) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL); }
    //     else                                { p_motor->StateFlags.Alarm = 1U;}
    // }
}

static StateMachine_State_T * Stop_InputDirection(Motor_T * p_motor, state_machine_value_t direction)
{
    // Motor_SetCommutationModeUInt8(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, direction);
    // Motor_CommutationModeFn(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast)(p_motor, direction);
    Motor_CommutationModeFn_Call(p_motor, Motor_FOC_SetDirection_Cast, Motor_SetDirection_Cast, direction);

    return NULL;
}

static StateMachine_State_T * Stop_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:     Phase_Float(&p_motor->Phase);   break;
        case PHASE_OUTPUT_GROUND:    Phase_Ground(&p_motor->Phase);  break;
        case PHASE_OUTPUT_VPWM:
            if (Motor_IsClosedLoopStart(p_motor) == true)
            {
                Motor_ZeroSensor(p_motor); // todo as match or clear
                p_nextState = &MOTOR_STATE_RUN;
            }
            else
            {
                p_nextState = &MOTOR_STATE_OPEN_LOOP; /* ZeroSensors after OpenLoop */
            }
            break;
    }

    return p_nextState;
}

static StateMachine_State_T * Stop_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}

/* Transition for user input */
static StateMachine_State_T * Stop_InputOpenLoop(Motor_T * p_motor, state_machine_value_t state)
{
    // p_motor->OpenLoopState = MOTOR_OPEN_LOOP_STATE_PASSIVE;
    // return ((Motor_OpenLoopState_T)state == MOTOR_OPEN_LOOP_STATE_ENTER) ? &MOTOR_STATE_OPEN_LOOP : NULL;

    // StateMachine_ValidateSubState(p_motor->StateMachine, &MOTOR_STATE_OPEN_LOOP, (StateMachine_State_T *)state);
    return &MOTOR_STATE_OPEN_LOOP;
}

static StateMachine_State_T * Stop_InputCalibration(Motor_T * p_motor, state_machine_value_t state)
{
    // StateMachine_State_T * p_subState = (StateMachine_State_T *)state;

    // return (p_subState != NULL) ? p_subState : NULL;

    // p_motor->CalibrationState = state;
    return &MOTOR_STATE_CALIBRATION;
}

static const StateMachine_Input_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Stop_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Stop_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = (StateMachine_Input_T)Stop_InputDirection,
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Input_T)Stop_InputCalibration,
    [MSM_INPUT_OPEN_LOOP]       = (StateMachine_Input_T)Stop_InputOpenLoop,
};

const StateMachine_State_T MOTOR_STATE_STOP =
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
    /* Optionally poll angle sensor or let interpolate angle be off by 1 */
    // Motor_UpdateSpeedControlLimits(p_motor); /* Alternatively run in SetFeedbackMode does not need to updated unless feedback mode changed */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_MatchFeedbackState, NULL);
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ActivateOutput, NULL);
}

static void Run_Proc(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleControl, NULL/* Motor_SixStep_ProcPhaseControl */);
}

/*
    StateMachine in Sync mode, [ProcInput] in the same thread as [Run_Proc]/[ProcAngleControl]
    Process [Motor_FOC_MatchFeedbackState] before [Motor_FOC_ProcAngleControl]
*/
static StateMachine_State_T * Run_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:
            //   (Motor_CheckSpeed(p_motor) == true) ? &MOTOR_STATE_FREEWHEEL : 0U; // check speed range
            p_nextState = &MOTOR_STATE_FREEWHEEL;
            break;
        case PHASE_OUTPUT_GROUND:
            // if (p_motor->Speed_Fract16 == 0U) { Phase_Ground ; }
            break;
        case PHASE_OUTPUT_VPWM: break;
    }

    return p_nextState;
}

static StateMachine_State_T * Run_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    StateMachine_State_T * p_nextState = NULL;

    if (feedbackMode != p_motor->FeedbackMode.Value)
    {
        // if openloop return freewheel
        Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
        p_nextState = &MOTOR_STATE_RUN; /* repeat entry function */ /* Alternatively, transition through Freewheel */
    }

    return p_nextState;
}

static const StateMachine_Input_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Run_InputFeedbackMode,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Run_InputControl,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const StateMachine_State_T MOTOR_STATE_RUN =
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
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ClearFeedbackState, NULL);
}

static void Freewheel_Proc(Motor_T * p_motor)
{
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcCaptureAngleVBemf, NULL /* Motor_SixStep_ProcPhaseObserve */);
    // alternatively wait for input
    if (p_motor->Speed_Fract16 == 0U) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_STOP); }

}

static StateMachine_State_T * Freewheel_Next(Motor_T * p_motor)
{
    return (p_motor->Speed_Fract16 == 0U) ? &MOTOR_STATE_STOP : NULL;
}

/* Match Feedback to ProcAngleBemf on Resume */
static StateMachine_State_T * Freewheel_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:  break;
        case PHASE_OUTPUT_GROUND:
            // if (p_motor->Speed_Fract16 == 0U) { Phase_Ground p_nextState = &MOTOR_STATE_STOP; }
            break;
        case PHASE_OUTPUT_VPWM:
            if (Motor_IsClosedLoopStart(p_motor) == true) { p_nextState = &MOTOR_STATE_RUN; } /* If flags set */
            /* OpenLoop does not resume */
            break;
    }

    return p_nextState;
}

static StateMachine_State_T * Freewheel_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode);
    return NULL;
}

static const StateMachine_Input_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Freewheel_InputControl,
    [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)Freewheel_InputFeedbackMode,
    [MSM_INPUT_DIRECTION]       = NULL,
    [MSM_INPUT_CALIBRATION]     = NULL,
};

const StateMachine_State_T MOTOR_STATE_FREEWHEEL =
{
    .ID                 = MSM_STATE_ID_FREEWHEEL,
    .P_TRANSITION_TABLE = &FREEWHEEL_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Freewheel_Entry,
    .LOOP               = (StateMachine_Function_T)Freewheel_Proc,
};

/******************************************************************************/
/*!
    @brief  State OpenLoop - OpenLoop, Align, and Start Up, Feedback Acquisition

    Only Entry is from [STOP] State
*/
/******************************************************************************/
static void OpenLoop_Entry(Motor_T * p_motor)
{
    p_motor->FeedbackMode.OpenLoop = 1U; /* limits user cmd input */
    Phase_Ground(&p_motor->Phase);
    // FOC_ClearControlState(&p_motor->Foc);

    _StateMachine_EndSubState(&p_motor->StateMachine); /* 'unmount' last operation */
    // determine initial substate from stop
}


static void OpenLoop_Proc(Motor_T * p_motor)
{
    _StateMachine_ProcSubState(&p_motor->StateMachine);
}

/* as motor state */
/* maintain consistent interface with other states, use substate cmd for phase state without exiting */
static StateMachine_State_T * OpenLoop_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    switch ((Phase_Mode_T)phaseMode)
    {
        case PHASE_OUTPUT_FLOAT:  p_nextState = &MOTOR_STATE_FREEWHEEL;          break;
        case PHASE_OUTPUT_GROUND: Phase_Ground(&p_motor->Phase);                 break;
        case PHASE_OUTPUT_VPWM: /* Motor_FOC_ActivateOutputZero(p_motor); */   break;
        /* No resume from OpenLoop, freewheel state check stop */
    }

    return p_nextState;
}

static StateMachine_State_T * OpenLoop_InputFeedbackMode(Motor_T * p_motor, state_machine_value_t feedbackMode)
{
    Motor_SetFeedbackMode_Cast(p_motor, feedbackMode); /* a different flag mode will change ramp limits */
    return (p_motor->FeedbackMode.OpenLoop == 0U) ? &MOTOR_STATE_FREEWHEEL : NULL;
    // return NULL;
}

/*
    using openloop substate inputs
*/
static StateMachine_State_T * OpenLoop_InputOpenLoop(Motor_T * p_motor, state_machine_value_t openLoop)
{
    StateMachine_State_T * p_nextState = NULL;

    _StateMachine_EndSubState(&p_motor->StateMachine);

    switch (openLoop)
    {

    }

    return p_nextState;
}

const StateMachine_State_T MOTOR_STATE_OPEN_LOOP =
{
    .ID                 = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY              = (StateMachine_Function_T)OpenLoop_Entry,
    .LOOP               = (StateMachine_Function_T)OpenLoop_Proc,

    .P_TRANSITION_TABLE = (StateMachine_Input_T[MSM_TRANSITION_TABLE_LENGTH])
    {
        [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
        [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)OpenLoop_InputControl,
        [MSM_INPUT_FEEDBACK_MODE]   = (StateMachine_Input_T)OpenLoop_InputFeedbackMode,
        [MSM_INPUT_OPEN_LOOP]       = (StateMachine_Input_T)OpenLoop_InputOpenLoop,
        [MSM_INPUT_DIRECTION]       = NULL,
        [MSM_INPUT_CALIBRATION]     = NULL,
    },

    .P_PARENT           = NULL,
};


/******************************************************************************/
/*
    Open Loop SubStates/Cmds
*/
/******************************************************************************/
/*
    Angle Cmd and User Current/Voltage Cmd
*/
static const StateMachine_State_T OPEN_LOOP_STATE_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartAlignCmd,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcAlignCmd,
    .NEXT       = NULL,
};

static StateMachine_State_T * OpenLoop_StartUpAlign_Transition(Motor_T * p_motor); // todo

/* Align with Aux Ramp */
static const StateMachine_State_T OPEN_LOOP_STATE_START_UP_ALIGN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartStartUpAlign,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcStartUpAlign,
    .NEXT       = (StateMachine_Transition_T)OpenLoop_StartUpAlign_Transition,
};


static const StateMachine_State_T OPEN_LOOP_STATE_RUN =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartOpenLoop,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcOpenLoop,
    .NEXT       = NULL,
};


/******************************************************************************/
/* */
/******************************************************************************/
static void OpenLoop_StartUpAlign_EntryTimer(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
}

static StateMachine_State_T * OpenLoop_StartUpAlign_Transition(Motor_T * p_motor)
{
    return (Timer_Periodic_Poll(&p_motor->ControlTimer) == true) ? &OPEN_LOOP_STATE_RUN : NULL;
}

//test branch
static const StateMachine_State_T OPEN_LOOP_STATE_RUN_START_UP =
{
    // .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &OPEN_LOOP_STATE_START_UP_ALIGN,
    .DEPTH      = 2U,
    .ENTRY      = (StateMachine_Function_T)OpenLoop_StartUpAlign_EntryTimer,
    .NEXT       = (StateMachine_Transition_T)OpenLoop_StartUpAlign_Transition,
};


/******************************************************************************/
/*
    sets the phase state without exiting openloop
*/
/******************************************************************************/
static void OpenLoop_CmdPhaseControl(Motor_T * p_motor, state_machine_value_t phaseState)
{
    Motor_FOC_ClearFeedbackState(p_motor);
    Phase_ActivateOutputState(&p_motor->Phase, (Phase_Output_T)phaseState);
    // _StateMachine_EndSubState(&p_motor->StateMachine); /* No periodic  */
}

/*

*/
void Motor_OpenLoop_SetPhaseState(Motor_T * p_motor, Phase_Output_T phase)
{
    static const StateMachine_Cmd_T OPEN_LOOP_CMD_PHASE = { .CMD = (StateMachine_CmdInput_T)OpenLoop_CmdPhaseControl, .P_INITIAL = &MOTOR_STATE_OPEN_LOOP, };
    StateMachine_StartCmd(&p_motor->StateMachine, &OPEN_LOOP_CMD_PHASE, phase);
}

/******************************************************************************/
/*
    Activate Align
*/
/******************************************************************************/
static void OpenLoop_CmdPhaseAlign(Motor_T * p_motor, state_machine_value_t phaseAlign)
{
    Phase_ActivateOutput(&p_motor->Phase);
    Phase_Align(&p_motor->Phase, (Phase_Id_T)phaseAlign, Ramp_GetTarget(&p_motor->Ramp)); /* limited by p_motor->Config.AlignPower_UFract16 */

    /* Set state variables for User Interface consistency */
    Ramp_SetOutputState(&p_motor->Ramp, Ramp_GetTarget(&p_motor->Ramp));
    p_motor->ElectricalAngle = Phase_AngleOf((Phase_Id_T)phaseAlign);
}

void Motor_OpenLoop_SetPhaseVAlign(Motor_T * p_motor, Phase_Id_T align)
{
    static const StateMachine_Cmd_T OPEN_LOOP_CMD_V_ALIGN = { .CMD = (StateMachine_CmdInput_T)OpenLoop_CmdPhaseAlign, .P_INITIAL = &MOTOR_STATE_OPEN_LOOP, };
    StateMachine_StartCmd(&p_motor->StateMachine, &OPEN_LOOP_CMD_V_ALIGN, align);
}

/******************************************************************************/
/* */
/******************************************************************************/
static void OpenLoop_CmdJog(Motor_T * p_motor, state_machine_value_t direction)
{
    Ramp_SetOutputState(&p_motor->Ramp, Ramp_GetTarget(&p_motor->Ramp));
    // Ramp_ProcEndState(&p_motor->Ramp);
    p_motor->ElectricalAngle = Phase_AngleOf(Phase_JogDirection(&p_motor->Phase, Ramp_GetTarget(&p_motor->Ramp), 1));
}

void Motor_OpenLoop_SetJog(Motor_T * p_motor, int8_t direction)
{
    static const StateMachine_Cmd_T OPEN_LOOP_CMD_JOG = { .CMD = (StateMachine_CmdInput_T)OpenLoop_CmdJog, .P_INITIAL = &MOTOR_STATE_OPEN_LOOP, };
    StateMachine_StartCmd(&p_motor->StateMachine, &OPEN_LOOP_CMD_JOG, direction);
}

/******************************************************************************/
/*
    intensity using user ramp
    feedback align angle
*/
/******************************************************************************/
/* ramp isolated from userCmd   */
// void OpenLoop_CmdAlign(Motor_T * p_motor, state_machine_value_t ivCmd)
// {
//     Ramp_SetTarget(&p_motor->Ramp, Motor_DirectionalValueOf(p_motor, math_clamp(ivCmd, 0, p_motor->Config.OpenLoopPower_UFract16)));
// }

static void OpenLoop_CmdAngleAlign(Motor_T * p_motor, state_machine_value_t angle)
{
    p_motor->ElectricalAngle = angle;
}

void Motor_OpenLoop_SetAngleAlign(Motor_T * p_motor, angle16_t angle)
{
    static const StateMachine_Cmd_T OPEN_LOOP_CMD_ANGLE_ALIGN = { .CMD = (StateMachine_CmdInput_T)OpenLoop_CmdAngleAlign, .P_INITIAL = &OPEN_LOOP_STATE_ALIGN, };
    StateMachine_StartCmd(&p_motor->StateMachine, &OPEN_LOOP_CMD_ANGLE_ALIGN, angle);
}


/******************************************************************************/
/* */
/******************************************************************************/
void Motor_OpenLoop_StartRunChain(Motor_T * p_motor)
{
    static const StateMachine_Cmd_T OPEN_LOOP_CMD_RUN = { .CMD = NULL, .P_INITIAL = &OPEN_LOOP_STATE_START_UP_ALIGN, };
    StateMachine_StartCmd(&p_motor->StateMachine, &OPEN_LOOP_CMD_RUN, 0);
}



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

    // switch (p_motor->CalibrationState)
    // {
    //     case MOTOR_CALIBRATION_STATE_DISABLE:  break;
    //     case MOTOR_CALIBRATION_STATE_ADC:       Motor_Analog_StartCalibration(p_motor);   break;
    //     /* Sensor */
    //     case MOTOR_CALIBRATION_STATE_HALL:      Motor_Calibration_StartHall(p_motor);     break;
    //     // case MOTOR_CALIBRATION_STATE_ENCODER:   Motor_Calibration_StartEncoderDirection(p_motor);  break;
    //     #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    //     case MOTOR_CALIBRATION_STATE_SIN_COS:   Motor_Calibration_StartSinCos(p_motor);   break;
    //     #endif
    //     // case MOTOR_CALIBRATION_STATE_INPUT:      break;
    //     default: break;
    // }
}

static void Calibration_Proc(Motor_T * p_motor)
{
    _StateMachine_ProcSubState(&p_motor->StateMachine);

    // StateMachine_State_T * p_nextState = NULL;
    // bool isComplete = false;

    // switch (p_motor->CalibrationState)
    // {
    //     case MOTOR_CALIBRATION_STATE_IDLE:      _StateMachine_ProcSubState(&p_motor->StateMachine);     break;
    //     case MOTOR_CALIBRATION_STATE_DISABLE:   isComplete = true;                                      break;
    //     case MOTOR_CALIBRATION_STATE_ADC:       isComplete = Motor_Analog_ProcCalibration(p_motor);     break;

    //     case MOTOR_CALIBRATION_STATE_HALL:
    //         isComplete = Motor_Calibration_ProcHall(p_motor);
    //         if (isComplete == true) { p_motor->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor); }
    //         break;

    //     // case MOTOR_CALIBRATION_STATE_ENCODER:
    //     //     isComplete = Motor_Calibration_ProcEncoderDirection(p_motor);
    //     //     if (isComplete == true) { p_motor->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor); }
    //         break;

    //     #if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
    //     case MOTOR_CALIBRATION_STATE_SIN_COS:   isComplete = Motor_Calibration_ProcSinCos(p_motor);     break;
    //     #endif
    //     default: break;
    // }

    // if (isComplete == true)
    // {
    //     p_nextState = (p_motor->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : &MOTOR_STATE_STOP;
    //     p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_DISABLE;
    // }
}

static StateMachine_State_T * Calibration_Next(Motor_T * p_motor)
{
    return (p_motor->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : NULL;
    // if (isComplete == true)
    // {
    //     p_nextState = (p_motor->FaultFlags.Value != 0U) ? &MOTOR_STATE_FAULT : &MOTOR_STATE_STOP;
    //     p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_DISABLE;
    // }

    // return p_nextState;

    // return (p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_DISABLE) ? &MOTOR_STATE_STOP : NULL;
}


/* Release on all input values */
static StateMachine_State_T * Calibration_InputControl(Motor_T * p_motor, state_machine_value_t phaseMode)
{
    StateMachine_State_T * p_nextState = NULL;

    // switch ((Phase_Mode_T)phaseMode)
    // {
    //     case PHASE_OUTPUT_FLOAT:    Phase_Float(&p_motor->Phase);  p_nextState = &MOTOR_STATE_CALIBRATION; break; /* or fault */
    //     case PHASE_OUTPUT_GROUND:   Phase_Ground(&p_motor->Phase);  break;
    //     case PHASE_OUTPUT_VPWM: break;
    // }

    return &MOTOR_STATE_STOP;
    return NULL;
}

static StateMachine_State_T * Calibration_InputCalibration(Motor_T * p_motor, state_machine_value_t state)
{
    StateMachine_State_T * input = (StateMachine_State_T *)state;
    if (input == 0U) return &MOTOR_STATE_STOP;
    else if (StateMachine_IsActiveBranch(&p_motor->StateMachine, input)) { return input; }
    else { return NULL; }
    // return (state != &MOTOR_STATE_CALIBRATION) ? &MOTOR_STATE_STOP : NULL;
    // return &MOTOR_STATE_STOP;
}

static const StateMachine_Input_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]           = (StateMachine_Input_T)TransitionFault,
    [MSM_INPUT_CONTROL_STATE]   = (StateMachine_Input_T)Calibration_InputControl,
    [MSM_INPUT_CALIBRATION]     = (StateMachine_Input_T)Calibration_InputCalibration,
    [MSM_INPUT_DIRECTION]       = NULL,
};

const StateMachine_State_T MOTOR_STATE_CALIBRATION =
{
    .ID                 = MSM_STATE_ID_CALIBRATION,
    .P_TRANSITION_TABLE = &CALIBRATION_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Calibration_Entry,
    .LOOP               = (StateMachine_Function_T)Calibration_Proc,
};

// static void Calibration_SetIdle(Motor_T * p_motor, state_machine_value_t state)
// {
//     p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_IDLE;
//     Phase_Float(&p_motor->Phase);
// }

// static const StateMachine_Cmd_T CALIBRATION_CMD_IDLE = { .STATE = MSM_STATE_ID_CALIBRATION, .CMD = (StateMachine_CmdInput_T)Calibration_SetIdle, .LOOP = NULL, };

// void Motor_Calibration_SetIdle(Motor_T * p_motor)
// {
//     // StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_IDLE);
//     StateMachine_SetSubState(&p_motor->StateMachine, &CALIBRATION_CMD_IDLE, 0);
// }

/******************************************************************************/
/*!  */
/******************************************************************************/
/*
    with position sensor without position feedback loop
*/
static void Calibration_CmdHome(Motor_T * p_motor)
{
    //for now
    p_motor->ControlTimerBase = 0U;
    p_motor->CalibrationStateIndex = 0U;
    p_motor->FeedbackMode.Current = 0U;

    Phase_ActivateOutput(&p_motor->Phase);
    Timer_StartPeriod_Millis(&p_motor->ControlTimer, 20); // ~1rpm
    Ramp_Set(&p_motor->AuxRamp, p_motor->Config.RampAccel_Cycles, 0, Motor_DirectionalValueOf(p_motor, p_motor->Config.OpenLoopPower_UFract16));

    p_motor->ElectricalAngle = Motor_PollSensorAngle(p_motor);
    p_motor->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);
}

//todo curent voltage speed calc
static void Calibration_ProcHome(Motor_T * p_motor)
{
    // uint16_t angleDelta = Encoder_GetHomingAngle(&p_motor->Encoder); // * direction
    /* alternatively use openloop speed */
    uint16_t angleDelta = 65536/1000;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        Motor_PollSensorAngle(p_motor); /*  */

        // p_motor->ElectricalAngle = (Motor_GetMechanicalAngle(p_motor) + angleDelta) * p_motor->Config.PolePairs;
        p_motor->ElectricalAngle += (angleDelta * p_motor->Config.PolePairs);
        // Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, Ramp_ProcOutput(&p_motor->AuxRamp), 0);
        Motor_FOC_ProcAngleFeedforward(p_motor, p_motor->ElectricalAngle, p_motor->Config.OpenLoopPower_UFract16 * 2, 0);
    }
}

static StateMachine_State_T * Calibration_HomeTransition(Motor_T * p_motor)
{
    uint16_t angleDelta = 65536 / 1000;

    /* error on full rev todo */
    if (angle16_cycle(p_motor->MechanicalAngle, Motor_GetMechanicalAngle(p_motor), (p_motor->Direction == MOTOR_DIRECTION_CCW)) == true)
    {
        Phase_Ground(&p_motor->Phase);
        // _StateMachine_EndSubState(&p_motor->StateMachine);
    }

    p_motor->MechanicalAngle = Motor_GetMechanicalAngle(p_motor);

    return NULL;
}

static const StateMachine_State_T CALIBRATION_STATE_HOMING =
{
    // .ID         = MSM_STATE_ID_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Calibration_CmdHome,
    .LOOP       = (StateMachine_Function_T)Calibration_ProcHome,
    .NEXT       = (StateMachine_Transition_T)Calibration_HomeTransition,
};

void Motor_Calibration_StartHome(Motor_T * p_motor)
{
    static const StateMachine_Cmd_T CALIBRATION_CMD_HOME = { .CMD = NULL, .P_INITIAL = &CALIBRATION_STATE_HOMING, };
    StateMachine_StartCmd(&p_motor->StateMachine, &CALIBRATION_CMD_HOME, 0); /* substate only reachable after setting calib first */
}


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void Fault_Entry(Motor_T * p_motor) { Phase_Float(&p_motor->Phase); }

static void Fault_Proc(Motor_T * p_motor)
{
    Phase_Float(&p_motor->Phase);
    /* If auto exit Fault State */
    if (p_motor->FaultFlags.Value == 0U) { _StateMachine_SetState(&p_motor->StateMachine, &MOTOR_STATE_STOP); }
}

static StateMachine_State_T * Fault_Next(Motor_T * p_motor)
{
    return (p_motor->FaultFlags.Value == 0U) ? &MOTOR_STATE_STOP : NULL;
}

static StateMachine_State_T * Fault_InputClearFault(Motor_T * p_motor, state_machine_value_t faultFlags)
{
    // p_motor->FaultFlags.Value &= ~faultFlags;
    p_motor->FaultFlags.Value = 0U;
    p_motor->FaultFlags.PositionSensor = !Motor_VerifySensorCalibration(p_motor); // non polling check
    // Motor_PollAdcFaultFlags(p_motor); // p_motor->FaultFlags.Overheat = Thermistor_IsFault(&p_motor->Thermistor);
    return NULL; /* Transition on proc if no flags remain active */
    // return (p_motor->FaultFlags.Value == 0U) ? &MOTOR_STATE_STOP : 0U;
}

static StateMachine_State_T * Fault_InputCalibration(Motor_T * p_motor, state_machine_value_t state)
{
    StateMachine_State_T * p_nextState = NULL;

    if (p_motor->FaultFlags.Overheat == 0U)
    {
        p_motor->CalibrationState = state;
        p_nextState = &MOTOR_STATE_CALIBRATION;
    }

    return p_nextState;
}

static const StateMachine_Input_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
    [MSM_INPUT_FAULT]       = (StateMachine_Input_T)Fault_InputClearFault,
    [MSM_INPUT_DIRECTION]   = NULL,
    [MSM_INPUT_CALIBRATION] = (StateMachine_Input_T)Fault_InputCalibration,
};

const StateMachine_State_T MOTOR_STATE_FAULT =
{
    .ID                 = MSM_STATE_ID_FAULT,
    .P_TRANSITION_TABLE = &FAULT_TRANSITION_TABLE[0U],
    .ENTRY              = (StateMachine_Function_T)Fault_Entry,
    .LOOP               = (StateMachine_Function_T)Fault_Proc,
};


/******************************************************************************/
/* Fault interface functions */
/******************************************************************************/

bool Motor_StateMachine_IsFault(const Motor_T * p_motor) { return (StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT); }

void Motor_StateMachine_EnterFault(Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, 0U); }
}

/*! @return true if no fault remains */
bool Motor_StateMachine_ExitFault(Motor_T * p_motor)
{
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, 0U); }
    return Motor_StateMachine_IsFault(p_motor);
}

void Motor_StateMachine_SetFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    p_motor->FaultFlags.Value |= faultFlags.Value;
    if (Motor_StateMachine_IsFault(p_motor) == false) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, faultFlags.Value); }
}

/*! @return true if cleared applies, fault to non fault */
bool Motor_StateMachine_ClearFault(Motor_T * p_motor, Motor_FaultFlags_T faultFlags)
{
    bool isFault = Motor_StateMachine_IsFault(p_motor);
    if (Motor_StateMachine_IsFault(p_motor) == true) { StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT, faultFlags.Value); }
    return (Motor_StateMachine_IsFault(p_motor) != isFault);
}
