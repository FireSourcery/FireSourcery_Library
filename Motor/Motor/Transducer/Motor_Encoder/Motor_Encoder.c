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
    @file   Motor_Encoder.c
    @author FireSourcery
    @version V0
    @brief
*/
/******************************************************************************/
#include "Motor_Encoder.h"



/* todo error status */
// static inline bool ProcHoming(Motor_T * p_motor)
// {
//     bool isComplete = false;
//     uint16_t angle = Encoder_GetHomingAngle(&p_motor->Encoder) * p_motor->Config.PolePairs;

//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         if (Encoder_PollHomingComplete(&p_motor->Encoder) == true)
//         {
//             // Encoder_CalibrateQuadratureDirection(&p_motor->Encoder, p_motor->Direction == MOTOR_DIRECTION_CCW);
//             isComplete = true;
//         }
//         else
//         {
//             //openloop speed ramp
//             Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle + angle, p_motor->Config.AlignPower_UFract16, 0);
//         }
//     }

//     return isComplete;
// }

// static void Calibration_ProcEncoderHoming(Motor_T * p_motor)
// {
//     if (ProcHoming(p_motor) == true)
//     {
//         _StateMachine_EndSubState(&p_motor->StateMachine);

//         // Motor_Calibration_SetIdle(p_motor);
//         // StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_IDLE);
//         StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_STATE_FLOAT); /* or exit */
//     }
// }

/*

*/
static void StartHoming(Motor_T * p_motor)
{
    Timer_StartPeriod_Millis(&p_motor->ControlTimer, 20); //~1rpm
    Encoder_StartHoming(&p_motor->Encoder);
    p_motor->ElectricalAngle = 0U;
}

static void ProcHoming(Motor_T * p_motor)
{
    uint16_t angle;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        angle = Encoder_GetHomingAngle(&p_motor->Encoder) * p_motor->Config.PolePairs;
        Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle + angle, p_motor->Config.AlignPower_UFract16, 0);
    }
}

static StateMachine_State_T * HomingTransition(Motor_T * p_motor)
{
    if (Encoder_PollHomingComplete(&p_motor->Encoder) == true)
    {
        // Encoder_CalibrateQuadratureDirection(&p_motor->Encoder, p_motor->Direction == MOTOR_DIRECTION_CCW);
        _StateMachine_EndSubState(&p_motor->StateMachine);
        StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_STATE_FLOAT);
        // return &STATE_STOP;
    }

    return NULL;
}


static const StateMachine_State_T STATE_ENCODER_HOMING = {
    // .ID         = MSM_STATE_ID_CALIBRATION,
    .HOST_ID    = MSM_STATE_ID_CALIBRATION,
    .ENTRY      = (StateMachine_Function_T)StartHoming,
    .LOOP       = (StateMachine_Function_T)ProcHoming,
    .NEXT       = (StateMachine_Transition_T)HomingTransition,
};

static const StateMachine_Cmd_T CMD_HOME = {  .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &STATE_ENCODER_HOMING, };

void Motor_Encoder_StartHoming(Motor_T * p_motor)
{
    StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_IDLE); /* for now */
    StateMachine_StartCmd(&p_motor->StateMachine, &CMD_HOME, 0);
}

/*  */
void Motor_Encoder_CalibrateHomeOffset(Motor_T * p_motor)
{
    Encoder_CalibrateIndexZeroRef(&p_motor->Encoder);
}

/******************************************************************************/
/*
    Open Loop
*/
/******************************************************************************/
// aligning assuming phase a has not been found.
// static void OpenLoop_EncoderAlignZero(Motor_T * p_motor, state_machine_input_value_t null)
// {
//     Motor_ProcCommutationMode(p_motor, Motor_FOC_StartStartUpAlign, NULL);
// }

// static void OpenLoop_ProcEncoderAlignZero(Motor_T * p_motor)
// {
//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         p_motor->ElectricalAngle = 0U;
//         Encoder_CaptureAlignZero(&p_motor->Encoder);
//         _StateMachine_EndSubState(&p_motor->StateMachine);
//     }
//     else
//     {
//         Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcStartUpAlign, NULL);

//         if (Encoder_ModeDT_GetScalarSpeed(&p_motor->Encoder) != 0U) /* reset the timer until speed is 0 */
//         {
//             Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
//         }
//     }
// }

static StateMachine_State_T *  AlignZeroTransition(Motor_T * p_motor)
{
    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        p_motor->ElectricalAngle = 0U;
        Encoder_CaptureAlignZero(&p_motor->Encoder);
        _StateMachine_EndSubState(&p_motor->StateMachine);
    }
    // else
    // {
    //     Motor_PollSensorAngle(p_motor);
    //     Motor_PollCaptureSpeed(p_motor);
    //     if (Encoder_ModeDT_GetScalarSpeed(&p_motor->Encoder) != 0U) /* reset the timer until speed is 0 */
    //     {
    //         Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    //     }
    // }

    return NULL;
}


static const StateMachine_State_T ALIGN =
{
    .ID     = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY  = (StateMachine_Function_T)Motor_FOC_StartStartUpAlign,
    .LOOP   = (StateMachine_Function_T)Motor_FOC_ProcStartUpAlign,
    .NEXT   = (StateMachine_Transition_T)AlignZeroTransition,
    // .P_HOST = &STATE_OPEN_LOOP,
    // .P_TRANSITION_TABLE = NULL,
};


static const StateMachine_Cmd_T CMD_ALIGN = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &ALIGN, };


/*

*/
static void  ValidateAlign(Motor_T * p_motor)
{
    // if transitioning from align
    // Motor_ZeroSensor(p_motor);
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles * 2U);
    Encoder_ModeDT_SetInitial(&p_motor->Encoder);
    FOC_SetReqD(&p_motor->Foc, 0);
    Motor_FOC_MatchFeedbackState(p_motor);
    Motor_FOC_StartOpenLoop(p_motor);
    // p_motor->FeedbackMode.OpenLoop = 1U; /* input limited until openloop clears */
    //p_motor->ElectricalAngle == 0U;
    p_motor->MechanicalAngle = Encoder_GetAngle(&p_motor->Encoder);
}

// static void OpenLoop_ProcEncoderValidateAlign(Motor_T * p_motor)
// {
//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         if (Encoder_GetAngle(&p_motor->Encoder) != p_motor->MechanicalAngle) { p_motor->FaultFlags.PositionSensor = 1U; }
//         // p_motor->FeedbackMode.OpenLoop = 0U;
//         // Encoder_CompleteAlignValidate(&p_motor->Encoder);
//         _StateMachine_EndSubState(&p_motor->StateMachine);
//     }
//     else
//     {
//         Motor_FOC_ProcOpenLoop(p_motor);
//         // Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleControl, NULL);  /* try closed loop */
//         // if ((p_motor->Speed_Fract16 ^ math_sign(p_motor->Foc.Vq)) < 0) { p_motor->FaultFlags.PositionSensor = 1U; }
//         // if (p_motor->FaultFlags.PositionSensor == 1U) { _StateMachine_EndSubState(&p_motor->StateMachine); }
//     }
// }

static StateMachine_State_T *  ValidateAlignTransition(Motor_T * p_motor)
{
    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        if (Encoder_GetAngle(&p_motor->Encoder) != p_motor->MechanicalAngle) { p_motor->FaultFlags.PositionSensor = 1U; }
        // p_motor->FeedbackMode.OpenLoop = 0U;
        // Encoder_CompleteAlignValidate(&p_motor->Encoder);
        _StateMachine_EndSubState(&p_motor->StateMachine);
    }

    return NULL;
}

static const StateMachine_State_T VALIDATE_ALIGN =
{
    .ID     = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY  = (StateMachine_Function_T)ValidateAlign,
    .LOOP   = (StateMachine_Function_T)Motor_FOC_ProcOpenLoop,
    .NEXT   = (StateMachine_Transition_T)ValidateAlignTransition,
    // .P_HOST = &STATE_OPEN_LOOP,
    // .P_TRANSITION_TABLE = NULL,
};

static const StateMachine_Cmd_T CMD_VALIDATE_ALIGN = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &VALIDATE_ALIGN, };

// static void OpenLoop_ProcEncoderValidateClosedLoop(Motor_T * p_motor)
// {
//     if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
//     {
//         p_motor->FeedbackMode.OpenLoop = 0U;
//         Encoder_CompleteAlignValidate(&p_motor->Encoder);
//         _StateMachine_EndSubState(&p_motor->StateMachine);
//     }
//     else
//     {
//         Motor_ProcCommutationMode(p_motor, Motor_FOC_ProcAngleControl, NULL);  /* try closed loop */
//         // or encoder get speed
//         if ((p_motor->Speed_Fract16 ^ math_sign(p_motor->Foc.Vq)) < 0) { p_motor->FaultFlags.PositionSensor = 1U; }
//         if (p_motor->FaultFlags.PositionSensor == 1U) { _StateMachine_EndSubState(&p_motor->StateMachine); }
//     }
// }

/*  */
static StateMachine_State_T * ValidateClosedLoopTransition(Motor_T * p_motor)
{
    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        p_motor->FeedbackMode.OpenLoop = 0U;
        Encoder_CompleteAlignValidate(&p_motor->Encoder);
        _StateMachine_EndSubState(&p_motor->StateMachine);
    }
    else
    {
        // or encoder get speed
        if ((p_motor->Speed_Fract16 ^ math_sign(p_motor->Foc.Vq)) < 0)
        {
            p_motor->FaultFlags.PositionSensor = 1U;
            _StateMachine_EndSubState(&p_motor->StateMachine);
        }
    }

    return NULL;
}

static const StateMachine_State_T VALIDATE_CLOSED_LOOP =
{
    .ID = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY = (StateMachine_Function_T)NULL,
    .LOOP = (StateMachine_Function_T)Motor_FOC_ProcAngleControl,
    .NEXT = (StateMachine_Transition_T)ValidateClosedLoopTransition,
    // .P_HOST = &STATE_OPEN_LOOP,
    // .P_TRANSITION_TABLE = NULL,
};

static const StateMachine_Cmd_T CMD_VALIDATE_CLOSED_LOOP = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &VALIDATE_CLOSED_LOOP, };


/*
    Start Up Chain
*/
static StateMachine_State_T * StartUpTransition(Motor_T * p_motor)
{


    return NULL;
}

static StateMachine_State_T * StartUpAlignTransition(Motor_T * p_motor)
{

}

static StateMachine_State_T * StartUpValidateAlignTransition(Motor_T * p_motor)
{

}

static StateMachine_State_T * StartUpValidateClosedLoopTransition(Motor_T * p_motor)
{

}


static const StateMachine_State_T START_UP =
{
    .ID = MSM_STATE_ID_OPEN_LOOP,
    // .ENTRY = (StateMachine_Function_T) ,
    // .LOOP = (StateMachine_Function_T) ,
    .NEXT = (StateMachine_Transition_T)StartUpTransition,
};

static const StateMachine_State_T START_UP_ALIGN =
{
    .ID = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY = (StateMachine_Function_T)Motor_FOC_StartStartUpAlign,
    .LOOP = (StateMachine_Function_T)Motor_FOC_ProcStartUpAlign,
    .NEXT = (StateMachine_Transition_T)StartUpAlignTransition,
    // .P_HOST = &STATE_OPEN_LOOP,
    // .P_TRANSITION_TABLE = NULL,
};

static const StateMachine_State_T START_UP_VALIDATE_ALIGN =
{
    .ID     = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY  = (StateMachine_Function_T)ValidateAlign,
    .LOOP   = (StateMachine_Function_T)Motor_FOC_ProcOpenLoop,
    .NEXT   = (StateMachine_Transition_T)StartUpValidateAlignTransition,
    // .P_HOST = &STATE_OPEN_LOOP,
    // .P_TRANSITION_TABLE = NULL,
};

static const StateMachine_State_T START_UP_VALIDATE_CLOSED_LOOP =
{
    .ID     = MSM_STATE_ID_OPEN_LOOP,
    .ENTRY  = (StateMachine_Function_T)NULL,
    .LOOP   = (StateMachine_Function_T)Motor_FOC_ProcAngleControl,
    .NEXT   = (StateMachine_Transition_T)StartUpValidateClosedLoopTransition,
    // .P_HOST = &STATE_OPEN_LOOP,
    // .P_TRANSITION_TABLE = NULL,
};

static const StateMachine_Cmd_T CMD_START_UP_CHAIN = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &START_UP, };

// static StateMachine_Cmd_T  OPEN_LOOP_ENCODER_CHAIN =
// [
//     { CMD_ALIGN, OpenLoop_EncoderChain,  },
//     { CMD_VALIDATE_ALIGN, OpenLoop_EncoderChain, },
//     { CMD_VALIDATE_CLOSED_LOOP, OpenLoop_EncoderChain, },
// ]

void Motor_Encoder_StartUpChain(Motor_T * p_motor) { StateMachine_StartCmd(&p_motor->StateMachine, &CMD_START_UP_CHAIN, 0); }


/******************************************************************************/
/* */
/******************************************************************************/
static inline void Motor_Calibration_StartEncoderDirection(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_Cycles);
    Phase_WriteDuty_Fract16(&p_motor->Phase, p_motor->Config.AlignPower_UFract16, 0U, 0U);
}

static inline bool Motor_Calibration_ProcEncoderDirection(Motor_T * p_motor)
{
    bool isComplete = false;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        switch (p_motor->CalibrationStateIndex)
        {
            case 0U:
                Encoder_CaptureQuadratureReference(&p_motor->Encoder);
                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, p_motor->Config.AlignPower_UFract16, 0U);
                p_motor->CalibrationStateIndex = 1U;
                break;

            case 1U:
                Encoder_CalibrateQuadraturePositive(&p_motor->Encoder);
                Phase_Float(&p_motor->Phase);
                isComplete = true;
                break;
            default: break;
        }
    }

    return isComplete;
}
