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


/******************************************************************************/
/*
    Interface Table
*/
/******************************************************************************/
// void Motor_InitSensor(Motor_T * p_motor)
// {
//     Encoder_ModeDT_Init_InterruptQuadrature(&p_motor->Encoder);
//     Encoder_EnableQuadratureMode(&p_motor->Encoder);
//     Motor_ResetUnitsSensor(p_motor);
// }

// bool Motor_VerifySensorCalibration(Motor_T * p_motor)
// {

// }

// angle16_t Motor_PollSensorAngle(Motor_T * p_motor)
// {
//     Encoder_GetAngle_Scalar(&p_motor->Encoder, p_motor->Config.PolePairs); /* ElectricalAngle => MechanicalAngle * PolePairs */
//     // electricalAngle += Encoder_ModeDT_InterpolateAngularDisplacement(&p_motor->Encoder);
// }

// angle16_t Motor_GetMechanicalAngle(const Motor_T * p_motor)
// {
//     Encoder_GetAngle(&p_motor->Encoder);
// }

// /*!
//     @return Fract16 Q1.15 unsaturated
// */
// int32_t Motor_PollSensorSpeed(Motor_T * p_motor)
// {
//     Encoder_ModeDT_PollScalarVelocity(&p_motor->Encoder) / 2;
// }

// void Motor_ZeroSensor(Motor_T * p_motor)
// {
//     Encoder_ModeDT_SetInitial(&p_motor->Encoder);
// }

// /* From Stop and after Align */
// bool _Motor_IsSensorAvailable(const Motor_T * p_motor)
// {
//     Encoder_IsAligned(&p_motor->Encoder);
// }

// // inline bool Motor_IsClosedLoopStart(const Motor_T * p_motor)
// // {
// //     return ((_Motor_IsSensorAvailable(p_motor) == true) && (_Motor_IsOpenLoop(p_motor) == false));
// // }

// Motor_Direction_T Motor_GetDirection(Motor_T * p_motor)
// {

// }

// /*
//     Sensor Direction
// */
// void Motor_SetSensorCcw(Motor_T * p_motor)
// {

// }

// void Motor_SetSensorCw(Motor_T * p_motor)
// {

// }

// void Motor_ResetUnitsSensor(Motor_T * p_motor)
// {
//     Motor_ResetUnitsEncoder(p_motor);
// }

// /* Common, Set after PolePairs */
// void Motor_ResetUnitsEncoder(Motor_T * p_motor)
// {
//     if (Motor_GetSpeedVRef_Rpm(p_motor) != p_motor->Encoder.Config.ScalarSpeedRef_Rpm)
//     {
//         Encoder_SetScalarSpeedRef(&p_motor->Encoder, Motor_GetSpeedVRef_Rpm(p_motor));
//     }
//     if (p_motor->Config.PolePairs != p_motor->Encoder.Config.InterpolateAngleScalar) /* Set for electrical cycle */
//     {
//         Encoder_DeltaT_SetInterpolateAngleScalar(&p_motor->Encoder, p_motor->Config.PolePairs);
//     }
//     // if(p_motor->Config.GearRatioOutput != p_motor->Encoder.Config.GearRatioOutput) ||
//     // {
//     //     Encoder_SetSurfaceRatio(&p_motor->Encoder, p_motor->Config.GearRatio);
//     // }
// }

/******************************************************************************/
/*
    States
*/
/******************************************************************************/

/*

*/
static void StartHoming(Motor_T * p_motor)
{
    Timer_StartPeriod_Millis(&p_motor->ControlTimer, 20); //~1rpm
    Motor_FOC_ActivateOutputZero(p_motor);
    Encoder_StartHoming(&p_motor->Encoder);
    p_motor->ElectricalAngle = 0U;
}

static void ProcHoming(Motor_T * p_motor)
{
    uint16_t angle;

    /* alternatively openloop speed/angle ramp */
    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        angle = Encoder_GetHomingAngle(&p_motor->Encoder) * p_motor->Config.PolePairs;
        Motor_FOC_ActivateAngle(p_motor, p_motor->ElectricalAngle + angle, p_motor->Config.AlignPower_UFract16, 0);
    }
}

static StateMachine_State_T * HomingTransition(Motor_T * p_motor)
{
    if (Encoder_PollHomingComplete(&p_motor->Encoder) == true) /* todo error status */
    {
        // Encoder_CalibrateQuadratureDirection(&p_motor->Encoder, p_motor->Direction == MOTOR_DIRECTION_CCW);
        // _StateMachine_EndSubState(&p_motor->StateMachine);
        // StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE, PHASE_OUTPUT_FLOAT);
        return &MOTOR_STATE_STOP;
    }

    return NULL;
}

static const StateMachine_State_T STATE_ENCODER_HOMING =
{
    // .ID         = MSM_STATE_ID_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)StartHoming,
    .LOOP       = (StateMachine_Function_T)ProcHoming,
    .NEXT       = (StateMachine_Transition_T)HomingTransition,
};


void Motor_Encoder_StartHoming(Motor_T * p_motor)
{
    // static const StateMachine_Cmd_T CMD_HOME = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &STATE_ENCODER_HOMING, };
    // StateMachine_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, MOTOR_CALIBRATION_STATE_IDLE); /* for now */// enter calib state
    // StateMachine_StartCmd(&p_motor->StateMachine, &CMD_HOME, 0); //begin from calib state only
    _StateMachine_ProcSubStateInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION, (uintptr_t)&STATE_ENCODER_HOMING);
}

/*  */
void Motor_Encoder_CalibrateHomeOffset(Motor_T * p_motor)
{
    Encoder_CalibrateIndexZeroRef(&p_motor->Encoder);
}

void Motor_Encoder_StartVirtualHome(Motor_T * p_motor)
{
    Motor_Calibration_StartHome(p_motor);
}

/******************************************************************************/
/*
    Open Loop
*/
/******************************************************************************/
// aligning assuming phase a has not been found.
// static void OpenLoop_EncoderAlignZero(Motor_T * p_motor, state_machine_value_t null)
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
    .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)Motor_FOC_StartStartUpAlign,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcStartUpAlign,
    .NEXT       = (StateMachine_Transition_T)AlignZeroTransition,
};


/*

*/
static void ValidateAlign(Motor_T * p_motor)
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
    .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)ValidateAlign,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcOpenLoop,
    .NEXT       = (StateMachine_Transition_T)ValidateAlignTransition,


};


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
    .ID         = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT   = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH      = 1U,
    .ENTRY      = (StateMachine_Function_T)NULL,
    .LOOP       = (StateMachine_Function_T)Motor_FOC_ProcAngleControl,
    .NEXT       = (StateMachine_Transition_T)ValidateClosedLoopTransition,
    // .P_TRANSITION_TABLE = NULL,
};




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
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    // .ENTRY = (StateMachine_Function_T) ,
    // .LOOP = (StateMachine_Function_T) ,
    .NEXT = (StateMachine_Transition_T)StartUpTransition,
};

static const StateMachine_State_T START_UP_ALIGN =
{
    .ID = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (StateMachine_Function_T)Motor_FOC_StartStartUpAlign,
    .LOOP = (StateMachine_Function_T)Motor_FOC_ProcStartUpAlign,
    .NEXT = (StateMachine_Transition_T)StartUpAlignTransition,
};

static const StateMachine_State_T START_UP_VALIDATE_ALIGN =
{
    .ID = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (StateMachine_Function_T)ValidateAlign,
    .LOOP = (StateMachine_Function_T)Motor_FOC_ProcOpenLoop,
    .NEXT = (StateMachine_Transition_T)StartUpValidateAlignTransition,
};

static const StateMachine_State_T START_UP_VALIDATE_CLOSED_LOOP =
{
    .ID = MSM_STATE_ID_OPEN_LOOP,
    .P_PARENT = &MOTOR_STATE_OPEN_LOOP,
    .DEPTH = 1U,
    .ENTRY = (StateMachine_Function_T)NULL,
    .LOOP = (StateMachine_Function_T)Motor_FOC_ProcAngleControl,
    .NEXT = (StateMachine_Transition_T)StartUpValidateClosedLoopTransition,
};


/*

*/

void Motor_Encoder_StartAlignZero(Motor_T * p_motor)
{
    static const StateMachine_Cmd_T CMD_ALIGN = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &ALIGN, };
    StateMachine_StartCmd(&p_motor->StateMachine, &CMD_ALIGN, 0);
}

void Motor_Encoder_StartValidateAlign(Motor_T * p_motor)
{
    static const StateMachine_Cmd_T CMD_VALIDATE_ALIGN = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &VALIDATE_ALIGN, };
    StateMachine_StartCmd(&p_motor->StateMachine, &CMD_VALIDATE_ALIGN, 0);
}

void Motor_Encoder_StartValidateClosedLoop(Motor_T * p_motor)
{
    static const StateMachine_Cmd_T CMD_VALIDATE_CLOSED_LOOP = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &VALIDATE_CLOSED_LOOP, };
    StateMachine_StartCmd(&p_motor->StateMachine, &CMD_VALIDATE_CLOSED_LOOP, 0);
}

void Motor_Encoder_StartUpChain(Motor_T * p_motor)
{
    static const StateMachine_Cmd_T CMD_START_UP_CHAIN = { .CMD = (StateMachine_CmdInput_T)NULL, .P_INITIAL = &START_UP, };
    StateMachine_StartCmd(&p_motor->StateMachine, &CMD_START_UP_CHAIN, 0);
}


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
