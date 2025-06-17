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
    @file   Motor.c
    @author FireSourcery
    @brief  Motor module conventional function definitions.

*/
/******************************************************************************/
#include "Motor.h"
#include <string.h>


// void Motor_InitFrom(const Motor_T * p_context, const Motor_Config_T * p_config)
// {
//     if (p_config != NULL) { memcpy(&p_context->P_ACTIVE->Config, p_config, sizeof(Motor_Config_T)); }

//     /*
//         HW Wrappers Init
//     */
//     Phase_Init(&p_context->PHASE);
// #if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
//     Phase_Polar_ActivateMode(&p_motor->PHASE, p_motor->Config.PhasePwmMode);
// #endif
//     Motor_Sensor_Init(p_context);
//     Motor_Reset(p_context->P_ACTIVE); // alternatively move to state machine
//     StateMachine_Init(&p_context->STATE_MACHINE);
// }

/*

*/
void Motor_Init(const Motor_T * p_context)
{
    assert(MotorAnalog_GetVSource_Fract16() != 0U); /* set before init */
    // Motor_InitFrom(p_context, p_context->P_NVM_CONFIG);

    /* Config including selected angle sensor init */
    if (p_context->P_NVM_CONFIG != NULL) { p_context->P_ACTIVE->Config = *p_context->P_NVM_CONFIG; }

    /*
        HW Wrappers Init
    */
    Phase_Init(&p_context->PHASE);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_ActivateMode(&p_motor->PHASE, p_motor->Config.PhasePwmMode);
#endif

    p_context->P_ACTIVE->p_ActiveSensor = MotorSensor_Of(&p_context->SENSOR_TABLE, p_context->P_ACTIVE->Config.SensorMode);
    MotorSensor_Init(p_context->P_ACTIVE->p_ActiveSensor);

    // HeatMonitor_Init(&p_motor->Thermistor);

    Motor_Reset(p_context->P_ACTIVE); // alternatively move to state machine
    StateMachine_Init(&p_context->STATE_MACHINE);
}

/*

*/
void Motor_Reset(Motor_State_T * p_motor)
{
    /*
        SW Structs
    */
   Timer_InitPeriodic(&p_motor->ControlTimer, 1U);
   Timer_InitPeriodic(&p_motor->SpeedTimer, 1U);

    /*
       Feedback State
    */
    FOC_Init(&p_motor->Foc);
    // BEMF_Init(&p_motor->Bemf);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
    PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
    PID_Init(&p_motor->PidIBus);
#endif

    Motor_ResetSpeedLimitActive(p_motor);
    Motor_ResetILimitActive(p_motor);

    Motor_ResetSpeedRamp(p_motor);
    Motor_ResetTorqueRamp(p_motor);

// #if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopRampSpeed_Cycles, p_motor->Config.OpenLoopRampSpeedFinal_Fract16); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampI_Cycles, p_motor->Config.OpenLoopRampIFinal_Fract16);
// #endif

    /* Keep for physical units and external reading */
    // Motor_ResetUnitsVabc(p_motor);
    // Motor_ResetUnitsIabc(p_motor);

    p_motor->ControlTimerBase = 0U;
}


/******************************************************************************/
/*

*/
/******************************************************************************/
void Motor_ResetBaseOpenLoopILimit(Motor_State_T * p_motor)
{
    // p_motor->Config.OpenLoopScalarLimit_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);
    // p_motor->Config.AlignScalar_Fract16 =  p_motor->Config.AlignScalar_Fract16 ;
    // p_motor->Config.OpenLoopRampIFinal_Fract16 = Motor_OpenLoopILimitOf(p_motor, p_motor->Config.OpenLoopRampIFinal_Fract16);
}

void Motor_ResetSpeedRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, INT16_MAX);
    // Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, p_motor->Config.SpeedRated_DegPerCycle);
    // Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, p_motor->Config.SpeedLimitForward_DegPerCycle);
}

void Motor_ResetTorqueRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, MotorAnalogRef_GetIRatedPeak_Fract16());
    // Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, p_motor->Config.ILimitMotoring_Fract16);
}

void Motor_EnableSpeedRamp(Motor_State_T * p_motor) { Motor_ResetSpeedRamp(p_motor); }
void Motor_EnableTorqueRamp(Motor_State_T * p_motor) { Motor_ResetTorqueRamp(p_motor); }
void Motor_DisableSpeedRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
void Motor_DisableTorqueRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

void Motor_ResetSpeedPid(Motor_State_T * p_motor)
{
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
    // PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
    // PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor));
}

void Motor_ResetCurrentPid(Motor_State_T * p_motor)
{
    PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
}

/******************************************************************************/
/*
    Propagate limits Common
    optionally store Cw/Ccw limits
*/
/******************************************************************************/
/* reset input can be removed for on proc */
static void ApplySpeedLimit(Motor_State_T * p_motor)
{
    // if (p_motor->FeedbackMode.Speed == 1U) /* speed limit is applied on input */
    // {
    //     Ramp_SetTarget(&p_motor->SpeedRamp, Motor_SpeedReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->SpeedRamp))); /* clamp the req until the next input */
    // }
    /* else speed limit is applied on feedback */
}

/* only speed loop must update */
static void ApplyILimit(Motor_State_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U)  /* limit is applied on feedback */
    {
        if (p_motor->FeedbackMode.Current == 1U) /* SpeedPid Output is I */
        {
            PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor)); // or on get
        }
        // else /* SpeedPid Output is V */
        // {
        //     PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor)); /* as phase voltage if foc */
        // }
    }
//     else /* limit is applied on input */ /* alternatively move to on proc */
//     {
//         if (p_motor->FeedbackMode.Current == 1U)
//         {
//             Ramp_SetTarget(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)));
//         }
//     }

    // alternatively,
    // if (p_motor->FeedbackMode.Current == 1U)
    // {
    //     if (p_motor->FeedbackMode.Speed == 1U)  /* limit is applied on feedback */
    //     {
    //         PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
    //     }
    //     else /* limit is applied on input */
    //     {
    //         Ramp_SetOutput(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, Ramp_GetOutput(&p_motor->TorqueRamp)));
    //     }
    // }
}

/******************************************************************************/
/*
    StateMachine Controlled
*/
/******************************************************************************/
/******************************************************************************/
/*
    FeedbackMode may update feedback limits
*/
/******************************************************************************/
void Motor_SetFeedbackMode(Motor_State_T * p_motor, Motor_FeedbackMode_T mode)
{
    p_motor->FeedbackMode.Value = mode.Value;
    ApplyILimit(p_motor);
    ApplySpeedLimit(p_motor);
}

void Motor_SetFeedbackMode_Cast(Motor_State_T * p_motor, int modeValue) { Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(modeValue)); }


/******************************************************************************/
/*
    Direction - applied voltage direcction
*/
/******************************************************************************/
/*
*/
void Motor_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction)
{
    p_motor->Direction = direction;
    // Motor_SetSensorDirection(p_motor, direction);
    ApplyILimit(p_motor);
    ApplySpeedLimit(p_motor);
    // vlimit on non
}

/*
    Forward/Reverse using calibration param
    alternatively move to user
*/
void Motor_SetDirectionForward(Motor_State_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionForward(p_motor)); }
void Motor_SetDirectionReverse(Motor_State_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }
// void Motor_SetUserDirection(Motor_State_T * p_motor, int sign) { Motor_SetDirection(p_motor, (Motor_Direction_T)(sign * p_motor->Config.DirectionForward)); }


/******************************************************************************/
/*!
    Active Limits - Non directional
    Inner Unconditional Set, always overwrite.
    Derive directional Feedback Limits
    optionally store cw/ccw limits first
*/
/******************************************************************************/
void Motor_ResetSpeedLimitActive(Motor_State_T * p_motor)
{
    p_motor->SpeedLimitForward_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
    p_motor->SpeedLimitReverse_Fract16 = p_motor->Config.SpeedLimitReverse_Fract16;
}

void Motor_ResetILimitActive(Motor_State_T * p_motor)
{
    p_motor->ILimitMotoring_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    p_motor->ILimitGenerating_Fract16 = p_motor->Config.ILimitGenerating_Fract16;
}

/*
    Speed Limit
*/
void Motor_SetSpeedLimitForward(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
    if (Motor_IsDirectionForward(p_motor) == true) { ApplySpeedLimit(p_motor); }
}

void Motor_SetSpeedLimitReverse(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
    if (Motor_IsDirectionReverse(p_motor) == true) { ApplySpeedLimit(p_motor); }
}

/* As scalar of base config */
void Motor_SetSpeedLimit_Scalar(Motor_State_T * p_motor, uint16_t scalar_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = fract16_mul(p_motor->Config.SpeedLimitForward_Fract16, scalar_ufract16);
    p_motor->SpeedLimitReverse_Fract16 = fract16_mul(p_motor->Config.SpeedLimitReverse_Fract16, scalar_ufract16);
    ApplySpeedLimit(p_motor);
}

void Motor_ClearSpeedLimit(Motor_State_T * p_motor)
{
    p_motor->SpeedLimitForward_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
    p_motor->SpeedLimitReverse_Fract16 = p_motor->Config.SpeedLimitReverse_Fract16;
    ApplySpeedLimit(p_motor);
}


/*
    ILimit
*/
void Motor_SetILimitMotoring(Motor_State_T * p_motor, uint16_t i_Fract16)
{
    p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitMotoring_Fract16);
    ApplyILimit(p_motor);
}

void Motor_SetILimitGenerating(Motor_State_T * p_motor, uint16_t i_Fract16)
{
    p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_Fract16, p_motor->Config.ILimitGenerating_Fract16);
    ApplyILimit(p_motor);
}

void Motor_SetILimit_Scalar(Motor_State_T * p_motor, uint16_t scalar_ufract16)
{
    p_motor->ILimitMotoring_Fract16 = fract16_mul(p_motor->Config.ILimitMotoring_Fract16, scalar_ufract16);
    p_motor->ILimitGenerating_Fract16 = fract16_mul(p_motor->Config.ILimitGenerating_Fract16, scalar_ufract16);
    ApplyILimit(p_motor);
}

void Motor_ClearILimit(Motor_State_T * p_motor)
{
    p_motor->ILimitMotoring_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    p_motor->ILimitGenerating_Fract16 = p_motor->Config.ILimitGenerating_Fract16;
    ApplyILimit(p_motor);
}


/******************************************************************************/
/*
*/
/******************************************************************************/
bool Motor_TrySpeedLimit(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    bool isLimit = false;
    switch (Motor_GetUserDirection(p_motor))
    {
        case 1:
            if (speed_ufract16 > p_motor->SpeedLimitForward_Fract16) { Motor_SetSpeedLimitForward(p_motor, speed_ufract16); isLimit = true; }
            break;
        case -1:
            if (speed_ufract16 > p_motor->SpeedLimitReverse_Fract16) { Motor_SetSpeedLimitReverse(p_motor, speed_ufract16); isLimit = true; }
            break;
        default: break;
    }

    return isLimit;
}

bool Motor_TryILimit(Motor_State_T * p_motor, uint16_t i_Fract16)
{
    bool isLimit = false;
    if (i_Fract16 < p_motor->ILimitMotoring_Fract16) { p_motor->ILimitMotoring_Fract16 = i_Fract16; isLimit = true; }
    if (i_Fract16 < p_motor->ILimitGenerating_Fract16) { p_motor->ILimitGenerating_Fract16 = i_Fract16; isLimit = true; }
    if (isLimit == true) { ApplyILimit(p_motor); }
    return isLimit;
}

/*
    Interface
    Set using comparison struct
*/
void Motor_SetSpeedLimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TrySpeedLimit(p_motor, LimitArray_GetUpper(p_limit)); }
    // else                                        { Motor_ClearSpeedLimit(p_motor); }
}

void Motor_SetILimitWith(Motor_State_T * p_motor, LimitArray_T * p_limit)
{
    if (LimitArray_IsUpperActive(p_limit) == true) { Motor_TryILimit(p_motor, LimitArray_GetUpper(p_limit)); }
    // else                                        { Motor_ClearILimit(p_motor); }
}

