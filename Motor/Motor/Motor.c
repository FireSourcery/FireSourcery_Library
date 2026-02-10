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


/*

*/
void Motor_Init(const Motor_T * p_context)
{
    assert(Phase_VBus_Fract16() != 0U); /* set before init */

    /* Config including selected angle sensor init */
    if (p_context->P_NVM_CONFIG != NULL) { p_context->P_MOTOR_STATE->Config = *p_context->P_NVM_CONFIG; }

    /*
        HW Modules Init
    */
    Phase_Init(&p_context->PHASE);
    // Phase_Analog_Init(&p_context->PHASE);
#if defined(MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_ActivateMode(&p_context->PHASE, p_context->P_MOTOR_STATE->Config.PhasePwmMode);
#endif

    /* Using Config Id */
    p_context->P_MOTOR_STATE->p_ActiveSensor = RotorSensor_Of(&p_context->SENSOR_TABLE, p_context->P_MOTOR_STATE->Config.SensorMode);
    RotorSensor_Init(p_context->P_MOTOR_STATE->p_ActiveSensor);

    // HeatMonitor_Init(&p_context->HEAT_MONITOR_CONTEXT);

    TimerT_Periodic_Init(&p_context->CONTROL_TIMER, 1U);
    TimerT_Periodic_Init(&p_context->SPEED_TIMER, 1U);

    Motor_Reset(p_context->P_MOTOR_STATE); // alternatively move to state machine
    StateMachine_Init(&p_context->STATE_MACHINE);
}

/*
*/
void Motor_Reset(Motor_State_T * p_motor)
{
    Motor_ResetUnits(p_motor);

    /*
        SW Structs
    */

    /*
       Feedback State
    */
    FOC_Init(&p_motor->Foc);
    // BEMF_Init(&p_motor->Bemf);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
    PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
#if defined(MOTOR_SIX_STEP_ENABLE)
    PID_Init(&p_motor->PidIBus);
#endif

    Motor_ResetSpeedLimitActive(p_motor);
    Motor_ResetILimitActive(p_motor);

    Motor_ResetSpeedRamp(p_motor);
    Motor_ResetTorqueRamp(p_motor);

// #if defined(MOTOR_OPEN_LOOP_ENABLE)
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopRampSpeedTime_Cycles, p_motor->Config.OpenLoopRampSpeedFinal_Fract16); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampITime_Cycles, p_motor->Config.OpenLoopRampIFinal_Fract16);
    // #endif
    Angle_InitFrom(&p_motor->OpenLoopAngle, &(Angle_Config_T){.SpeedRef_Angle16 = Motor_GetSpeedTypeMax_DegPerCycle(p_motor)});

    /* Keep for physical units and external reading */
    // Motor_ResetUnitsVabc(p_motor);
    // Motor_ResetUnitsIabc(p_motor);
    p_motor->ControlTimerBase = 0U;
}


/******************************************************************************/
/*

*/
/******************************************************************************/

/*
    propagate Motor Config to sensor module params
*/
void Motor_ResetUnits(Motor_State_T * p_motor)
{
    RotorSensor_Config_T config =
    {
        .PolePairs = p_motor->Config.PolePairs,
        .ElSpeedRated_DegPerCycle = Motor_GetSpeedRated_DegPerCycle(p_motor),
        .MechSpeedRated_Rpm = Motor_GetSpeedRated_Rpm(p_motor),
        .SpeedTypeMax_DegPerCycle = Motor_GetSpeedTypeMax_DegPerCycle(p_motor),  /* allow up to 2x the rated speed for unit conversion */
        .SpeedTypeMax_Rpm = Motor_GetSpeedTypeMax_Rpm(p_motor),
    };

    RotorSensor_InitUnitsFrom(p_motor->p_ActiveSensor, &config);
    Angle_InitFrom(&p_motor->p_ActiveSensor->P_STATE->AngleSpeed, &(Angle_Config_T){.SpeedRef_Angle16 = Motor_GetSpeedTypeMax_DegPerCycle(p_motor)}); /*  */
}

void Motor_ResetSpeedRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, INT16_MAX); /* As normalized Speed */
    // Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, p_motor->Config.SpeedLimitForward);
}

void Motor_ResetTorqueRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, Phase_Calibration_GetIRatedPeak_Fract16()); /* Current by default */
    // Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, p_motor->Config.ILimitMotoring_Fract16);
}

void Motor_EnableSpeedRamp(Motor_State_T * p_motor) { Motor_ResetSpeedRamp(p_motor); }
void Motor_EnableTorqueRamp(Motor_State_T * p_motor) { Motor_ResetTorqueRamp(p_motor); }
void Motor_DisableSpeedRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
void Motor_DisableTorqueRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

// void Motor_ResetSpeedPid(Motor_State_T * p_motor)
// {
//     PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
// }

// void Motor_ResetCurrentPid(Motor_State_T * p_motor)
// {
//     PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
//     PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
// }

/******************************************************************************/
/*
    Propagate limits Common
    optionally store Cw/Ccw limits
*/
/******************************************************************************/
/*
    Ramp limits apply on proc
    Pid hold limits for integral clamp
*/
/* reset input can be removed for on proc */
static void ApplySpeedLimit(Motor_State_T * p_motor)
{
    // if (p_motor->FeedbackMode.Speed == 1U) /* speed limit is applied on input, and proc */
    // {
    //     Ramp_SetTarget(&p_motor->SpeedRamp, Motor_SpeedReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->SpeedRamp))); /* clamp the req until the next input */
    // }
    /* else speed limit is applied on feedback */

    // SpeedLimit_Fract16 = _Motor_GetSpeedLimitActive(p_motor);
}

/* only speed loop must update */
// alternatively update on proc
static void ApplyILimit(Motor_State_T * p_motor)
{
    if (p_motor->FeedbackMode.Speed == 1U)  /* limit is applied on feedback */
    {
        if (p_motor->FeedbackMode.Current == 1U) /* SpeedPid Output is I */
        {
            PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetILimitCw(p_motor), _Motor_GetILimitCcw(p_motor));
        }
        else /* SpeedPid Output is V */
        {
            PID_SetOutputLimits(&p_motor->PidSpeed, _Motor_GetVLimitCw(p_motor), _Motor_GetVLimitCcw(p_motor));
        }
    }
//     else /* limit is applied on input */ /* alternatively move to on proc */
//     {
//         if (p_motor->FeedbackMode.Current == 1U)
//         {
//             Ramp_SetTarget(&p_motor->TorqueRamp, Motor_IReqLimitOf(p_motor, Ramp_GetTarget(&p_motor->TorqueRamp)));
//         }
//     }
//  alternatively set cached limits
//  p_motor->ILimitCcw_Fract16 =  _Motor_GetILimitCcw(p_motor);
//  p_motor->ILimitCw_Fract16 =  _Motor_GetILimitCw(p_motor);
}

/******************************************************************************/
/*
    Inner unconditional set, always overwrite.
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

void Motor_SetFeedbackMode_Cast(Motor_State_T * p_motor, int mode) { Motor_SetFeedbackMode(p_motor, Motor_FeedbackMode_Cast(mode)); }


/******************************************************************************/
/*
    Direction - applied voltage direction
*/
/******************************************************************************/
void Motor_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction)
{
    p_motor->Direction = direction;
    RotorSensor_SetDirection(p_motor->p_ActiveSensor, direction);
    RotorSensor_ZeroInitial(p_motor->p_ActiveSensor);
    ApplyILimit(p_motor);
    ApplySpeedLimit(p_motor);
}

/*
    Forward/Reverse using calibration param
    alternatively move to user
*/
void Motor_SetDirectionForward(Motor_State_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionForward(p_motor)); }
void Motor_SetDirectionReverse(Motor_State_T * p_motor) { Motor_SetDirection(p_motor, Motor_GetDirectionReverse(p_motor)); }
// void Motor_SetDirectionSign(Motor_State_T * p_motor, int sign) { Motor_SetDirection(p_motor, (Motor_Direction_T)(sign * p_motor->Config.DirectionForward)); }


/******************************************************************************/
/*!
    Active Limits - Non directional
    Table comparison control
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

// void Motor_SetSpeedLimit(Motor_State_T * p_motor, uint16_t speed_ufract16)
// {
    // set both, use one side
    // math_limit_upper(speed_ufract16, (p_motor->Direction == p_motor->Config.DirectionForward) ? p_motor->Config.SpeedLimitForward_Fract16 : p_motor->Config.SpeedLimitReverse_Fract16);
    // p_motor->SpeedLimitForward_Fract16 = speed_ufract16;
    // p_motor->SpeedLimitReverse_Fract16 = speed_ufract16;
//     p_motor->SpeedLimitActive  = speed_ufract16;
//     ApplySpeedLimit(p_motor);
// }

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


