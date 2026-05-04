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
// #include "_Motor_Config.h"
#include "Motor.h"
#include <string.h>


/*

*/
void Motor_Init(const Motor_T * p_dev)
{
    // assert(Phase_VBus_GetVNominal() != 0U); /* set by caller init */

    /* Config including selected angle sensor init */
    if (p_dev->P_NVM_CONFIG != NULL) { p_dev->P_MOTOR->Config = *p_dev->P_NVM_CONFIG; }

    /*
        HW Modules Init
    */
    Phase_Init(&p_dev->PHASE);
#if defined(MOTOR_SIX_STEP_ENABLE)
    Phase_Polar_ActivateMode(&p_dev->PHASE, p_dev->P_MOTOR->Config.PhasePwmMode);
#endif

    /* Using Config Id */
    p_dev->P_MOTOR->p_ActiveSensor = RotorSensor_Of(&p_dev->SENSOR_TABLE, p_dev->P_MOTOR->Config.SensorMode);
    RotorSensor_Init(p_dev->P_MOTOR->p_ActiveSensor);

    p_dev->P_MOTOR->p_VBus = p_dev->P_VBUS; /* BackPointer for now */

    HeatMonitor_Init(&p_dev->HEAT_MONITOR);

    TimerT_Periodic_Init(&p_dev->CONTROL_TIMER, 1U);
    TimerT_Periodic_Init(&p_dev->SPEED_TIMER, 1U);


    Motor_Reset(p_dev->P_MOTOR); // alternatively move to state machine
    StateMachine_Init(&p_dev->STATE_MACHINE);
}

/*
    Reset derived reference and state variables
    alt handle in state machine after validate config
    void Motor_Config_Reset(Motor_State_T * p_motor)
*/
void Motor_Reset(Motor_State_T * p_motor)
{
    Motor_Config_Validate(&p_motor->Config);
    Motor_InitUnits(p_motor);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
    PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
#if defined(MOTOR_SIX_STEP_ENABLE)
    PID_Init(&p_motor->PidIBus);
    BEMF_Init(&p_motor->Bemf);
#endif

    Motor_ResetSpeedLimit(p_motor);
    Motor_ResetILimit(p_motor);

    Motor_InitSpeedRamp(p_motor);
    Motor_InitTorqueRamp(p_motor);
    Ramp_Init(&p_motor->VRamp, p_motor->Config.SpeedRampTime_Cycles, Phase_VBus_GetVNominal());

    /* Preset rate ramps do not need output limits */
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopRampSpeedTime_Cycles, p_motor->Config.OpenLoopRampSpeedFinal_Fract16); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampITime_Cycles, p_motor->Config.OpenLoopRampIFinal_Fract16);
    // Ramp_SetOutputLimit(&p_motor->OpenLoopSpeedRamp, -Motor_GetSpeedRated_Fract16(p_motor), Motor_GetSpeedRated_Fract16(p_motor));
    // Ramp_SetOutputLimit(&p_motor->OpenLoopIRamp, -Motor_OpenLoopILimit(p_motor), Motor_OpenLoopILimit(p_motor));

    Angle_SpeedRef_Init(&p_motor->OpenLoopSpeedRef, Motor_GetSpeedTypeMax_Angle(&p_motor->Config));

    /*
        Feedback State
    */
    FOC_Init(&p_motor->Foc);
    p_motor->ControlTimerBase = 0U;

    /* Keep for physical units and external reading */
    // Motor_ResetUnitsVabc(p_motor);
    // Motor_ResetUnitsIabc(p_motor);
}

void Motor_Reinit(Motor_T * p_motor)
{
    if (p_motor->P_NVM_CONFIG != NULL) { p_motor->P_MOTOR->Config = *p_motor->P_NVM_CONFIG; }
    Motor_Reset(p_motor->P_MOTOR);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    propagate Motor Config to sensor module params
*/
void Motor_InitUnits(Motor_State_T * p_motor)
{
    RotorSensor_Config_T config =
    {
        .PolePairs = p_motor->Config.PolePairs,
        .SpeedTypeMax_DegPerCycle = Motor_GetSpeedTypeMax_Angle(&p_motor->Config),  /* allow up to 2x the rated speed for unit conversion */
        .SpeedTypeMax_Rpm = Motor_GetSpeedTypeMax_Rpm(&p_motor->Config),
    };

    RotorSensor_InitUnitsFrom(p_motor->p_ActiveSensor, &config);
}


/******************************************************************************/
/*

*/
/******************************************************************************/

/* Ramp slope set independent of user Config.limits. By characteristics.   todo set with frac32 */
void Motor_InitSpeedRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, Motor_GetSpeedRated_Fract16(&p_motor->Config));
    // Motor_ResolveSpeedLimits(p_motor);
}

void Motor_InitTorqueRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, Phase_Calibration_GetIRatedPeak_Fract16()); /* Current by default */
    // Motor_ResolveILimits(p_motor);
}

// void Motor_EnableSpeedRamp(Motor_State_T * p_motor) { Motor_InitSpeedRamp(p_motor); }
// void Motor_DisableSpeedRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
// void Motor_EnableTorqueRamp(Motor_State_T * p_motor) { Motor_InitTorqueRamp(p_motor); }
// void Motor_DisableTorqueRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

/* use wider config window before direction are known */
void Motor_ResetSpeedLimit(Motor_State_T * p_motor)
{
    // p_motor->SpeedLimitCcw_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
    // p_motor->SpeedLimitCw_Fract16 = -p_motor->Config.SpeedLimitForward_Fract16;
    Ramp_SetOutputLimit(&p_motor->SpeedRamp, -p_motor->Config.SpeedLimitForward_Fract16, p_motor->Config.SpeedLimitForward_Fract16);
}

void Motor_ResetILimit(Motor_State_T * p_motor)
{
    // p_motor->ILimitCcw_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    // p_motor->ILimitCw_Fract16 = -p_motor->Config.ILimitMotoring_Fract16;
    Ramp_SetOutputLimit(&p_motor->TorqueRamp, -p_motor->Config.ILimitMotoring_Fract16, p_motor->Config.ILimitMotoring_Fract16);
}

void Motor_ResetSpeedPid(Motor_State_T * p_motor)
{
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
}

void Motor_ResetIPid(Motor_State_T * p_motor)
{
    PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
}

void _Motor_ResetTuning(Motor_T * p_motor)
{
    /* load from nvm to main consistency for save */
    p_motor->P_MOTOR->Config.PidSpeed = p_motor->P_NVM_CONFIG->PidSpeed;
    p_motor->P_MOTOR->Config.PidI = p_motor->P_NVM_CONFIG->PidI;
    Motor_ResetSpeedPid(p_motor->P_MOTOR);
    Motor_ResetIPid(p_motor->P_MOTOR);
}


/******************************************************************************/
/*
    FeedbackMode may update feedback limits
*/
/******************************************************************************/
void Motor_SetFeedbackMode(Motor_State_T * p_motor, Motor_FeedbackMode_T mode)
{
    p_motor->FeedbackMode.Value = mode.Value;

    interval_t v = VBus_AntiPluggingLimits(p_motor->p_VBus, (sign_t)p_motor->Direction);

    if (p_motor->FeedbackMode.Speed == 1U)
    {
        if (p_motor->FeedbackMode.Current == 1U) { PID_SetOutputLimits(&p_motor->PidSpeed, Motor_ILimitCw(p_motor), Motor_ILimitCcw(p_motor)); } /* SpeedPid Output is I */
        else                                     { PID_SetOutputLimits(&p_motor->PidSpeed, v.low, v.high); } /* SpeedPid Output is V */
    }

    if (p_motor->FeedbackMode.Current == 1U)    { Ramp_SetOutputLimit(&p_motor->TorqueRamp, Motor_ILimitCw(p_motor), Motor_ILimitCcw(p_motor)); }
    else                                        { Ramp_SetOutputLimit(&p_motor->TorqueRamp, v.low, v.high); }
    // else                                        { Ramp_SetOutputLimit(&p_motor->VRamp, v.low, v.high); }
}


/******************************************************************************/
/*
    Direction - applied voltage direction
*/
/******************************************************************************/
void Motor_SetDirection(Motor_T * p_dev, Motor_Direction_T direction)
{
    p_dev->P_MOTOR->Direction = direction;
    RotorSensor_ZeroInitial(p_dev->P_MOTOR->p_ActiveSensor);
    Motor_ResolveILimits(p_dev);
    Motor_ResolveSpeedLimits(p_dev);
}


/******************************************************************************/
/*!
    Active Limits
*/
/******************************************************************************/
/*
    effective when system ResolveLimits is disabled.
    Virtual fields resolved to ccw/cw limits on se
    write direction-resolved Ccw/Cw directly.
*/
/* Forward direction = Config.DirectionForward sign; lands in Ccw if forward==CCW, else in -Cw. */
// void _Motor_SetSpeedLimitForward(Motor_State_T * p_motor, uint16_t speed_ufract16)
// {
//     ufract16_t v = math_min(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
//     if (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) { p_motor->SpeedLimitCcw_Fract16 = v; } else { p_motor->SpeedLimitCw_Fract16 = -v; }
// }

// void _Motor_SetSpeedLimitReverse(Motor_State_T * p_motor, uint16_t speed_ufract16)
// {
//     ufract16_t v = math_min(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
//     if (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) { p_motor->SpeedLimitCw_Fract16 = -v; } else { p_motor->SpeedLimitCcw_Fract16 = v; }
// }

// void _Motor_SetSpeedLimit(Motor_State_T * p_motor, uint16_t speed_ufract16)
// {
//     _Motor_SetSpeedLimitForward(p_motor, speed_ufract16);
//     _Motor_SetSpeedLimitReverse(p_motor, speed_ufract16);
// }

// void _Motor_SetSpeedLimits(Motor_State_T * p_motor, uint16_t speed_ufract16)
// {
//     switch (p_motor->Config.DirectionForward)
//     {
//     }
// }

/* Motoring sign matches Direction: lands in Ccw if Direction==CCW, else in -Cw. */
// void _Motor_SetILimitMotoring(Motor_State_T * p_motor, uint16_t i_fract16)
// {
//     ufract16_t i_mot = math_min(i_fract16, p_motor->Config.ILimitMotoring_Fract16);
//     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { p_motor->ILimitCcw_Fract16 = i_mot; } else { p_motor->ILimitCw_Fract16 = -i_mot; }
// }

// /* Generating sign opposes Direction: lands in -Cw if Direction==CCW, else in Ccw. */
// void _Motor_SetILimitGenerating(Motor_State_T * p_motor, uint16_t i_fract16)
// {
//     ufract16_t i_gen = math_min(i_fract16, p_motor->Config.ILimitGenerating_Fract16);
//     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { p_motor->ILimitCw_Fract16 = -i_gen; } else { p_motor->ILimitCcw_Fract16 = i_gen; }
// }

// void _Motor_SetILimit(Motor_State_T * p_motor, uint16_t i_fract16)
// {
//     _Motor_SetILimitMotoring(p_motor, i_fract16);
//     _Motor_SetILimitGenerating(p_motor, i_fract16);
// }

// /*  Motor_GetILimits */
// void _Motor_SetILimits(Motor_State_T * p_motor, uint16_t motoring, uint16_t generating)
// {
//     switch (p_motor->Direction)
//     {
//         case MOTOR_DIRECTION_CCW:
//             p_motor->ILimitCcw_Fract16 = math_min(motoring, p_motor->Config.ILimitMotoring_Fract16);
//             p_motor->ILimitCw_Fract16 = -math_min(generating, p_motor->Config.ILimitGenerating_Fract16);
//             break;
//         case MOTOR_DIRECTION_CW:
//             p_motor->ILimitCcw_Fract16 = math_min(generating, p_motor->Config.ILimitGenerating_Fract16);
//             p_motor->ILimitCw_Fract16 = -math_min(motoring, p_motor->Config.ILimitMotoring_Fract16);
//             break;
//         default:
//             p_motor->ILimitCcw_Fract16 = math_min(motoring, p_motor->Config.ILimitGenerating_Fract16);
//             p_motor->ILimitCw_Fract16 = -math_min(motoring, p_motor->Config.ILimitGenerating_Fract16);
//             break;
//     }
// }





