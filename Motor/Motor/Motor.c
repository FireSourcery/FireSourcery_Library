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

    LimitArray_ClearAll(&p_dev->I_LIMITS_LOCAL);
    LimitArray_ClearAll(&p_dev->I_GEN_LIMITS_LOCAL);
    LimitArray_ClearAll(&p_dev->SPEED_LIMITS_LOCAL);

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
static void _Motor_ResolveILimits(Motor_State_T * p_motor)
{
    interval_t iLimits = Motor_GetILimits(p_motor);
    p_motor->ILimitCcw_Fract16 = iLimits.high;
    p_motor->ILimitCw_Fract16  = iLimits.low;
}

static void _Motor_ResolveSpeedLimits(Motor_State_T * p_motor)
{
    interval_t speedLimits = Motor_GetSpeedLimits(p_motor);
    p_motor->SpeedLimitCcw_Fract16 = speedLimits.high;
    p_motor->SpeedLimitCw_Fract16  = speedLimits.low;
}

static void Motor_ResolveILimits(Motor_State_T * p_motor)
{
    _Motor_ResolveILimits(p_motor);

    Ramp_SetOutputLimit(&p_motor->TorqueRamp, Motor_ILimitCw(p_motor), Motor_ILimitCcw(p_motor));
    if ((p_motor->FeedbackMode.Speed == 1) && (p_motor->FeedbackMode.Current == 1))  /* SpeedPid Output is I */
    {
        PID_SetOutputLimits(&p_motor->PidSpeed, Motor_ILimitCw(p_motor), Motor_ILimitCcw(p_motor));
    }
}

static void Motor_ResolveSpeedLimits(Motor_State_T * p_motor)
{
    _Motor_ResolveSpeedLimits(p_motor);

    Ramp_SetOutputLimit(&p_motor->SpeedRamp, Motor_SpeedLimitCw(p_motor), Motor_SpeedLimitCcw(p_motor));
}


/* Ramp slope set independent of user Config.limits. By characteristics.   todo set with frac32 */
void Motor_InitSpeedRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, Motor_GetSpeedRated_Fract16(&p_motor->Config));
    Motor_ResolveSpeedLimits(p_motor);
}

void Motor_InitTorqueRamp(Motor_State_T * p_motor)
{
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, Phase_Calibration_GetIRatedPeak_Fract16()); /* Current by default */
    Motor_ResolveILimits(p_motor);
}

// void Motor_EnableSpeedRamp(Motor_State_T * p_motor) { Motor_InitSpeedRamp(p_motor); }
// void Motor_DisableSpeedRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
// void Motor_EnableTorqueRamp(Motor_State_T * p_motor) { Motor_InitTorqueRamp(p_motor); }
// void Motor_DisableTorqueRamp(Motor_State_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

void Motor_ResetSpeedPid(Motor_State_T * p_motor)
{
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
}

void Motor_ResetIPid(Motor_State_T * p_motor)
{
    PID_InitFrom(&p_motor->PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->PidId, &p_motor->Config.PidI);
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
void Motor_SetDirection(Motor_State_T * p_motor, Motor_Direction_T direction)
{
    p_motor->Direction = direction;
    RotorSensor_ZeroInitial(p_motor->p_ActiveSensor);
    Motor_ResolveILimits(p_motor);
    Motor_ResolveSpeedLimits(p_motor);
}


/******************************************************************************/
/*!
    Active Limits - Non directional
    Table comparison control
    Derive directional Feedback Limits
*/
/******************************************************************************/
/*
    Speed Limit
*/
void Motor_ResetSpeedLimit(Motor_State_T * p_motor)
{
    p_motor->SpeedLimitForward_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
    p_motor->SpeedLimitReverse_Fract16 = p_motor->Config.SpeedLimitReverse_Fract16;
    Motor_ResolveSpeedLimits(p_motor);
}

void Motor_SetSpeedLimits(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
    p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
    Motor_ResolveSpeedLimits(p_motor);
}

void Motor_SetSpeedLimitForward(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitForward_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
    if (Motor_IsDirectionForward(p_motor) == true) { Motor_ResolveSpeedLimits(p_motor); }
}

void Motor_SetSpeedLimitReverse(Motor_State_T * p_motor, uint16_t speed_ufract16)
{
    p_motor->SpeedLimitReverse_Fract16 = math_limit_upper(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
    if (Motor_IsDirectionReverse(p_motor) == true) { Motor_ResolveSpeedLimits(p_motor); }
}

/*
    ILimit
*/
void Motor_ResetILimit(Motor_State_T * p_motor)
{
    p_motor->ILimitMotoring_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
    p_motor->ILimitGenerating_Fract16 = p_motor->Config.ILimitGenerating_Fract16;
    Motor_ResolveILimits(p_motor);
}

void Motor_SetILimitMotoring(Motor_State_T * p_motor, uint16_t i_fract16)
{
    p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_fract16, p_motor->Config.ILimitMotoring_Fract16);
    Motor_ResolveILimits(p_motor);
}

void Motor_SetILimitGenerating(Motor_State_T * p_motor, uint16_t i_fract16)
{
    p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_fract16, p_motor->Config.ILimitGenerating_Fract16);
    Motor_ResolveILimits(p_motor);
}

void Motor_SetILimits(Motor_State_T * p_motor, uint16_t i_fract16)
{
    p_motor->ILimitMotoring_Fract16 = math_limit_upper(i_fract16, p_motor->Config.ILimitMotoring_Fract16);
    p_motor->ILimitGenerating_Fract16 = math_limit_upper(i_fract16, p_motor->Config.ILimitGenerating_Fract16);
    Motor_ResolveILimits(p_motor);
}




