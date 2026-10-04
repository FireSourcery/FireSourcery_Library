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
#include "Math/FOC_Ext.h"
#include "Motor_Config.h"
#include <string.h>


/*

*/
void Motor_Init(Motor_T * p_dev)
{
    assert(VBus_Pu(p_dev->P_VBUS) != 0U); /* set by caller init */

    /* Config including selected angle sensor init */
    if (p_dev->P_NVM_CONFIG != NULL) { p_dev->P_MOTOR->Config = *p_dev->P_NVM_CONFIG; }
    if (p_dev->P_FOC_NVM_CONFIG != NULL) { FOC_Init(&p_dev->P_MOTOR->Foc, p_dev->P_FOC_NVM_CONFIG); }

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

    HeatMonitor_Init(&p_dev->HEAT_MONITOR);

    TimerT_Periodic_Init(&p_dev->CONTROL_TIMER, 1U);
    TimerT_Periodic_Init(&p_dev->SPEED_TIMER, 1U);

    /* Limit inputs are upper layer owned. Init once, retained across Motor_Reset/Motor_Reinit */
    p_dev->P_MOTOR->ILimit.Motoring = FRACT16_MAX;
    p_dev->P_MOTOR->ILimit.Generating = FRACT16_MAX;
    p_dev->P_MOTOR->ILimit.Derate = FRACT16_MAX;
    p_dev->P_MOTOR->SpeedLimit.Forward = FRACT16_MAX;
    p_dev->P_MOTOR->SpeedLimit.Reverse = FRACT16_MAX;
    p_dev->P_MOTOR->SpeedLimit.Derate = FRACT16_MAX;

    Motor_Reset(p_dev->P_MOTOR); // alternatively move to state machine
    StateMachine_Init(&p_dev->STATE_MACHINE);
}

// void Motor_InitFrom(Motor_T * p_dev, const Motor_Config_T * p_config)
// {
// }

// void Motor_InitFocFrom(Motor_T * p_dev, const FOC_Config_T * p_foc_config)
// {
// }

/*
    Reset derived reference and state variables
    alt handle in state machine after validate config
    void Motor_Config_Reset(Motor_Context_T * p_motor)
*/
void Motor_Reset(Motor_Context_T * p_motor)
{
    // Motor_Config_Validate(&p_motor->Config); /* State Machine Enter Fault on invalid config */
    Motor_InitUnits(p_motor);
    Motor_ResolveFocParams(p_motor);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);

    /* Limits update on direction set */
    Ramp_Init_Slope(&p_motor->SpeedRamp, p_motor->Config.SpeedRampSlope_PuPerTick);
    Ramp_Init_Slope(&p_motor->TorqueRamp, p_motor->Config.TorqueRampSlope_PuPerTick);
    // Motor_ResolveSpeedLimits(p_motor);
    // Motor_ResolveILimits(p_motor);
    // Ramp_Init(&p_motor->VRamp, p_motor->Config.SpeedRampTime_Cycles, Phase_VRated_Pu());

    /* Preset rate ramps do not need output limits */
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopRampSpeedTime_Cycles, p_motor->Config.OpenLoopRampSpeedFinal_Pu); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampITime_Cycles, p_motor->Config.OpenLoopRampIFinal_Pu);
    // Ramp_SetLimits(&p_motor->OpenLoopSpeedRamp, -_Motor_SpeedRated_Pu(p_motor), _Motor_SpeedRated_Pu(p_motor));
    // Ramp_SetLimits(&p_motor->OpenLoopIRamp, -_Motor_OpenLoopILimit(p_motor), _Motor_OpenLoopILimit(p_motor));
    Angle_SpeedPuRef_Init_Freq(&p_motor->OpenLoopSpeedRef, MOTOR_CONTROL_FREQ, Motor_SpeedBase_AngleFreq(&p_motor->Config.Electrical));

    PID_InitFrom(&p_motor->Foc.PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->Foc.PidId, &p_motor->Config.PidI);
    p_motor->ControlTimerBase = 0U;

    p_motor->Direction = MOTOR_DIRECTION_NULL;

    /* Keep for physical units and external reading */
    // Motor_ResetUnitsVabc(p_motor);
    // Motor_ResetUnitsIabc(p_motor);

// #if defined(MOTOR_SIX_STEP_ENABLE)
//     PID_Init(&p_motor->PidIBus);
//     BEMF_Init(&p_motor->Bemf);
// #endif
}

/* common reinit. without hw registers. reload flash. */
/* Sensor update still require reboot */
void Motor_Reinit(Motor_T * p_motor)
{
    if (p_motor->P_NVM_CONFIG != NULL) { p_motor->P_MOTOR->Config = *p_motor->P_NVM_CONFIG; }
    if (p_motor->P_FOC_NVM_CONFIG != NULL) { FOC_Init(&p_motor->P_MOTOR->Foc, p_motor->P_FOC_NVM_CONFIG); }
    Motor_Reset(p_motor->P_MOTOR);
}

/* Validate config across modules: base Motor config + FOC field-weakening speed-limit consistency */
bool Motor_IsConfigValid(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
#if defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE)
    uint16_t speedCeiling = FOC_Config_IsFwEnabled(&p_context->Foc.Config) ? INT16_MAX : Motor_SpeedRated_Pu(p_motor);
#else
    uint16_t speedCeiling = Motor_SpeedRated_Pu(p_motor);
#endif
    return Motor_Config_IsValid(&p_context->Config) && _Motor_Config_IsValidSpeed(&p_context->Config, speedCeiling);
    // && _Motor_Config_IsValidVoltage(&p_context->Config, VBus_Pu(p_motor->P_VBUS));
}

/*

*/
void Motor_ValidateConfig(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
    Motor_Config_ValidateVAlign(&p_context->Config);
#if defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE)
    p_motor->P_MOTOR->Foc.Config.FieldWeakening.IdLimit = math_min(p_motor->P_MOTOR->Foc.Config.FieldWeakening.IdLimit, Phase_IRatedFw_Pu());
    p_motor->P_MOTOR->Config.ILimitMotoring_Pu = math_min(p_motor->P_MOTOR->Config.ILimitMotoring_Pu, fract16_vector_component(p_motor->P_MOTOR->Foc.Config.FieldWeakening.IdLimit, Phase_IRatedPeak_Pu()));
    // optionally add runtime current limit
#endif
#if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
    /* The one observer gain derived from motor params rather than stored tuning. */
    FOC_Sensorless_InitG(p_motor->SENSOR_TABLE.SENSORLESS.P_OBSERVER, p_context->Config.Electrical.Ls);
#endif
}

/*
    FOC params at the speed base, from [Motor_Electrical_T] at the tick base.
    Re-resolve on a change of the model or the board base.
*/
void Motor_ResolveFocParams(Motor_Context_T * p_motor)
{
    p_motor->Foc.Electrical = Motor_Electrical_FocOf(&p_motor->Config.Electrical);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    propagate Motor Config to sensor module params
*/
void Motor_InitUnits(Motor_Context_T * p_motor)
{
    RotorSensor_UnitRef_T config =
    {
        .PolePairs = p_motor->Config.Electrical.PolePairs,
        .AngleFreqBase = Motor_SpeedBase_AngleFreq(&p_motor->Config.Electrical),
        .PollingFreq = MOTOR_CONTROL_FREQ,
    };

    RotorSensor_InitUnitsFrom(p_motor->p_ActiveSensor, &config);
}

/******************************************************************************/
/*

*/
/******************************************************************************/
void Motor_ClearFeedbackState(Motor_Context_T * p_motor)
{
    PID_Reset(&p_motor->PidSpeed);
    Phase_Input_ClearI(&p_motor->PhaseInput);
    Phase_Input_ClearV(&p_motor->PhaseInput);
    Ramp_SetOutputState(&p_motor->SpeedRamp, 0);
    Ramp_SetOutputState(&p_motor->TorqueRamp, 0);
    Ramp_SetTarget(&p_motor->TorqueRamp, 0);
    Ramp_SetTarget(&p_motor->SpeedRamp, 0);
}

// void Motor_EnableSpeedRamp(Motor_Context_T * p_motor) { Ramp_Init_Slope(&p_motor->SpeedRamp, p_motor->Config.SpeedRampSlope_PuPerTick); }
// void Motor_DisableSpeedRamp(Motor_Context_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
// void Motor_EnableTorqueRamp(Motor_Context_T * p_motor) { Motor_InitTorqueRamp(p_motor); }
// void Motor_DisableTorqueRamp(Motor_Context_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

/******************************************************************************/
/*
    FeedbackMode may update feedback limits
*/
/******************************************************************************/
void Motor_SetFeedbackMode(Motor_T * p_dev, Motor_FeedbackMode_T mode)
{
    p_dev->P_MOTOR->FeedbackMode.Value = mode.Value;
    Motor_ResolveFeedbackLimits(p_dev);
}

/* TorqueRamp and SpeedPid output units follow FeedbackMode.Current */
void Motor_ResolveFeedbackLimits(Motor_T * p_dev)
{
    Motor_Context_T * p_motor = p_dev->P_MOTOR;
    interval_t v = Motor_GetVLimitsAntiPlugging(p_dev);

    /* Current mode: resolve from inputs, TorqueRamp may hold V limits from voltage mode. */
    if (p_motor->FeedbackMode.Current == 1U) { Motor_ResolveILimits(p_motor); }
    else
    {
        if (p_motor->FeedbackMode.Speed == 1U) { PID_SetOutputLimits(&p_motor->PidSpeed, v.low, v.high); } /* SpeedPid Output is V */
        Ramp_SetLimits(&p_motor->TorqueRamp, v.low, v.high); /* alternatively use Vramp */
    }
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
    Motor_ResolveSpeedLimits(p_dev->P_MOTOR);
    Motor_ResolveILimits(p_dev->P_MOTOR);
}



/******************************************************************************/
/*!
    Active Limits — push interface

    The upper layer pushes two independent inputs; Motor owns the comparison and the direction resolve.
        Value:  physical cap in PU, per motor, single user channel.       Motor_Set*Limit*()
        Derate: system arbitration result, ratio of Config, shared.       Motor_Set*LimitDerate()
        limit = min(Value, Derate * Config)  => Motor_Resolve*Limits => Ramp/PID

    Unsigned magnitude in, direction-resolved signed [Cw:Ccw] out. Config is the ceiling via Derate * Config.
*/
/******************************************************************************/
/*
    I Limits. Motoring aligns with Direction, Generating opposes.
*/
void Motor_SetILimits(Motor_Context_T * p_motor, uint16_t motoring_pu, uint16_t generating_pu)
{
    p_motor->ILimit.Motoring = motoring_pu;
    p_motor->ILimit.Generating = generating_pu;
    Motor_ResolveILimits(p_motor);
}

void Motor_SetILimitMotoring(Motor_Context_T * p_motor, uint16_t motoring_pu) { Motor_SetILimits(p_motor, motoring_pu, p_motor->ILimit.Generating); }
void Motor_SetILimitGenerating(Motor_Context_T * p_motor, uint16_t generating_pu) { Motor_SetILimits(p_motor, p_motor->ILimit.Motoring, generating_pu); }

/* Both sides to a common magnitude */
void Motor_SetILimit(Motor_Context_T * p_motor, uint16_t i_pu) { Motor_SetILimits(p_motor, i_pu, i_pu); }

/* Clear the value channel. Derate remains. */
void Motor_ResetILimit(Motor_Context_T * p_motor) { Motor_SetILimits(p_motor, FRACT16_MAX, FRACT16_MAX); }

/* System derate. Ratio of Config, compared against the value channel. */
void Motor_SetILimitDerate(Motor_Context_T * p_motor, uint16_t scalar)
{
    p_motor->ILimit.Derate = scalar;
    Motor_ResolveILimits(p_motor);
}

/*
    Speed Limits. Forward aligns with Config.DirectionForward, Reverse opposes.
    Keyed by user direction — no resync on Direction change.
*/
void Motor_SetSpeedLimits(Motor_Context_T * p_motor, uint16_t forward_pu, uint16_t reverse_pu)
{
    p_motor->SpeedLimit.Forward = forward_pu;
    p_motor->SpeedLimit.Reverse = reverse_pu;
    Motor_ResolveSpeedLimits(p_motor);
}

void Motor_SetSpeedLimitForward(Motor_Context_T * p_motor, uint16_t forward_pu) { Motor_SetSpeedLimits(p_motor, forward_pu, p_motor->SpeedLimit.Reverse); }
void Motor_SetSpeedLimitReverse(Motor_Context_T * p_motor, uint16_t reverse_pu) { Motor_SetSpeedLimits(p_motor, p_motor->SpeedLimit.Forward, reverse_pu); }

void Motor_SetSpeedLimit(Motor_Context_T * p_motor, uint16_t speed_pu) { Motor_SetSpeedLimits(p_motor, speed_pu, speed_pu); }

void Motor_ResetSpeedLimit(Motor_Context_T * p_motor) { Motor_SetSpeedLimits(p_motor, FRACT16_MAX, FRACT16_MAX); }

void Motor_SetSpeedLimitDerate(Motor_Context_T * p_motor, uint16_t scalar)
{
    p_motor->SpeedLimit.Derate = scalar;
    Motor_ResolveSpeedLimits(p_motor);
}


/******************************************************************************/
/*
*/
/******************************************************************************/
void Motor_ResetSpeedPid(Motor_Context_T * p_motor)
{
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
}

void Motor_ResetIPid(Motor_Context_T * p_motor)
{
    PID_InitFrom(&p_motor->Foc.PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->Foc.PidId, &p_motor->Config.PidI);
}

void _Motor_ResetTuning(Motor_T * p_motor)
{
    /* load from nvm to maintain consistency for save */
    p_motor->P_MOTOR->Config.PidSpeed = p_motor->P_NVM_CONFIG->PidSpeed;
    p_motor->P_MOTOR->Config.PidI = p_motor->P_NVM_CONFIG->PidI;
    Motor_ResetSpeedPid(p_motor->P_MOTOR);
    Motor_ResetIPid(p_motor->P_MOTOR);
}

/* Maintain consistency between runtime and Nvm */
void _Motor_Tuning_SetSpeedKp(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidSpeed.Kp_Fixed32 = value;
    PID_SetKp_Fixed32(&p_state->PidSpeed, value);
}

void _Motor_Tuning_SetSpeedKi(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidSpeed.Ki_Fixed32 = value;
    PID_SetKi_Fixed32(&p_state->PidSpeed, value);
}

void _Motor_Tuning_SetIKp(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidI.Kp_Fixed32 = value;
    PID_SetKp_Fixed32(&p_state->Foc.PidIq, value);
    PID_SetKp_Fixed32(&p_state->Foc.PidId, value);
}

void _Motor_Tuning_SetIKi(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidI.Ki_Fixed32 = value;
    PID_SetKi_Fixed32(&p_state->Foc.PidIq, value);
    PID_SetKi_Fixed32(&p_state->Foc.PidId, value);
}

/******************************************************************************/
/*
*/
/******************************************************************************/
void _Motor_Tuning_SetSpeedKp_Fixed16(Motor_Context_T * p_state, uint32_t value)
{
    _PID_SetKp_Fixed16(&p_state->Config.PidSpeed, value);
    PID_SetKp_Fixed16(&p_state->PidSpeed, value);
}

void _Motor_Tuning_SetSpeedKi_Fixed16(Motor_Context_T * p_state, uint32_t value)
{
    _PID_SetKi_Fixed16(&p_state->Config.PidSpeed, value);
    PID_SetKi_Fixed16(&p_state->PidSpeed, value);
}

void _Motor_Tuning_SetIKp_Fixed16(Motor_Context_T * p_state, uint32_t value)
{
    _PID_SetKp_Fixed16(&p_state->Config.PidI, value);
    PID_SetKp_Fixed16(&p_state->Foc.PidIq, value);
    PID_SetKp_Fixed16(&p_state->Foc.PidId, value);
}

void _Motor_Tuning_SetIKi_Fixed16(Motor_Context_T * p_state, uint32_t value)
{
    _PID_SetKi_Fixed16(&p_state->Config.PidI, value);
    PID_SetKi_Fixed16(&p_state->Foc.PidIq, value);
    PID_SetKi_Fixed16(&p_state->Foc.PidId, value);
}
