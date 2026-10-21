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
#include "Motor_Config.h"
#include <string.h>


/*

*/
void Motor_Init(Motor_T * p_dev)
{
    assert(VBus_Pu(p_dev->P_VBUS) != 0U); /* set by caller init */

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

    HeatMonitor_Init(&p_dev->HEAT_MONITOR);

    TimerT_Periodic_Init(&p_dev->CONTROL_TIMER, 1U);
    TimerT_Periodic_Init(&p_dev->SPEED_TIMER, 1U);

    /* Limit inputs are upper layer owned. Init once, retained across Motor_Reset/Motor_Reinit */
    Motor_ResetILimit(p_dev->P_MOTOR);
    Motor_ResetSpeedLimit(p_dev->P_MOTOR);
    p_dev->P_MOTOR->ISupplyLimits = interval_symmetric(0, FRACT16_MAX); // call handle

    p_dev->P_MOTOR->FeedbackMode = MOTOR_FEEDBACK_MODE_CURRENT; /* Voltage modes are not accepted, [Motor_ApplyFeedbackMode] */

    Motor_Reset(p_dev->P_MOTOR); // alternatively move to state machine
    StateMachine_Init(&p_dev->STATE_MACHINE);
}


/*
    Reset derived reference and state variables
    alt handle in state machine after validate config
    void Motor_Config_Reset(Motor_Context_T * p_motor)
*/
void Motor_Reset(Motor_Context_T * p_motor)
{
    // Motor_Config_Validate(&p_motor->Config); /* State Machine Enter Fault on invalid config */
    Motor_InitUnits(p_motor);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);

    /* Limits update on direction set */
    Ramp_Init_Slope(&p_motor->SpeedRamp, p_motor->Config.SpeedRampSlope_PuPerTick);
    Ramp_Init_Slope(&p_motor->TorqueRamp, p_motor->Config.TorqueRampSlope_PuPerTick);
    Motor_FOC_InitParams(p_motor);
    // Ramp_Init(&p_motor->VRamp, p_motor->Config.SpeedRampTime_Cycles, Phase_VRated_Pu());

    /* Preset rate ramps do not need output limits */
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopRampSpeedTime_Cycles, p_motor->Config.OpenLoopRampSpeedFinal_Pu); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampITime_Cycles, p_motor->Config.OpenLoopRampIFinal_Pu);
    // Ramp_SetLimits(&p_motor->OpenLoopSpeedRamp, -_Motor_SpeedRated_Pu(p_motor), _Motor_SpeedRated_Pu(p_motor));
    // Ramp_SetLimits(&p_motor->OpenLoopIRamp, -_Motor_OpenLoopILimit(p_motor), _Motor_OpenLoopILimit(p_motor));

    PID_InitFrom(&p_motor->Foc.PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->Foc.PidId, &p_motor->Config.PidI);
    p_motor->ControlTimerBase = 0U;

    p_motor->Direction = MOTOR_DIRECTION_NULL;
    FOC_SetVAntiPlugging(&p_motor->Foc, SIGN_ZERO, 0U);
// #if defined(MOTOR_SENSOR_SENSORLESS_ENABLE)
//     EmfObserver_InitWinding(p_context->SENSOR_TABLE.SENSORLESS.P_OBSERVER, p_electrical->Rs, Motor_Lq_PuTurn(p_electrical));
// #endif
    /* Keep for physical units and external reading */
    // Motor_ResetUnitsVabc(p_motor);
    // Motor_ResetUnitsIabc(p_motor);

// #if defined(MOTOR_SIX_STEP_ENABLE)
//     PID_Init(&p_motor->PidIBus);
//     BEMF_Init(&p_motor->Bemf);
// #endif
}

void Motor_InitFrom(Motor_Context_T * p_motor, const Motor_Config_T * p_config)
{
    if (p_config != NULL) { p_motor->Config = *p_config; }
    Motor_Reset(p_motor);
}

/* common reinit. without hw registers. reload flash. */
/* Sensor update still require reboot */
void Motor_Reinit(Motor_T * p_motor)
{
    if (p_motor->P_NVM_CONFIG != NULL) { p_motor->P_MOTOR->Config = *p_motor->P_NVM_CONFIG; }
    Motor_Reset(p_motor->P_MOTOR);
}

/* Validate config across modules: base Motor config + field-weakening speed-limit consistency */
bool Motor_IsConfigValid(Motor_T * p_motor)
{
    Motor_Context_T * p_context = p_motor->P_MOTOR;
#if defined(MOTOR_FOC_FIELD_WEAKENING_ENABLE)
    uint16_t speedCeiling = Motor_FieldWeakening_IsEnabled(&p_context->Config.FieldWeakening) ? INT16_MAX : Motor_SpeedRated_Pu(p_motor);
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
    Motor_Config_ValidateFw(&p_context->Config);
    // optionally add runtime current limit
#endif
}

/******************************************************************************/
/*

*/
/******************************************************************************/
/*
    propagate Motor Config to sensor module params
    SensorState.UnitRef.SpeedPuRef is the motor's speed reference, shared with open loop
*/
void Motor_InitUnits(Motor_Context_T * p_motor)
{
    RotorSensor_InitUnitsFrom(p_motor->p_ActiveSensor, (RotorSensor_UnitRef_T)
    {
        .PollingFreq = MOTOR_CONTROL_FREQ,
        .PolePairs = p_motor->Config.Electrical.PolePairs,
        .AngleFreqBase = Motor_SpeedBase_AngleFreq(&p_motor->Config.Electrical),
    });
}

/*
    FOC params, derived. FOC and the observer hold no config: views of [Motor_Electrical_T] at the turn base, and the field weakening limit within the board FW rating.
    Re-resolve on a change of the model, the tuning, or the board base. An invalid model keeps the prior view.
*/
void Motor_FOC_InitParams(Motor_Context_T * p_context) /* optionally cal from state machine init to keep in Motor_FOC.h */
{
    const Motor_Electrical_T * p_electrical = &p_context->Config.Electrical;

    if (Motor_Electrical_IsValid(p_electrical))
    {
        FOC_InitElectrical(&p_context->Foc, &(FOC_Electrical_T)
        {
            .Ld     = Motor_Ld_PuTurn(p_electrical),
            .Lq     = Motor_Lq_PuTurn(p_electrical),
            .Rs     = p_electrical->Rs,
            .Psi    = Motor_Psi_PuTurn(p_electrical),
        });
    }
    p_context->Foc.FieldWeakening = (FOC_FieldWeakeningTuning_T) { .IdLimit = Motor_FieldWeakening_IdLimit(&p_context->Config.FieldWeakening), .IdGain = p_context->Config.FieldWeakening.IfwGain, };
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
    p_motor->UserCmd = 0;
}

// void Motor_EnableSpeedRamp(Motor_Context_T * p_motor) { Ramp_Init_Slope(&p_motor->SpeedRamp, p_motor->Config.SpeedRampSlope_PuPerTick); }
// void Motor_DisableSpeedRamp(Motor_Context_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
// void Motor_EnableTorqueRamp(Motor_Context_T * p_motor) { Motor_InitTorqueRamp(p_motor); }
// void Motor_DisableTorqueRamp(Motor_Context_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

/******************************************************************************/
/*
    FeedbackMode. Store only, Run_Entry resolves the feedback limits.
*/
/******************************************************************************/
void Motor_SetFeedbackMode(Motor_T * p_dev, Motor_FeedbackMode_T mode)
{
    p_dev->P_MOTOR->FeedbackMode.Value = mode.Value;
    // Motor_ResolveFeedbackLimits(p_dev);
}

/*
    Push alternative, with [_Motor_SetILimits]: catch-all on a feedback mode update, Run_Entry.
    TorqueRamp and SpeedPid output units follow FeedbackMode.Current
*/
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
    Direction - applied voltage direction, and its VLimit half-plane.
*/
/******************************************************************************/
void Motor_SetDirection(Motor_T * p_dev, Motor_Direction_T direction)
{
    FOC_SetVLimits(&p_dev->P_MOTOR->Foc, (sign_t)direction, VBus_GetVPhaseRefSvpwm(p_dev->P_VBUS));
    p_dev->P_MOTOR->Direction = direction;
    RotorSensor_ZeroInitial(p_dev->P_MOTOR->p_ActiveSensor);
    // Motor_ResolveSpeedLimits(p_dev->P_MOTOR);
    // Motor_ResolveILimits(p_dev->P_MOTOR);
}



/******************************************************************************/
/*!
    Active Limits — held inputs

    The upper layer stores its composed limit in PU, or a derate as a ratio of Config.
    Motor owns the Config comparison and the direction resolve.
        limit = min(Stored, Config)  => Motor_Resolve*Limits => Ramp/PID

    Unsigned magnitude in, direction-resolved signed [Cw:Ccw] out. FRACT16_MAX => no cap.
    I resolves within ISupplyLimits, signed already, captured at the operating point by [Motor_CaptureISupplyLimits].
    Store only. The state running the current loop resolves on entry and on its speed tick, [Motor_FOC_ResolveILimits].
*/
/******************************************************************************/
/*
    I Limits. Motoring aligns with Direction, Generating opposes.
*/
/* Push alternative: store and resolve from any state */
void _Motor_SetILimits(Motor_Context_T * p_motor, uint16_t motoring_pu, uint16_t generating_pu)
{
    p_motor->ILimit.Motoring = motoring_pu;
    p_motor->ILimit.Generating = generating_pu;
    Motor_ResolveILimits(p_motor);
}

void Motor_SetILimits(Motor_Context_T * p_motor, uint16_t motoring_pu, uint16_t generating_pu)
{
    Motor_CaptureILimits(p_motor, math_min(motoring_pu, p_motor->Config.ILimitMotoring_Pu), math_min(generating_pu, p_motor->Config.ILimitGenerating_Pu));
}

void Motor_SetILimitDerates(Motor_Context_T * p_motor, ufract16_t motoring, ufract16_t generating)
{
    Motor_CaptureILimits(p_motor, fract16_mul(motoring, p_motor->Config.ILimitMotoring_Pu), fract16_mul(generating, p_motor->Config.ILimitGenerating_Pu));
}

void Motor_SetILimitMotoring(Motor_Context_T * p_motor, uint16_t motoring_pu) { Motor_SetILimits(p_motor, motoring_pu, p_motor->ILimit.Generating); }
void Motor_SetILimitGenerating(Motor_Context_T * p_motor, uint16_t generating_pu) { Motor_SetILimits(p_motor, p_motor->ILimit.Motoring, generating_pu); }

/* Both sides to a common magnitude */
// void Motor_SetILimit(Motor_Context_T * p_motor, uint16_t i_pu) { Motor_SetILimits(p_motor, i_pu, i_pu); }

/* No cap. Config is the ceiling. */
void Motor_ResetILimit(Motor_Context_T * p_motor)
{
    Motor_CaptureILimits(p_motor, p_motor->Config.ILimitMotoring_Pu, p_motor->Config.ILimitGenerating_Pu);
    p_motor->ILimit.Forward = FRACT16_MAX;
    p_motor->ILimit.Reverse = FRACT16_MAX;
}


/******************************************************************************/
/*
*/
/******************************************************************************/
/*
    Speed Limits. Forward aligns with Config.DirectionForward, Reverse opposes.
    Keyed by user direction — no resync on Direction change.
*/
void Motor_SetSpeedLimits(Motor_Context_T * p_motor, uint16_t forward_pu, uint16_t reverse_pu)
{
    p_motor->SpeedLimit.Forward = forward_pu;
    p_motor->SpeedLimit.Reverse = reverse_pu;
    // Motor_ResolveSpeedLimits(p_motor);
}

void Motor_SetSpeedLimitForward(Motor_Context_T * p_motor, uint16_t forward_pu) { Motor_SetSpeedLimits(p_motor, forward_pu, p_motor->SpeedLimit.Reverse); }
void Motor_SetSpeedLimitReverse(Motor_Context_T * p_motor, uint16_t reverse_pu) { Motor_SetSpeedLimits(p_motor, p_motor->SpeedLimit.Forward, reverse_pu); }

void Motor_SetSpeedLimit(Motor_Context_T * p_motor, uint16_t speed_pu) { Motor_SetSpeedLimits(p_motor, speed_pu, speed_pu); }

void Motor_ResetSpeedLimit(Motor_Context_T * p_motor) { Motor_SetSpeedLimits(p_motor, p_motor->Config.SpeedLimitForward_Pu, p_motor->Config.SpeedLimitReverse_Pu); }

void Motor_SetSpeedLimitDerate(Motor_Context_T * p_motor, ufract16_t ratio) { Motor_SetSpeedLimits(p_motor, fract16_mul(ratio, p_motor->Config.SpeedLimitForward_Pu), fract16_mul(ratio, p_motor->Config.SpeedLimitReverse_Pu)); }


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

/******************************************************************************/
/*
*/
/******************************************************************************/
/* Maintain consistency between runtime and Nvm */
/* PID drops config to keep 1 copy */
void _Motor_Tuning_SetSpeedKp(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidSpeed.Kp = value;
    PID_SetKp_Fixed32(&p_state->PidSpeed, value);
}

void _Motor_Tuning_SetSpeedKi(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidSpeed.Ki = value;
    PID_SetKi_Fixed32(&p_state->PidSpeed, value);
}

void _Motor_Tuning_SetIKp(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidI.Kp = value;
    PID_SetKp_Fixed32(&p_state->Foc.PidIq, value);
    PID_SetKp_Fixed32(&p_state->Foc.PidId, value);
}

void _Motor_Tuning_SetIKi(Motor_Context_T * p_state, uint32_t value)
{
    p_state->Config.PidI.Ki = value;
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
