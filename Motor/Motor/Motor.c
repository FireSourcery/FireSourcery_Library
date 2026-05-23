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
    assert(VBus_GetVNominal(p_dev->P_VBUS) != 0U); /* set by caller init */

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

    // Motor_Config_ResolveSpeedRated
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
    // p_motor->ElectricalSpeedRef = Motor_ElectricalSpeedRef_FromSpeedRating(&p_motor->Config.SpeedRating);
    Motor_InitUnits(p_motor);

    /* Output Limits Set later depending on commutation mode, feedback mode, direction */
    PID_InitFrom(&p_motor->PidSpeed, &p_motor->Config.PidSpeed);
    PID_InitFrom(&p_motor->Foc.PidIq, &p_motor->Config.PidI);
    PID_InitFrom(&p_motor->Foc.PidId, &p_motor->Config.PidI);
#if defined(MOTOR_SIX_STEP_ENABLE)
    PID_Init(&p_motor->PidIBus);
    BEMF_Init(&p_motor->Bemf);
#endif

    Motor_InitSpeedRamp(p_motor);
    Motor_InitTorqueRamp(p_motor);
    // Motor_ResetSpeedLimit(p_motor);
    // Motor_ResetILimit(p_motor);
    Ramp_Init(&p_motor->VRamp, p_motor->Config.SpeedRampTime_Cycles, Phase_Calibration_GetVRated_Fract16());

    /* Preset rate ramps do not need output limits */
    /* Start at 0 speed in FOC mode for continuous angle displacements */
    Ramp_Init(&p_motor->OpenLoopSpeedRamp, p_motor->Config.OpenLoopRampSpeedTime_Cycles, p_motor->Config.OpenLoopRampSpeedFinal_Fract16); /* direction updated on set */
    Ramp_Init(&p_motor->OpenLoopIRamp, p_motor->Config.OpenLoopRampITime_Cycles, p_motor->Config.OpenLoopRampIFinal_Fract16);
    // Ramp_SetLimits(&p_motor->OpenLoopSpeedRamp, -Motor_GetSpeedRated_Fract16(p_motor), Motor_GetSpeedRated_Fract16(p_motor));
    // Ramp_SetLimits(&p_motor->OpenLoopIRamp, -Motor_OpenLoopILimit(p_motor), Motor_OpenLoopILimit(p_motor));

    Angle_SpeedRef_Init(&p_motor->OpenLoopSpeedRef, Motor_GetSpeedTypeMax_Angle(&p_motor->Config.SpeedRating));


    FOC_Init(&p_motor->Foc);
    Motor_InitDecouplingCoeffs(&p_motor->Config); // todo
    FOC_InitElectrical(&p_motor->Foc, &p_motor->Config.ElectricalParams_Pu);
    FOC_Sensorless_Init(&p_motor->FocSensorless, &p_motor->Config.SensorlessConfig);
    FOC_Sensorless_InitG(&p_motor->FocSensorless, smo_g_pu_rpm(MOTOR_I_LOOP_FREQ, Motor_GetSpeedTypeMax_Rpm(&p_motor->Config.SpeedRating), p_motor->Config.SpeedRating.PolePairs, (p_motor->Config.ElectricalParams_Pu.Ld + p_motor->Config.ElectricalParams_Pu.Lq) / 2));
    Angle_SpeedFractCalib_T speed_calib = { .PollingFreq = MOTOR_CONTROL_FREQ, .SpeedMax_Rpm = Motor_GetSpeedTypeMax_Rpm(&p_motor->Config.SpeedRating) };
    FOC_Sensorless_InitAngleUnits(&p_motor->FocSensorless, &speed_calib);

    p_motor->Foc.IdFwGain = p_motor->Config.FieldWeakening.IdFwGain;
    p_motor->Foc.IdFwLimit = p_motor->Config.FieldWeakening.IdFwLimit;
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
void Motor_InitUnits(Motor_Context_T * p_motor)
{
    RotorSensor_Config_T config =
    {
        .PolePairs = p_motor->Config.SpeedRating.PolePairs,
        .SpeedTypeMax_DegPerCycle = Motor_GetSpeedTypeMax_Angle(&p_motor->Config.SpeedRating),
        .SpeedTypeMax_Rpm = Motor_GetSpeedTypeMax_Rpm(&p_motor->Config.SpeedRating),
    };

    RotorSensor_InitUnitsFrom(p_motor->p_ActiveSensor, &config);
}

/*
    Encode (Rs, Ld, Lq, ψ_f) in the PU basis selected by MOTOR_PU_BASIS.
    Rs is basis-independent (no ω, no Fs). L and ψ pick up the basis anchor.
    Caller MUST feed ω_pu in the matching basis to FOC's decoupling multiplies.
*/
void Motor_InitDecouplingCoeffs(Motor_Config_T * p_config)
{
    uint16_t v_max = Phase_Calibration_GetVMaxVolts();
    uint16_t i_max = Phase_Calibration_GetIMaxAmps();

    p_config->ElectricalParams_Pu.Rs = rs_pu_of_mohm(v_max, i_max, p_config->ElectricalParams_Si.Rs);

#ifdef MOTOR_PU_BASIS_ANGLE16
    /*
        Fs-anchored (speed per unit base π · Fs => Fs 32768 / π), motor independent ω_pu

        accept int64 intermediary or
        direct-multiply (no Q15 scaling), el_delta_angle16 * L_pu → v_pu without /32768 => ~0.5% resolution loss
    */
    p_config->ElectricalParams_Pu.Ld = l_pu_of_uh(MOTOR_CONTROL_FREQ, v_max, i_max, p_config->ElectricalParams_Si.Ld);
    p_config->ElectricalParams_Pu.Lq = l_pu_of_uh(MOTOR_CONTROL_FREQ, v_max, i_max, p_config->ElectricalParams_Si.Lq);
    // p_config->ElectricalParams_Pu.Psi = psi_pu_of_wb(MOTOR_CONTROL_FREQ, v_max, p_config->ElectricalParams_Si.Psi, 1000);
    p_config->ElectricalParams_Pu.Psi = psi_pu_of_kv(MOTOR_CONTROL_FREQ, v_max, p_config->SpeedRating.PolePairs, p_config->SpeedRating.Kv);
#else
    /*
        motor speed  anchored:
        Fs independent
        ω_mech_pu = ω_elec_pu
        θ_increment = ω_pu · (ω_base / Fs)
    */
    uint32_t n_max = Motor_GetSpeedTypeMax_Rpm(&p_config->SpeedRating);
    uint8_t  pp = p_config->SpeedRating.PolePairs;
    p_config->ElectricalParams_Pu.Ld = l_pu_rpm_of_h(v_max, i_max, n_max, pp, p_config->ElectricalParams_Si.Ld, 1000000UL);
    p_config->ElectricalParams_Pu.Lq = l_pu_rpm_of_h(v_max, i_max, n_max, pp, p_config->ElectricalParams_Si.Lq, 1000000UL);
    p_config->ElectricalParams_Pu.Psi = psi_pu_rpm_of_kv(v_max, n_max, p_config->SpeedRating.Kv);
    // p_config->ElectricalParams_Pu_Test.Psi = psi_pu_rpm_of_wb(v_max, n_max, p_config->SpeedRating.PolePairs, p_config->ElectricalParams_Si.Psi, 1000U);
#endif
}

// void Motor_InitDecouplingCoeffs(Motor_Config_T * p_config)
// {
//     p_config->ElectricalParams_Pu.Rs = rs_pu_of_mohm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->ElectricalParams_Si.Rs);
//     p_config->ElectricalParams_Pu.Ld = l_pu_rpm_of_h(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), Motor_GetSpeedTypeMax_Rpm(&p_config->SpeedRating), p_config->SpeedRating.PolePairs, p_config->ElectricalParams_Si.Ld, 1000000UL);
//     p_config->ElectricalParams_Pu.Lq = l_pu_rpm_of_h(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), Motor_GetSpeedTypeMax_Rpm(&p_config->SpeedRating), p_config->SpeedRating.PolePairs, p_config->ElectricalParams_Si.Lq, 1000000UL);
//     p_config->ElectricalParams_Pu.Psi = psi_pu_rpm_of_kv(Phase_Calibration_GetVMaxVolts(), Motor_GetSpeedTypeMax_Rpm(&p_config->SpeedRating), p_config->SpeedRating.Kv);
// }

// void Motor_InitDecouplingCoeffs_Angle16(Motor_Config_T * p_config)
// {
//     /* MOTOR_CONTROL_FREQ computes angle speed, unrelated to I_LOOP_FREQ */
//     p_config->ElectricalParams_Pu.Ld = l_pu_of_h(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->ElectricalParams_Si.Ld, 1000000UL);
//     p_config->ElectricalParams_Pu.Lq = l_pu_of_h(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->ElectricalParams_Si.Lq, 1000000UL);
//     p_config->ElectricalParams_Pu.Rs = rs_pu_of_mohm(Phase_Calibration_GetVMaxVolts(), Phase_Calibration_GetIMaxAmps(), p_config->ElectricalParams_Si.Rs);
//     p_config->ElectricalParams_Pu.Psi = psi_pu_of_kv(MOTOR_CONTROL_FREQ, Phase_Calibration_GetVMaxVolts(), p_config->SpeedRating.Kv, p_config->SpeedRating.PolePairs);
// }

/******************************************************************************/
/*

*/
/******************************************************************************/

/* Ramp slope set independent of user Config.limits. By characteristics.   todo set with frac32 */
void Motor_InitSpeedRamp(Motor_Context_T * p_motor)
{
    Ramp_Init(&p_motor->SpeedRamp, p_motor->Config.SpeedRampTime_Cycles, Motor_GetSpeedRated_Fract16(&p_motor->Config.SpeedRating));
    // Motor_ResolveSpeedLimits(p_motor);
}

void Motor_InitTorqueRamp(Motor_Context_T * p_motor)
{
    Ramp_Init(&p_motor->TorqueRamp, p_motor->Config.TorqueRampTime_Cycles, Phase_Calibration_GetIRatedPeak_Fract16()); /* Current by default */
    // Motor_ResolveILimits(p_motor);
}

// void Motor_EnableSpeedRamp(Motor_Context_T * p_motor) { Motor_InitSpeedRamp(p_motor); }
// void Motor_DisableSpeedRamp(Motor_Context_T * p_motor) { _Ramp_Disable(&p_motor->SpeedRamp); }
// void Motor_EnableTorqueRamp(Motor_Context_T * p_motor) { Motor_InitTorqueRamp(p_motor); }
// void Motor_DisableTorqueRamp(Motor_Context_T * p_motor) { _Ramp_Disable(&p_motor->TorqueRamp); }

/* Reset from Derating */
/* use wider config window before direction are known */
// void Motor_ResetSpeedLimit(Motor_Context_T * p_motor)
// {
//     // p_motor->SpeedLimitCcw_Fract16 = p_motor->Config.SpeedLimitForward_Fract16;
//     // p_motor->SpeedLimitCw_Fract16 = -p_motor->Config.SpeedLimitForward_Fract16;
//     Ramp_SetLimits(&p_motor->SpeedRamp, -p_motor->Config.SpeedLimitForward_Fract16, p_motor->Config.SpeedLimitForward_Fract16);
// }

// void Motor_ResetILimit(Motor_Context_T * p_motor)
// {
//     // p_motor->ILimitCcw_Fract16 = p_motor->Config.ILimitMotoring_Fract16;
//     // p_motor->ILimitCw_Fract16 = -p_motor->Config.ILimitMotoring_Fract16;
//     Ramp_SetLimits(&p_motor->TorqueRamp, -p_motor->Config.ILimitMotoring_Fract16, p_motor->Config.ILimitMotoring_Fract16);
// }

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
    /* load from nvm to main consistency for save */
    p_motor->P_MOTOR->Config.PidSpeed = p_motor->P_NVM_CONFIG->PidSpeed;
    p_motor->P_MOTOR->Config.PidI = p_motor->P_NVM_CONFIG->PidI;
    Motor_ResetSpeedPid(p_motor->P_MOTOR);
    Motor_ResetIPid(p_motor->P_MOTOR);
}

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

/******************************************************************************/
/*
    FeedbackMode may update feedback limits
*/
/******************************************************************************/
void Motor_SetFeedbackMode(Motor_T * p_dev, Motor_FeedbackMode_T mode)
{
    Motor_Context_T * p_motor = p_dev->P_MOTOR;
    interval_t v = VBus_AntiPluggingLimits(p_dev->P_VBUS, (sign_t)p_dev->P_MOTOR->Direction);

    p_dev->P_MOTOR->FeedbackMode.Value = mode.Value;

    if (p_motor->FeedbackMode.Speed == 1U)
    {
        if (p_motor->FeedbackMode.Current == 1U) { PID_SetOutputLimits(&p_motor->PidSpeed, Motor_ILimitCw(p_motor), Motor_ILimitCcw(p_motor)); } /* SpeedPid Output is I */
        else                                     { PID_SetOutputLimits(&p_motor->PidSpeed, v.low, v.high); } /* SpeedPid Output is V */
    }

    if (p_motor->FeedbackMode.Current == 1U)    { Ramp_SetLimits(&p_motor->TorqueRamp, Motor_ILimitCw(p_motor), Motor_ILimitCcw(p_motor)); }
    else                                        { Ramp_SetLimits(&p_motor->TorqueRamp, v.low, v.high); } /* alternatively use Vramp */
    // else                                        { Ramp_SetLimits(&p_motor->VRamp, v.low, v.high); }
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
    Motor_ResolveSpeedLimits(p_dev);
    Motor_ResolveILimits(p_dev);
}


/******************************************************************************/
/*!
    Active Limits
*/
/******************************************************************************/
// static inline void _Motor_SetILimitCcw(const Motor_Context_T * p_motor, int16_t value) { return Ramp_SetLimitUpper(&p_motor->TorqueRamp); }
// static inline void _Motor_SetILimitCw(const Motor_Context_T * p_motor, int16_t value) { return Ramp_GetLimitLower(&p_motor->TorqueRamp); }
// static inline void _Motor_SetSpeedLimitCcw(const Motor_Context_T * p_motor, int16_t value) { return Ramp_GetLimitUpper(&p_motor->SpeedRamp); }
// static inline void _Motor_SetSpeedLimitCw(const Motor_Context_T * p_motor, int16_t value) { return Ramp_GetLimitLower(&p_motor->SpeedRamp); }

// /*
//     effective when system ResolveLimits is disabled.
//     Virtual fields resolved to ccw/cw limits on se
//     write direction-resolved Ccw/Cw directly.
// */
// /* Forward direction = Config.DirectionForward sign; lands in Ccw if forward==CCW, else in -Cw. */
// void _Motor_SetSpeedLimitForward(Motor_Context_T * p_motor, uint16_t speed_ufract16)
// {
//     ufract16_t v = math_min(speed_ufract16, p_motor->Config.SpeedLimitForward_Fract16);
//     if (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) { p_motor->SpeedLimitCcw_Fract16 = v; } else { p_motor->SpeedLimitCw_Fract16 = -v; }
// }

// void _Motor_SetSpeedLimitReverse(Motor_Context_T * p_motor, uint16_t speed_ufract16)
// {
//     ufract16_t v = math_min(speed_ufract16, p_motor->Config.SpeedLimitReverse_Fract16);
//     if (p_motor->Config.DirectionForward == MOTOR_DIRECTION_CCW) { p_motor->SpeedLimitCw_Fract16 = -v; } else { p_motor->SpeedLimitCcw_Fract16 = v; }
// }

// void _Motor_SetSpeedLimit(Motor_Context_T * p_motor, uint16_t speed_ufract16)
// {
//     _Motor_SetSpeedLimitForward(p_motor, speed_ufract16);
//     _Motor_SetSpeedLimitReverse(p_motor, speed_ufract16);
// }

// void _Motor_SetSpeedLimits(Motor_Context_T * p_motor, uint16_t speed_ufract16)
// {
//     switch (p_motor->Config.DirectionForward)
//     {
//     }
// }

// /* Motoring sign matches Direction: lands in Ccw if Direction==CCW, else in -Cw. */
// void _Motor_SetILimitMotoring(Motor_Context_T * p_motor, uint16_t i_fract16)
// {
//     ufract16_t i_mot = math_min(i_fract16, p_motor->Config.ILimitMotoring_Fract16);
//     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { p_motor->ILimitCcw_Fract16 = i_mot; } else { p_motor->ILimitCw_Fract16 = -i_mot; }
// }

// /* Generating sign opposes Direction: lands in -Cw if Direction==CCW, else in Ccw. */
// void _Motor_SetILimitGenerating(Motor_Context_T * p_motor, uint16_t i_fract16)
// {
//     ufract16_t i_gen = math_min(i_fract16, p_motor->Config.ILimitGenerating_Fract16);
//     if (p_motor->Direction == MOTOR_DIRECTION_CCW) { p_motor->ILimitCw_Fract16 = -i_gen; } else { p_motor->ILimitCcw_Fract16 = i_gen; }
// }

// void _Motor_SetILimit(Motor_Context_T * p_motor, uint16_t i_fract16)
// {
//     _Motor_SetILimitMotoring(p_motor, i_fract16);
//     _Motor_SetILimitGenerating(p_motor, i_fract16);
// }

// /*  Motor_GetILimits */
// void _Motor_SetILimits(Motor_Context_T * p_motor, uint16_t motoring, uint16_t generating)
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





