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
    @file   Motor_Config.c
    @author FireSourcery
    @brief @see Motor_Config.h
*/
/******************************************************************************/
#include "Motor_Config.h"

/******************************************************************************/
/*
    Nvm Config, check Stop State
    combine with VarConfig or UserConfig
*/
/******************************************************************************/
/*  */

static inline void PropagateSet(Motor_State_T * p_motor, Motor_Proc_T reset)
{
#ifdef CONFIG_MOTOR_CONFIG_PROPAGATE_SET_ENABLE
    reset(p_motor);
#else
    (void)p_motor;
    (void)reset;
#endif
}


// functions as full width variable to preserve calling convention, alternatively caller handle check set state

/* or use transition mapper */
/* or move interface to Var layer */
// void Motor_Config_SetWith(Motor_State_T * p_motor, State_Set_T setter, int32_t value)
// {
//     StateMachine_SetValueWith(&p_motor->StateMachine, &MOTOR_STATE_CALIBRATION, setter, value);
// }
/* alternatively TRANSITION_MAPPER */

/******************************************************************************/
/*
    Nvm Reference/Calibration
    Optionally propagate values during set, or wait for reboot
*/
/******************************************************************************/

/******************************************************************************/
/*
    Speed Position Calibration
*/
/******************************************************************************/
/* Reboot unless deinit is implemented in HAL */
void Motor_Config_SetSensorMode(Motor_State_T * p_motor, RotorSensor_Id_T mode)
{
    p_motor->Config.SensorMode = mode;
    PropagateSet(p_motor, Motor_ReinitSensor);
}

void Motor_Config_SetPolePairs(Motor_State_T * p_motor, uint8_t polePairs)
{
    p_motor->Config.PolePairs = polePairs;
    PropagateSet(p_motor, Motor_ResetUnits);
}

/* Setting Kv overwrites SpeedRefs. SpeedRefs can be set independently from Kv or lock */
// p_motor->Config.SpeedFeedback_Rpm = kv * MotorAnalog_GetVSource_V();
void Motor_Config_SetKv(Motor_State_T * p_motor, uint16_t kv)
{
    p_motor->Config.Kv = kv;
    PropagateSet(p_motor, Motor_ResetUnits);
}

void Motor_Config_SetSpeedRated(Motor_State_T * p_motor, uint16_t controlDeg)
{
    p_motor->Config.SpeedRated_DegPerCycle = math_max(controlDeg, Motor_GetSpeedVRef_DegPerCycle(p_motor));
    PropagateSet(p_motor, Motor_ResetUnits);
}


/*
    V of Speed Ref
    SpeedVRef =< SetSpeedFeedbackRef to ensure not match to higher speed
    Base as UFract16, multiply with SpeedRef*2 without overflow.
*/
void Motor_Config_SetVSpeedScalar_UFract16(Motor_State_T * p_motor, uint16_t scalar) { p_motor->Config.VSpeedScalar_Fract16 = math_min(scalar, INT16_MAX); }


#ifdef CONFIG_MOTOR_SIX_STEP_ENABLE
static inline void Motor_Config_SetPhaseMode(Motor_State_T * p_motor, Phase_Polar_Mode_T mode) { p_motor->Config.PhasePwmMode = mode; Phase_Polar_ActivateMode(&p_motor->PHASE, mode); }
#endif

/* Kv * VSource */
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
void Motor_Config_SetSpeedVRef_Rpm(Motor_State_T * p_motor, uint16_t rpm)
{
    // p_motor->Config.Kv = rpm / MotorAnalog_GetVSource_V();
    // PropagateSet(p_motor, Motor_ResetUnits);
    Motor_Config_SetKv(p_motor, MotorAnalog_GetVSource_V());
}

void Motor_Config_SetSpeedVMatchRef_Rpm(Motor_State_T * p_motor, uint16_t rpm)
{
    Motor_Config_SetVSpeedScalar_UFract16(p_motor, ((uint32_t)rpm << 15U) / Motor_GetSpeedVRef_Rpm(p_motor));
}
#endif

/*

*/
void Motor_Config_SetSpeedRated_Rpm(Motor_State_T * p_motor, uint16_t rpm)
{
    Motor_Config_SetSpeedRated(p_motor, speed_angle16_of_rpm(MOTOR_CONTROL_FREQ / p_motor->Config.PolePairs, rpm));
}

// void Motor_Config_SetSpeedRatedAsVRef(Motor_State_T * p_motor)
// {
//     p_motor->Config.SpeedRated_DegPerCycle = Motor_GetSpeedVSvpwmRef_DegPerCycle(p_motor);
// }




/******************************************************************************/
/*
    ISensorRef
*/
/******************************************************************************/
void Motor_Config_SetIaZero_Adcu(Motor_State_T * p_motor, uint16_t adcu) { p_motor->Config.IaZeroRef_Adcu = adcu; /* PropagateSet(p_motor, Motor_ResetUnitsIa); */ }
void Motor_Config_SetIbZero_Adcu(Motor_State_T * p_motor, uint16_t adcu) { p_motor->Config.IbZeroRef_Adcu = adcu; /* PropagateSet(p_motor, Motor_ResetUnitsIb); */ }
void Motor_Config_SetIcZero_Adcu(Motor_State_T * p_motor, uint16_t adcu) { p_motor->Config.IcZeroRef_Adcu = adcu; /* PropagateSet(p_motor, Motor_ResetUnitsIc); */ }


/******************************************************************************/
/*
    Persistent Base Limits / Ramps
*/
/******************************************************************************/

/*
    Persistent Base SpeedLimit
*/
void Motor_Config_SetSpeedLimitForward_Fract16(Motor_State_T * p_motor, uint16_t forward_Fract16)
{
    p_motor->Config.SpeedLimitForward_Fract16 = forward_Fract16;
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Config_SetSpeedLimitReverse_Fract16(Motor_State_T * p_motor, uint16_t reverse_Fract16)
{
    p_motor->Config.SpeedLimitReverse_Fract16 = reverse_Fract16;
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Config_SetSpeedLimitForward_DegPerCycle(Motor_State_T * p_motor, uint16_t value)
{
    // p_motor->Config.SpeedLimitForward_DegPerCycle = math_min(p_motor->Config.SpeedRated_DegPerCycle, value);
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

void Motor_Config_SetSpeedLimitReverse_DegPerCycle(Motor_State_T * p_motor, uint16_t value)
{
    // p_motor->Config.SpeedLimitReverse_DegPerCycle = math_min(p_motor->Config.SpeedRated_DegPerCycle, value);
    PropagateSet(p_motor, Motor_ResetSpeedLimitActive);
}

/*
    Persistent Base ILimit
*/
void Motor_Config_SetILimitMotoring_Fract16(Motor_State_T * p_motor, uint16_t motoring_Fract16)
{
    p_motor->Config.ILimitMotoring_Fract16 = Motor_IRatedLimitOf(motoring_Fract16);
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

void Motor_Config_SetILimitGenerating_Fract16(Motor_State_T * p_motor, uint16_t generating_Fract16)
{
    p_motor->Config.ILimitGenerating_Fract16 = Motor_IRatedLimitOf(generating_Fract16);
    PropagateSet(p_motor, Motor_ResetILimitActive);
}

/*
    Ramps
*/
void Motor_Config_SetSpeedRampTime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.SpeedRampTime_Cycles = cycles;
    PropagateSet(p_motor, Motor_ResetSpeedRamp);
}

void Motor_Config_SetSpeedRampTime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetSpeedRampTime_Cycles(p_motor, MOTOR_SPEED_CYCLES(millis));
}

void Motor_Config_SetTorqueRampTime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.TorqueRampTime_Cycles = cycles;
    PropagateSet(p_motor, Motor_ResetTorqueRamp);
}

void Motor_Config_SetTorqueRampTime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetTorqueRampTime_Cycles(p_motor, MOTOR_TORQUE_CYCLES(millis));
}


/******************************************************************************/
/*

*/
/******************************************************************************/
/* call getter each time for config */
static inline uint16_t Motor_Config_OpenLoopILimitOf(Motor_State_T * p_motor, uint16_t fract16) { return math_min(fract16, _Motor_Config_GetOpenLoopILimit_Fract16(p_motor)); }


/*  */
void Motor_Config_SetOpenLoopScalarLimit(Motor_State_T * p_motor, uint16_t scalar16)
{
    p_motor->Config.OpenLoopScalarLimit_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);
    p_motor->Config.AlignScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);;
    p_motor->Config.OpenLoopRampIFinal_Fract16 = Motor_Config_OpenLoopILimitOf(p_motor, p_motor->Config.OpenLoopRampIFinal_Fract16);
}


/* */
void Motor_Config_SetAlignPowerScalar(Motor_State_T * p_motor, uint16_t scalar16) { p_motor->Config.AlignScalar_Fract16 = math_min(scalar16, MOTOR_OPEN_LOOP_MAX_SCALAR);; }

void Motor_Config_SetAlignTime_Cycles(Motor_State_T * p_motor, uint32_t cycles) { p_motor->Config.AlignTime_Cycles = cycles; }
void Motor_Config_SetAlignTime_Millis(Motor_State_T * p_motor, uint16_t millis) { p_motor->Config.AlignTime_Cycles = _Motor_ControlCyclesOf(millis); }


/*
*/
void Motor_Config_SetOpenLoopRampIFinal_Fract16(Motor_State_T * p_motor, uint16_t i_fract16)
{
    p_motor->Config.OpenLoopRampIFinal_Fract16 = Motor_Config_OpenLoopILimitOf(p_motor, i_fract16);
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampITime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.OpenLoopRampI_Cycles = cycles;
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampITime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetOpenLoopRampITime_Cycles(p_motor, _Motor_ControlCyclesOf(millis));
}

/*
    OpenLoop Ramp tick on control timer
*/
void Motor_Config_SetOpenLoopRampSpeedFinal_Fract16(Motor_State_T * p_motor, uint16_t speed_fract16)
{
    // p_motor->Config.OpenLoopRampSpeedFinal_Fract16 = speed_fract16; /* optionally limit */
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}


void Motor_Config_SetOpenLoopRampSpeedTime_Cycles(Motor_State_T * p_motor, uint32_t cycles)
{
    p_motor->Config.OpenLoopRampSpeed_Cycles = cycles;
    // PropagateSet(p_motor, Motor_ResetOpenLoopRamp);
}

void Motor_Config_SetOpenLoopRampSpeedTime_Millis(Motor_State_T * p_motor, uint16_t millis)
{
    Motor_Config_SetOpenLoopRampSpeedTime_Millis(p_motor, _Motor_ControlCyclesOf(millis));
}



/******************************************************************************/
/* Local Unit Conversion */
/******************************************************************************/
// #ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
// static uint16_t ConvertToSpeedLimitPercent16(Motor_State_T * p_motor, uint16_t speed_rpm)
// {
//     int32_t speed_fract16 = _Motor_ConvertSpeed_RpmToPercent16(p_motor, speed_rpm);
//     return (speed_fract16 > UINT16_MAX) ? UINT16_MAX : speed_fract16;
// }

// void Motor_Config_SetSpeedLimit_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
// {
//     Motor_Config_SetSpeedLimit_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, forward_Rpm), ConvertToSpeedLimitPercent16(p_motor, reverse_Rpm));
// }

// void Motor_Config_SetSpeedLimitForward_Rpm(Motor_State_T * p_motor, uint16_t forward_Rpm)
// {
//     Motor_Config_SetSpeedLimitForward_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, forward_Rpm));
// }

// void Motor_Config_SetSpeedLimitReverse_Rpm(Motor_State_T * p_motor, uint16_t reverse_Rpm)
// {
//     Motor_Config_SetSpeedLimitReverse_Fract16(p_motor, ConvertToSpeedLimitPercent16(p_motor, reverse_Rpm));
// }

// uint16_t Motor_Config_GetSpeedLimitForward_Rpm(Motor_State_T * p_motor)
// {
//     return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, p_motor->Config.SpeedLimitForward_Fract16);
//     // uint16_t speedLimit = (p_motor->Config.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
//     //     p_motor->Config.SpeedLimitCcw_Fract16 : p_motor->Config.SpeedLimitCw_Fract16;
//     // return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, speedLimit);
// }

// uint16_t Motor_Config_GetSpeedLimitReverse_Rpm(Motor_State_T * p_motor)
// {
//     return _Motor_ConvertSpeed_Percent16ToRpm(p_motor, p_motor->Config.SpeedLimitReverse_Fract16);
//     // uint16_t speedLimit = (p_motor->Config.DirectionForward == MOTOR_FORWARD_IS_CCW) ?
//     //  p_motor->Config.SpeedLimitCw_Fract16 : p_motor->Config.SpeedLimitCcw_Fract16;
// }

// static uint16_t ConvertToILimitPercent16(Motor_State_T * p_motor, uint16_t i_amp)
// {
//     int32_t i_fract16 = _Motor_ConvertI_AmpToPercent16(i_amp);
//     return (i_fract16 > UINT16_MAX) ? UINT16_MAX : i_fract16;
// }

// void Motor_Config_SetILimit_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
// {
//     Motor_Config_SetILimit_Fract16(p_motor, ConvertToILimitPercent16(p_motor, motoring_Amp), ConvertToILimitPercent16(p_motor, generating_Amp));
// }

// void Motor_Config_SetILimitMotoring_Amp(Motor_State_T * p_motor, uint16_t motoring_Amp)
// {
//     Motor_Config_SetILimitMotoring_Fract16(p_motor, ConvertToILimitPercent16(p_motor, motoring_Amp));
// }

// void Motor_Config_SetILimitGenerating_Amp(Motor_State_T * p_motor, uint16_t generating_Amp)
// {
//     Motor_Config_SetILimitGenerating_Fract16(p_motor, ConvertToILimitPercent16(p_motor, generating_Amp));
// }

// uint16_t Motor_Config_GetILimitMotoring_Amp(Motor_State_T * p_motor) { return _Motor_ConvertI_Percent16ToAmp(p_motor->Config.ILimitMotoring_Fract16); }
// uint16_t Motor_Config_GetILimitGenerating_Amp(Motor_State_T * p_motor) { return _Motor_ConvertI_Percent16ToAmp(p_motor->Config.ILimitGenerating_Fract16); }
// #endif