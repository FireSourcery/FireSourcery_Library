#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Motor_Calibration.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "../Motor_StateMachine.h"




/*
    Use inputId - Maps to multiple state. Substates use TransitionInput_T check for Parent State only
*/
static inline void Motor_Calibration_Enter(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&MOTOR_STATE_CALIBRATION); }
static inline void Motor_Calibration_Exit(const Motor_T * p_motor) { StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)NULL); }

/*  for Cmds that begin with a substate */
// static void Motor_Calibration_EnterBranch(const Motor_T * p_motor, State_T * p_subState) { _StateMachine_Branch_EnterSubstate(&p_motor->STATE_MACHINE, &MOTOR_STATE_CALIBRATION, p_subState); }

/* Calibration exits the substate and return to the parent state on complete */
/* IsComplete SubState = TopState = MOTOR_STATE_ID_CALIBRATION */
static inline bool Motor_Calibration_IsComplete(const Motor_T * p_motor) { return StateMachine_IsLeafState(p_motor->STATE_MACHINE.P_ACTIVE, &MOTOR_STATE_CALIBRATION); }

/*
    Calibration SubState
*/
typedef enum Motor_Calibration_StateId
{
    MOTOR_CALIBRATION_STATE_DISABLE,
    // MOTOR_CALIBRATION_STATE_SENSOR,
    MOTOR_CALIBRATION_STATE_TUNING,
    MOTOR_CALIBRATION_STATE_HOMING,
    MOTOR_CALIBRATION_STATE_ELECTRICAL,
}
Motor_Calibration_StateId_T;


/*

*/
extern void Motor_Calibration_StartHome(const Motor_T * p_motor);
extern void Motor_Calibration_EnterTuning(const Motor_T * p_motor);
extern bool Motor_Calibration_IsTuning(const Motor_T * p_motor);
extern void Motor_Calibration_StartElectrical(const Motor_T * p_motor);
extern bool Motor_Calibration_IsElectrical(const Motor_T * p_motor);

/******************************************************************************/
/*!
    Tuning excitation and capture.
    Drives the active loop's setpoint with a deterministic test signal and
    records (setpoint, feedback) per control tick. Buffer is shipped off-target
    (CAN/serial) for identification; gains are written back via Motor_Config.
*/
/******************************************************************************/
typedef enum Motor_Tuning_Excite
{
    MOTOR_TUNING_EXCITE_TRACK,      /* Manual: pass UserSpeedReq/UserTorqueReq through unchanged */
    MOTOR_TUNING_EXCITE_STEP,       /* Hold zero StepDelay ticks, then jump to Amplitude */
    MOTOR_TUNING_EXCITE_SQUARE,     /* +Amplitude / -Amplitude every HalfPeriod ticks */
}
Motor_Tuning_Excite_T;

typedef enum Motor_Tuning_Channel
{
    MOTOR_TUNING_CHANNEL_SPEED,     /* Drive UserSpeedReq, capture Speed_Fract16 */
    MOTOR_TUNING_CHANNEL_IQ,        /* Drive UserTorqueReq, capture FOC_Iq */
}
Motor_Tuning_Channel_T;

typedef struct Motor_Tuning_Config
{
    Motor_Tuning_Excite_T  Shape;
    Motor_Tuning_Channel_T Channel;
    int16_t                Amplitude;       /* fract16 setpoint amplitude */
    uint16_t               HalfPeriod;      /* control ticks per half-period (SQUARE) */
    uint16_t               StepDelay;       /* idle ticks before edge (STEP) */
}
Motor_Tuning_Config_T;

#define MOTOR_TUNING_CAPTURE_LEN 128U       /* @20kHz: 6.4ms window */

typedef struct Motor_Tuning_Capture
{
    int16_t  Setpoint[MOTOR_TUNING_CAPTURE_LEN];
    int16_t  Feedback[MOTOR_TUNING_CAPTURE_LEN];
    uint16_t Index;
    bool     Full;
}
Motor_Tuning_Capture_T;

extern void Motor_Calibration_EnterAutoTuning(const Motor_T * p_motor);
extern void Motor_Calibration_Tuning_ArmExcite(const Motor_T * p_motor, const Motor_Tuning_Config_T * p_config);
extern void Motor_Calibration_Tuning_Disarm(const Motor_T * p_motor);
extern bool Motor_Calibration_Tuning_IsCaptureDone(const Motor_T * p_motor);
extern const Motor_Tuning_Capture_T * Motor_Calibration_Tuning_GetCapture(const Motor_T * p_motor);

