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
    @file   .h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Motor_Analog.h"
#include "../StateMachine/Motor_StateMachine.h"


typedef struct CalibrationBuffer
{
    Accumulator_T FilterA;
    Accumulator_T FilterB;
    Accumulator_T FilterC;
}
CalibrationBuffer_T;

static_assert(sizeof(CalibrationBuffer_T) <= MOTOR_CALIBRATION_BUFFER_SIZE, "CalibrationBuffer_T must fit within MOTOR_CALIBRATION_BUFFER_SIZE");

/* allow adc module callback to skip storing raw value */
/* Phase_Input_Get Fract16 performed with 0 as zero reference */
static uint16_t AdcuOf(int16_t value) { return (uint16_t)(value / (PHASE_ANALOG_I_FRACT16_FACTOR * PHASE_ANALOG_I_POLARITY)); }

static CalibrationBuffer_T * GetBuffer(Motor_T * p_motor) { return (CalibrationBuffer_T *)p_motor->P_MOTOR->CalibrationBuffer; }

static Accumulator_T * GetFilterA(Motor_T * p_motor) { return &GetBuffer(p_motor)->FilterA; }
static Accumulator_T * GetFilterB(Motor_T * p_motor) { return &GetBuffer(p_motor)->FilterB; }
static Accumulator_T * GetFilterC(Motor_T * p_motor) { return &GetBuffer(p_motor)->FilterC; }

/******************************************************************************/
/*!
    @brief Calibration Routine via Motor_T StateMachine
*/
/******************************************************************************/
static void StartCalibration(Motor_T * p_motor)
{
    Motor_Context_T * const p_fields = p_motor->P_MOTOR;
    CalibrationBuffer_T * p_buffer = GetBuffer(p_motor);
    *p_buffer = (CalibrationBuffer_T){ 0 };

    // p_motor->P_MOTOR->ControlTimerBase = 0U;
    TimerT_OneShot_Start(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(2000U));

    Phase_WriteDuty_Fract16(&p_motor->PHASE, INT16_MAX / 2U, INT16_MAX / 2U, INT16_MAX / 2U);
    Phase_ActivateOutput(&p_motor->PHASE);

    Accumulator_Init(GetFilterA(p_motor));
    Accumulator_Init(GetFilterB(p_motor));
    Accumulator_Init(GetFilterC(p_motor));
    p_fields->Config.IabcZeroRef_Adcu.A = 0U;
    p_fields->Config.IabcZeroRef_Adcu.B = 0U;
    p_fields->Config.IabcZeroRef_Adcu.C = 0U;
    p_fields->PhaseInput.I.Flags.Bits = PHASE_ID_0;
    Motor_Analog_MarkIabc(p_motor);
}


static void ProcCalibration(Motor_T * p_motor)
{
    Motor_Context_T * const p_fields = p_motor->P_MOTOR;
    CalibrationBuffer_T * p_buffer = GetBuffer(p_motor);

    if (p_fields->PhaseInput.I.Flags.Bits == PHASE_ID_ABC)
    {
        Accumulator_Avg(GetFilterA(p_motor), AdcuOf(p_fields->PhaseInput.I.Values.A));
        Accumulator_Avg(GetFilterB(p_motor), AdcuOf(p_fields->PhaseInput.I.Values.B));
        Accumulator_Avg(GetFilterC(p_motor), AdcuOf(p_fields->PhaseInput.I.Values.C));
        p_fields->PhaseInput.I.Flags.Bits = PHASE_ID_0;
        Motor_Analog_MarkIabc(p_motor);
    }
}

static State_T * EndCalibration(Motor_T * p_motor)
{
    Motor_Context_T * const p_fields = p_motor->P_MOTOR;
    CalibrationBuffer_T * p_buffer = GetBuffer(p_motor);

    State_T * p_nextState = NULL;

    if (TimerT_IsElapsed(&p_motor->CONTROL_TIMER) == true)
    {
        // p_fields->Config.IabcZeroRef_Adcu.A = Accumulator_Avg(GetFilterA(p_motor), p_motor->P_MOTOR->ControlTimerBase, AdcuOf(p_fields->PhaseInput.I.Values.A));
        p_fields->Config.IabcZeroRef_Adcu.A = Accumulator_Avg(GetFilterA(p_motor), AdcuOf(p_fields->PhaseInput.I.Values.A));
        p_fields->Config.IabcZeroRef_Adcu.B = Accumulator_Avg(GetFilterB(p_motor), AdcuOf(p_fields->PhaseInput.I.Values.B));
        p_fields->Config.IabcZeroRef_Adcu.C = Accumulator_Avg(GetFilterC(p_motor), AdcuOf(p_fields->PhaseInput.I.Values.C));
        Phase_Deactivate(&p_motor->PHASE);
        p_nextState = &MOTOR_STATE_CALIBRATION; /* return to parent state, idle state */
    }

    return p_nextState;
}

/*  */
static const State_T CALIBRATION_STATE =
{
    .ID         = 0U,
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)StartCalibration,
    .LOOP       = (State_Action_T)ProcCalibration,
    .NEXT       = (State_Input0_T)EndCalibration,
};

static State_T * Calibration_Start(Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE; }

void Motor_Analog_Calibrate(Motor_T * p_motor)
{
    // StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE);
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Calibration_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}
