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
#include "Motor_ADC.h"
#include "../Motor_StateMachine.h"
#include "Math/Filter/MovAvg.h"

typedef struct CalibrationBuffer
{
    MovAvg_T FilterA;
    MovAvg_T FilterB;
    MovAvg_T FilterC;
}
CalibrationBuffer_T;

static_assert(sizeof(CalibrationBuffer_T) <= MOTOR_CALIBRATION_BUFFER_SIZE, "CalibrationBuffer_T must fit within MOTOR_CALIBRATION_BUFFER_SIZE");

/*
    The ADC keeps every channel's last count [ADC_Conversion_GetResult]: the routine averages the counts themselves.
    The capture's pu set only signals that the three landed [PhaseInput.I.Flags]. Config keeps its zero until the
    routine completes, so an abort (Z or the controller's fault) leaves the last calibration in place.
*/
static CalibrationBuffer_T * GetBuffer(Motor_T * p_motor) { return (CalibrationBuffer_T *)p_motor->P_MOTOR->CalibrationBuffer; }

/******************************************************************************/
/*!
    @brief Calibration Routine via Motor_T StateMachine
*/
/******************************************************************************/
static void StartCalibration(Motor_T * p_motor)
{
    Motor_Context_T * const p_context = p_motor->P_MOTOR;
    CalibrationBuffer_T * p_buffer = GetBuffer(p_motor);
    *p_buffer = (CalibrationBuffer_T){ 0 };

    TimerT_OneShot_Start(&p_motor->CONTROL_TIMER, MOTOR_CONTROL_CYCLES(2000U));
    Phase_ActivateT0(&p_motor->PHASE);

    p_context->PhaseInput.I.Flags.Bits = PHASE_ID_0;
    Motor_ADC_MarkIabc(p_motor);
}


static void ProcCalibration(Motor_T * p_motor)
{
    Motor_Context_T * const p_context = p_motor->P_MOTOR;

    if (p_context->PhaseInput.I.Flags.Bits == PHASE_ID_ABC)
    {
        MovAvg_Running(&GetBuffer(p_motor)->FilterA, ADC_Conversion_GetResult(&p_motor->PHASE_ADC.IA));
        MovAvg_Running(&GetBuffer(p_motor)->FilterB, ADC_Conversion_GetResult(&p_motor->PHASE_ADC.IB));
        MovAvg_Running(&GetBuffer(p_motor)->FilterC, ADC_Conversion_GetResult(&p_motor->PHASE_ADC.IC));
        p_context->PhaseInput.I.Flags.Bits = PHASE_ID_0;
        Motor_ADC_MarkIabc(p_motor);
    }
}

static State_T * EndCalibration(Motor_T * p_motor)
{
    Motor_Context_T * const p_context = p_motor->P_MOTOR;

    State_T * p_nextState = NULL;

    if (TimerT_IsElapsed(&p_motor->CONTROL_TIMER) == true)
    {
        p_context->Config.IabcZeroRef_Adcu.A = MovAvg_Running(&GetBuffer(p_motor)->FilterA, ADC_Conversion_GetResult(&p_motor->PHASE_ADC.IA));
        p_context->Config.IabcZeroRef_Adcu.B = MovAvg_Running(&GetBuffer(p_motor)->FilterB, ADC_Conversion_GetResult(&p_motor->PHASE_ADC.IB));
        p_context->Config.IabcZeroRef_Adcu.C = MovAvg_Running(&GetBuffer(p_motor)->FilterC, ADC_Conversion_GetResult(&p_motor->PHASE_ADC.IC));
        Phase_Deactivate(&p_motor->PHASE);
        p_nextState = &MOTOR_STATE_CALIBRATION; /* return to parent state, idle state */
    }

    return p_nextState;
}

/*  */
static const State_T CALIBRATION_STATE =
{
    .ID         = 0U,
    .PATH_ID      = { .Depth0 = MOTOR_STATE_ID_CALIBRATION }, /* sub-state id not yet allocated: reports as the root state */
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)StartCalibration,
    .LOOP       = (State_Action_T)ProcCalibration,
    .NEXT       = (State_Input0_T)EndCalibration,
};

static State_T * Calibration_Start(Motor_T * p_motor, state_value_t value) { (void)p_motor; (void)value; return &CALIBRATION_STATE; }

void Motor_ADC_Calibrate(Motor_T * p_motor)
{
    // StateMachine_Tree_Input(&p_motor->STATE_MACHINE, MOTOR_STATE_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE);
    static StateMachine_TransitionCmd_T CMD = { .P_START = &MOTOR_STATE_CALIBRATION, .NEXT = (State_Input_T)Calibration_Start };
    StateMachine_Tree_InvokeTransition(&p_motor->STATE_MACHINE, &CMD, 0U);
}
