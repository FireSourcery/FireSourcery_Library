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
#include "../Motor_StateMachine.h"

/******************************************************************************/
/*!
    @brief Adc Capture
*/
/******************************************************************************/
static inline fract16_t _Motor_Analog_VFract16Of(uint16_t adcu) { return adcu * MOTOR_ANALOG_V_FRACT16_ADCU_SCALAR; }
static inline fract16_t _Motor_Analog_IFract16Of(uint16_t zero, uint16_t adcu) { return ((int16_t)adcu - zero) * MOTOR_ANALOG_I_FRACT16_ADCU_SCALAR; }

void Motor_Analog_CaptureVa(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Va_Fract16 = _Motor_Analog_VFract16Of(adcu); p_motor->VBatch.A = 1U; }
void Motor_Analog_CaptureVb(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Vb_Fract16 = _Motor_Analog_VFract16Of(adcu); p_motor->VBatch.B = 1U; }
void Motor_Analog_CaptureVc(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Vc_Fract16 = _Motor_Analog_VFract16Of(adcu); p_motor->VBatch.C = 1U; }
void Motor_Analog_CaptureIa(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Ia_Fract16 = _Motor_Analog_IFract16Of(p_motor->Config.IaZeroRef_Adcu, adcu); p_motor->IBatch.A = 1U; }
void Motor_Analog_CaptureIb(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Ib_Fract16 = _Motor_Analog_IFract16Of(p_motor->Config.IbZeroRef_Adcu, adcu); p_motor->IBatch.B = 1U; }
void Motor_Analog_CaptureIc(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Ic_Fract16 = _Motor_Analog_IFract16Of(p_motor->Config.IcZeroRef_Adcu, adcu); p_motor->IBatch.C = 1U; }
void Motor_Analog_CaptureHeat(Motor_State_T * p_motor, adc_result_t adcu) { p_motor->AnalogState.Heat = adcu; }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
/* Without checking for previous completion. Conversions must complete within the analog cycle */
void Motor_Analog_MarkVabc(const Motor_T * p_motorConst)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    Analog_Conversion_MarkConversion(&p_motorConst->ANALOG.CONVERSION_VA);
    Analog_Conversion_MarkConversion(&p_motorConst->ANALOG.CONVERSION_VB);
    Analog_Conversion_MarkConversion(&p_motorConst->ANALOG.CONVERSION_VC);
#else
    (void)p_motorConst;
#endif
}

// void Motor_Analog_MarkVabc(Motor_Analog_T * p_channels)
// {
// #if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
//     Analog_Conversion_MarkConversion(&p_channels->CONVERSION_VA);
//     Analog_Conversion_MarkConversion(&p_channels->CONVERSION_VB);
//     Analog_Conversion_MarkConversion(&p_channels->CONVERSION_VC);
// #else
//     (void)p_channels;
// #endif
// }

void Motor_Analog_MarkIabc(const Motor_T * p_motorConst)
{
    Analog_Conversion_MarkConversion(&p_motorConst->ANALOG.CONVERSION_IA);
    Analog_Conversion_MarkConversion(&p_motorConst->ANALOG.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
    Analog_Conversion_MarkConversion(&p_motorConst->ANALOG.CONVERSION_IC);
#endif
}


/******************************************************************************/
/*!
    @brief Calibration Routine via Motor_T StateMachine
*/
/******************************************************************************/
static void StartCalibration(const Motor_T * p_motor)
{
    Motor_State_T * p_fields = p_motor->P_ACTIVE;

    Timer_StartPeriod_Millis(&p_motor->P_ACTIVE->ControlTimer, 2000U); /* 2 Seconds */

    Phase_WriteDuty_Fract16(&p_motor->PHASE, INT16_MAX / 2U, INT16_MAX / 2U, INT16_MAX / 2U);
    Phase_ActivateOutput(&p_motor->PHASE);

    Filter_Init(&p_motor->P_ACTIVE->FilterA);
    Filter_Init(&p_motor->P_ACTIVE->FilterB);
    Filter_Init(&p_motor->P_ACTIVE->FilterC);
    p_motor->P_ACTIVE->Config.IaZeroRef_Adcu = 0U;
    p_motor->P_ACTIVE->Config.IbZeroRef_Adcu = 0U;
    p_motor->P_ACTIVE->Config.IcZeroRef_Adcu = 0U;
    Motor_Analog_MarkIabc(p_motor);
    p_motor->P_ACTIVE->IBatch.Value = PHASE_ID_0;
}

static void ProcCalibration(const Motor_T * p_motor)
{
    if (p_motor->P_ACTIVE->IBatch.Value == PHASE_ID_ABC)
    {
        Filter_Avg(&p_motor->P_ACTIVE->FilterA, MotorAnalog_GetIa_Fract16(&p_motor->P_ACTIVE->AnalogState));
        Filter_Avg(&p_motor->P_ACTIVE->FilterB, MotorAnalog_GetIb_Fract16(&p_motor->P_ACTIVE->AnalogState));
        Filter_Avg(&p_motor->P_ACTIVE->FilterC, MotorAnalog_GetIc_Fract16(&p_motor->P_ACTIVE->AnalogState));
        Motor_Analog_MarkIabc(p_motor);
        p_motor->P_ACTIVE->IBatch.Value = PHASE_ID_0;
    }
}

static State_T * EndCalibration(const Motor_T * p_motor)
{
    State_T * p_nextState = NULL;

    if (Timer_IsElapsed(&p_motor->P_ACTIVE->ControlTimer) == true)
    {
        p_motor->P_ACTIVE->Config.IaZeroRef_Adcu = Filter_Avg(&p_motor->P_ACTIVE->FilterA, MotorAnalog_GetIa_Fract16(&p_motor->P_ACTIVE->AnalogState));
        p_motor->P_ACTIVE->Config.IbZeroRef_Adcu = Filter_Avg(&p_motor->P_ACTIVE->FilterB, MotorAnalog_GetIb_Fract16(&p_motor->P_ACTIVE->AnalogState));
        p_motor->P_ACTIVE->Config.IcZeroRef_Adcu = Filter_Avg(&p_motor->P_ACTIVE->FilterC, MotorAnalog_GetIc_Fract16(&p_motor->P_ACTIVE->AnalogState));
        // Motor_ResetUnitsIabc(p_motor->p_Analog);
        Phase_Float(&p_motor->PHASE);
    }

    return p_nextState;
}

static const State_T CALIBRATION_STATE =
{
    .P_TOP      = &MOTOR_STATE_CALIBRATION,
    .P_PARENT   = &MOTOR_STATE_CALIBRATION,
    .DEPTH      = 1U,
    .ENTRY      = (State_Action_T)StartCalibration,
    .LOOP       = (State_Action_T)ProcCalibration,
    .NEXT       = (State_InputVoid_T)EndCalibration,
};

void Motor_Analog_Calibrate(const Motor_T * p_motor)
{
    StateMachine_ProcBranchInput(&p_motor->STATE_MACHINE, MSM_INPUT_CALIBRATION, (uintptr_t)&CALIBRATION_STATE);
}
