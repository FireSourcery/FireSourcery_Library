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
    @version V0
*/
/******************************************************************************/
#include "Motor_Analog.h"
#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif

/******************************************************************************/
/*!
    @brief  Callback functions
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @brief  Vabc
*/
/******************************************************************************/
void Motor_Analog_CaptureVa(MotorPtr_T p_motor) { Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureVa, 0U/* Motor_SixStep_CaptureBemfA */); }
void Motor_Analog_CaptureVb(MotorPtr_T p_motor) { Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureVb, 0U/* Motor_SixStep_CaptureBemfB */); }
void Motor_Analog_CaptureVc(MotorPtr_T p_motor) { Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureVc, 0U/* Motor_SixStep_CaptureBemfC */); }

/******************************************************************************/
/*!
    @brief  Iabc
*/
/******************************************************************************/
void Motor_Analog_CaptureIa(MotorPtr_T p_motor) { Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureIa, 0U/* Motor_SixStep_CaptureIa */); }
void Motor_Analog_CaptureIb(MotorPtr_T p_motor) { Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureIb, 0U/* Motor_SixStep_CaptureIb */); }
void Motor_Analog_CaptureIc(MotorPtr_T p_motor) { Motor_ProcCommutationMode(p_motor, Motor_FOC_CaptureIc, 0U/* Motor_SixStep_CaptureIc */); }


/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
static inline bool CheckDivider(MotorPtr_T p_motor) { return ((p_motor->ControlTimerBase & GLOBAL_MOTOR.CONTROL_ANALOG_DIVIDER) == 0UL); }

void Motor_Analog_EnqueueVabc(MotorPtr_T p_motor)
{
#if defined(CONFIG_MOTOR_V_SENSORS_ANALOG)
    if(CheckDivider(p_motor) == true)
    {
        AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VA);
        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VB);
        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VC);
        AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
    }
#else
    (void)p_motor;
#endif
}

void Motor_Analog_EnqueueIabc(MotorPtr_T p_motor)
{
    if(CheckDivider(p_motor) == true)
    {
        AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IA);
        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IB);
#if defined(CONFIG_MOTOR_I_SENSORS_ABC)
        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_IC);
#endif
        AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_I);
        // AnalogN_SetChannelConversion(p_motor->CONFIG.P_ANALOG_N, MOTOR_ANALOG_CHANNEL_VA);
    }
}


void Motor_Analog_Proc(MotorPtr_T p_motor)
{
//    AnalogN_Group_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);

//    if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
//    {
//        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_SIN);
//        AnalogN_Group_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.ANALOG_CONVERSIONS.CONVERSION_COS);
//    }
//    switch(p_motor->AnalogCmd)
//    {
//     //    case FOC_I_ABC :
//     //        break;

//     //    case FOC_VBEMF :
//     //        break;

//     //    default :
//     //        break;
//    }

//    AnalogN_Group_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_PWM);
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
void Motor_Analog_StartCalibration(MotorPtr_T p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, GLOBAL_MOTOR.CONTROL_FREQ * 2U); /* 2 Seconds */
    Motor_ProcCommutationMode(p_motor, Motor_FOC_ActivateOutput, 0U /*Phase_Ground(&p_motor->Phase)*/);

    Filter_InitAvg(&p_motor->FilterA);
    Filter_InitAvg(&p_motor->FilterB);
    Filter_InitAvg(&p_motor->FilterC);
    p_motor->AnalogResults.Ia_Adcu = 0U;
    p_motor->AnalogResults.Ib_Adcu = 0U;
    p_motor->AnalogResults.Ic_Adcu = 0U;
    Motor_Analog_EnqueueIabc(p_motor);
}

bool Motor_Analog_ProcCalibration(MotorPtr_T p_motor)
{
    static const uint32_t DIVIDER = (GLOBAL_MOTOR.CONTROL_ANALOG_DIVIDER << 1U) & 1U; /* 2x normal sample time */
    bool isComplete = Timer_Periodic_Poll(&p_motor->ControlTimer);
    if (isComplete == true)
    {
        p_motor->Parameters.IaZeroRef_Adcu = Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_Adcu);
        p_motor->Parameters.IbZeroRef_Adcu = Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_Adcu);
        p_motor->Parameters.IcZeroRef_Adcu = Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_Adcu);
        Motor_ResetUnitsIabc(p_motor);
        Phase_Float(&p_motor->Phase);
    }
    else
    {
        if((p_motor->ControlTimerBase & DIVIDER) == 0UL)
        {
            Filter_Avg(&p_motor->FilterA, p_motor->AnalogResults.Ia_Adcu);
            Filter_Avg(&p_motor->FilterB, p_motor->AnalogResults.Ib_Adcu);
            Filter_Avg(&p_motor->FilterC, p_motor->AnalogResults.Ic_Adcu);
            Motor_Analog_EnqueueIabc(p_motor);
        }
    }

    return isComplete;
}