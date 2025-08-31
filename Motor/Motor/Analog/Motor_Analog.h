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
    @file   Motor_Analog.h
    @author FireSourcery
    @brief  ADC Conversions
            - callbacks on Motor_State_T
*/
/******************************************************************************/
#include "Peripheral/Analog/Analog.h"
#include "../Motor.h"


/* Capture with context of IZeroRef */
static inline void Motor_Analog_CaptureVa(Motor_State_T * p_motor, adc_result_t adcu) { Phase_Analog_CaptureVa(&p_motor->PhaseInput, adcu); }
static inline void Motor_Analog_CaptureVb(Motor_State_T * p_motor, adc_result_t adcu) { Phase_Analog_CaptureVb(&p_motor->PhaseInput, adcu); }
static inline void Motor_Analog_CaptureVc(Motor_State_T * p_motor, adc_result_t adcu) { Phase_Analog_CaptureVc(&p_motor->PhaseInput, adcu); }
static inline void Motor_Analog_CaptureIa(Motor_State_T * p_motor, adc_result_t adcu) { Phase_Analog_CaptureIa(&p_motor->PhaseInput, &p_motor->Config.IabcZeroRef_Adcu, adcu); }
static inline void Motor_Analog_CaptureIb(Motor_State_T * p_motor, adc_result_t adcu) { Phase_Analog_CaptureIb(&p_motor->PhaseInput, &p_motor->Config.IabcZeroRef_Adcu, adcu); }
static inline void Motor_Analog_CaptureIc(Motor_State_T * p_motor, adc_result_t adcu) { Phase_Analog_CaptureIc(&p_motor->PhaseInput, &p_motor->Config.IabcZeroRef_Adcu, adcu); }

/* Without checking for previous completion. Conversions must complete within the analog cycle */
static inline void Motor_Analog_MarkVabc(const Motor_T * p_context) { Phase_Analog_MarkVabc(&p_context->PHASE_ANALOG); }
static inline void Motor_Analog_MarkIabc(const Motor_T * p_context) { Phase_Analog_MarkIabc(&p_context->PHASE_ANALOG); }


/*
*/
extern void Motor_Analog_Calibrate(const Motor_T * p_context);

