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
    @file   Encoder.c
    @author FireSourcery
    @brief  Encoder

*/
/******************************************************************************/
#include "Encoder.h"
#include "Math/Angle/AngleCounter.h"


/******************************************************************************/
/*!
    Init
*/
/******************************************************************************/
void Encoder_InitInterrupts_Quadrature(const Encoder_T * p_encoder)
{
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID);
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID);
    HAL_Encoder_InitPinInterruptRisingEdge(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID);
}

void Encoder_InitInterrupts_Incremental(const Encoder_T * p_encoder)
{
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID);
}



// static inline void Encoder_Quadrature_InitDirection(Encoder_State_T * p_encoder) { p_encoder->DirectionComp = (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }

void Encoder_InitCounter(const Encoder_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_InitCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounterMax(p_encoder->P_HAL_ENCODER_COUNTER, p_encoder->P_STATE->Config.CountsPerRevolution - 1U);
#elif   defined(ENCODER_HW_EMULATED)
    if (p_encoder->P_STATE->Config.IsQuadratureCaptureEnabled == true)
    {
        Pin_Input_Init(&p_encoder->PIN_A);
        Pin_Input_Init(&p_encoder->PIN_B);
    }
#endif
}

void Encoder_SetCounterInitial(const Encoder_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    HAL_Encoder_ClearCounterOverflow(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0U);
    p_encoder->P_STATE->IndexCount = 0U;
#elif   defined(ENCODER_HW_EMULATED)
#endif
    _Encoder_ZeroPulseCount(p_encoder->P_STATE);
}


/******************************************************************************/
/*!
    Set reference by homing or aligning
*/
/******************************************************************************/
void Encoder_StartHoming(Encoder_State_T * p_encoder)
{
    p_encoder->IndexCount = 0U;
    p_encoder->IndexAngleError = 0U;
    p_encoder->IsHomed = false;
    AngleCounter_Zero(&p_encoder->AngleCounter);
    Angle_ZeroCaptureState(&p_encoder->AngleCounter.Base);
}

uint16_t Encoder_GetHomingAngle(const Encoder_State_T * p_encoder)
{
    return ENCODER_ANGLE_DEGREES / math_max(p_encoder->Config.CountsPerRevolution, 1000);
}

bool Encoder_IsHomingIndexFound(const Encoder_State_T * p_encoder)
{
    return p_encoder->IndexCount > 0U;
//     return p_encoder->Angle32 == p_encoder->Config.IndexAngleRef;
}

bool Encoder_IsHomingIndexError(const Encoder_State_T * p_encoder)
{
    return (math_abs(p_encoder->AngleCounter.CounterD) > (int32_t)p_encoder->Config.CountsPerRevolution);
}

//enum Encoder_HomingStatus { Encoder_HomingStatus_None, Encoder_HomingStatus_Found, Encoder_HomingStatus_Error };
bool Encoder_PollHomingComplete(Encoder_State_T * p_encoder)
{
    bool isComplete = false;

    if (Encoder_IsHomingIndexFound(p_encoder) == true)
    {
        // p_encoder->IndexCount = 0U;
        // p_encoder->AngleCounter.CounterD = 0U;
        p_encoder->IsHomed = true;
        isComplete = true;
    }
    else if (Encoder_IsHomingIndexError(p_encoder) == true)
    {
        // p_encoder->IndexCount = 0U;
        // p_encoder->AngleCounter.CounterD = 0U;
        p_encoder->IsHomed = false;
        isComplete = true;
    }

    return isComplete;
}

// bool Encoder_ProcHomingVirtualIndex(Encoder_State_T * p_encoder)
// {
//     // bool isVirtualIndex = p_encoder->Angle32 < p_encoder->Config.IndexAngleRef;
//     // bool isVirtualIndex = p_encoder->Angle32 > p_encoder->Config.IndexAngleRef;
// }

/*
    Calibrate IndexAngleRef from the last homing result.
    IndexAngleError = Angle32(before ISR) - Config.IndexAngleRef(old)
    Accumulate correction so future ISR snaps to the true index angle.
    Call after homing completes and motor alignment is known.
*/
void Encoder_CalibrateIndexZeroRef(Encoder_State_T * p_encoder)
{
    if (p_encoder->IsHomed == true)
    {
        p_encoder->Config.IndexAngleRef += p_encoder->IndexAngleError;
        p_encoder->IndexAngleError = 0U;
    }
}
// // Encoder_SetIndexZeroRef(p_encoder, p_encoder->Angle32);
// // p_encoder->Config.IndexAngleRef = p_encoder->Angle32 - p_encoder->IndexAngleRef;
// // p_encoder->IndexAngleRef = p_encoder->Config.IndexAngleRef;
// p_encoder->Config.IndexAngleRef = p_encoder->Angle32 - p_encoder->Config.IndexAngleRef;
// p_encoder->Angle32 = 0U;



/******************************************************************************/
/*!
    Align to phase without homing
*/
/******************************************************************************/
/*
    Load saved AlignOffsetRef from NvMem Config on init.
    If homed with a stored offset, restore it so aligned angle is usable immediately.
*/
void Encoder_CheckAlignRef(Encoder_State_T * p_encoder)
{
    if (p_encoder->IsHomed == true)
    {
        p_encoder->AlignOffsetRef = p_encoder->Config.AlignOffsetRef;
        p_encoder->Align = ENCODER_ALIGN_PHASE;
    }
}

//
// void Encoder_CalibrateAlignRef(Encoder_State_T * p_encoder)
// {
//     // p_encoder->Angle32 = 0U;
//     p_encoder->AlignOffsetRef = p_encoder->Angle32 /* - p_encoder->Config.IndexAngleRef */;
//     p_encoder->Config.AlignOffsetRef = p_encoder->AlignOffsetRef;
//     _Encoder_ZeroPulseCount(p_encoder);
// }

/*
    Capture the current Angle32 as the phase-A zero reference offset.
    Call after energizing phase A and waiting for the shaft to settle.
    If homed, save to NvMem Config for persistent calibration.
*/
void Encoder_CaptureAlignZero(Encoder_State_T * p_encoder)
{
    p_encoder->AlignOffsetRef = p_encoder->AngleCounter.Base.Angle;
    p_encoder->AlignAngle = p_encoder->AngleCounter.Base.Angle;
    if (p_encoder->IsHomed == true)
    {
        p_encoder->Config.AlignOffsetRef = p_encoder->AlignOffsetRef;
    }
}

/* this way angle starts from a known pole */
uint16_t Encoder_GetAngleAligned(const Encoder_State_T * p_encoder)
{
    return (p_encoder->AngleCounter.Base.Angle - p_encoder->AlignOffsetRef) >> ENCODER_ANGLE_SHIFT;
}


/*
    Validate alignment by checking if the angle drifted from the captured AlignAngle.
    Call after open-loop settle following alignment.
    Returns true if angle is within one encoder step of alignment reference.
*/
bool Encoder_ProcAlignValidate(Encoder_State_T * p_encoder)
{
    (void)p_encoder;
    // uint32_t angleDiff = (p_encoder->AngleCounter.Base.Angle > p_encoder->AlignAngle) ? (p_encoder->AngleCounter.Base.Angle - p_encoder->AlignAngle) : (p_encoder->AlignAngle - p_encoder->AngleCounter.Base.Angle);
    // return (angleDiff <= p_encoder->AngleCounter.Ref.Angle32PerCount);
    return true;
}

void Encoder_CompleteAlignValidate(Encoder_State_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_PHASE;
    _Encoder_ZeroPulseCount(p_encoder);
}

void Encoder_ClearAlign(Encoder_State_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_NONE;
}


/******************************************************************************/
/*!
    Calibration for Reference
    Determine the values initially
*/
/******************************************************************************/
void Encoder_SetQuadratureMode(Encoder_State_T * p_encoder, bool isEnabled) { p_encoder->Config.IsQuadratureCaptureEnabled = isEnabled; }
void Encoder_EnableQuadratureMode(Encoder_State_T * p_encoder) { p_encoder->Config.IsQuadratureCaptureEnabled = true; }
void Encoder_DisableQuadratureMode(Encoder_State_T * p_encoder) { p_encoder->Config.IsQuadratureCaptureEnabled = false; }
/*! isALeadBPositive - User runtime calibrate */
void Encoder_SetQuadratureDirection(Encoder_State_T * p_encoder, bool isALeadBPositive) { p_encoder->Config.IsALeadBPositive = isALeadBPositive; }

/*
    Run on calibration routine start
*/
void Encoder_CaptureQuadratureReference(Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    p_encoder->AngleCounter.CounterD = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0);
#elif   defined(ENCODER_HW_EMULATED)
    p_encoder->AngleCounter.CounterD = 0;
#endif
}

/*
    Call after having moved in the positive direction
*/
void Encoder_CalibrateQuadraturePositive(Encoder_State_T * p_encoder)
{
#if     defined(ENCODER_HW_DECODER)
    uint32_t counterValue = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    // #ifdef ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
    p_encoder->Config.IsALeadBPositive = (counterValue > p_encoder->AngleCounter.CounterD);
    // #elif defined(ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
    // p_encoder->Config.IsALeadBPositive = !(counterValue > p_encoder->AngleCounter.CounterD);
    // #endif
#elif   defined(ENCODER_HW_EMULATED)
    p_encoder->Config.IsALeadBPositive = (p_encoder->AngleCounter.CounterD > 0);
#endif
}

void Encoder_CalibrateQuadratureDirection(Encoder_State_T * p_encoder, bool isPositive)
{
    p_encoder->Config.IsALeadBPositive = ((Encoder_GetCounterD(p_encoder) > 0) == isPositive);
}

/******************************************************************************/
/*!
    Config Units
*/
/******************************************************************************/

void Encoder_SetCountsPerRevolution(Encoder_State_T * p_encoder, uint16_t countsPerRevolution)
{
    p_encoder->Config.CountsPerRevolution = countsPerRevolution;
    // _Encoder_ResetUnits(p_encoder);
}

void Encoder_SetScalarSpeedRef(Encoder_State_T * p_encoder, uint16_t speedRef)
{
    p_encoder->Config.ScalarSpeedRef_Rpm = speedRef;
    // Angle_SetSpeedRef_Rpm(&p_encoder->Base, speedRef);
}

/*
    gearRatio as Surface/Encoder
*/
// void Encoder_SetSurfaceRatio(Encoder_State_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatioSurface, uint32_t gearRatioDrive)
// {
//     p_encoder->Config.SurfaceDiameter = surfaceDiameter;
//     p_encoder->Config.GearRatioOutput = gearRatioSurface;
//     p_encoder->Config.GearRatioInput = gearRatioDrive;
//     // _Encoder_ResetUnitsLinearSpeed(p_encoder);
// }

// void Encoder_SetGroundRatio_US(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelRatio, uint32_t motorRatio)
// {
//     Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Inch10 * 254 / 100, wheelRatio, motorRatio);
// }

// void Encoder_SetGroundRatio_Metric(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelRatio, uint32_t motorRatio)
// {
//     Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Mm, wheelRatio, motorRatio);
// }

/*

*/
// void _Encoder_ResetUnits(const Encoder_T * p_encoder)
// {
//     // AngleCounterConfig_T angleCounterConfig = {
//     //     .CountsPerRevolution = p_encoder->Config.CountsPerRevolution,
//     //     .TimerFreq = p_encoder->
//     //     .SampleFreq = p_encoder->
//     //     .PollingFreq = p_encoder->
//     //     .FractSpeedRef_Rpm = p_encoder->Config.ScalarSpeedRef_Rpm
//     // };

//     // AngleCounter_InitFrom(&p_encoder->AngleCounter, &angleCounterConfig);

//     // p_encoder->DirectionComp = _Encoder_GetDirectionComp(p_encoder->P_STATE);
// //     _Encoder_ResetUnitsAngle(p_encoder);
// //     _Encoder_ResetUnitsPollingAngle(p_encoder);
// //     _Encoder_ResetUnitsScalarSpeed(p_encoder);
// //     _Encoder_ResetUnitsAngularSpeed(p_encoder);
// //     _Encoder_ResetUnitsLinearSpeed(p_encoder);
// }

/******************************************************************************/
/*!

*/
/******************************************************************************/
int32_t _Encoder_ConfigId_Get(const Encoder_Config_T * p_encoder, Encoder_ConfigId_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case ENCODER_CONFIG_COUNTS_PER_REVOLUTION:             value = p_encoder->CountsPerRevolution;            break;
        case ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_encoder->IsQuadratureCaptureEnabled;     break;
        case ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE:              value = p_encoder->IsALeadBPositive;               break;
        case ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP:       value = p_encoder->ExtendedDeltaTStop;             break;
        case ENCODER_CONFIG_INTERPOLATE_ANGLE_SCALAR:          value = 0;    break;
        case ENCODER_CONFIG_INDEX_ZERO_REF:                    value = Encoder_GetIndexZeroRef(p_encoder);             break;
        // case ENCODER_CONFIG_CALIBRATE_ZERO_REF:                value = p_encoder->IndexAngleRef;                  break;
    }
    return value;
}

void _Encoder_ConfigId_Set(Encoder_Config_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue)
{
    switch (varId)
    {
        case ENCODER_CONFIG_COUNTS_PER_REVOLUTION:             p_encoder->CountsPerRevolution = varValue;            break;
        case ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED:     p_encoder->IsQuadratureCaptureEnabled = varValue;     break;
        case ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE:              p_encoder->IsALeadBPositive = varValue;               break;
        case ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP:       p_encoder->ExtendedDeltaTStop = varValue;             break;
        case ENCODER_CONFIG_INTERPOLATE_ANGLE_SCALAR:          break;

        case ENCODER_CONFIG_INDEX_ZERO_REF:                    Encoder_SetIndexZeroRef(p_encoder, varValue);              break;
        // case ENCODER_CONFIG_CALIBRATE_ZERO_REF:                Motor_Encoder_CalibrateHomeOffset(p_motor);                  break;
    }
}

/*
    Set with propagate
*/
void Encoder_ConfigId_Set(const Encoder_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue)
{
    _Encoder_ConfigId_Set(&p_encoder->P_STATE->Config, varId, varValue);
    // _Encoder_ResetUnits(p_encoder);
}