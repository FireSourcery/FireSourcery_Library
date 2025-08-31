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

/* as Hall  */
void Encoder_InitInterrupts_ABC(const Encoder_T * p_encoder)
{
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->P_HAL_PIN_A, p_encoder->PIN_A_ID);
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->P_HAL_PIN_B, p_encoder->PIN_B_ID);
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID);
}

// static inline void Encoder_Quadrature_InitDirection(Encoder_State_T * p_encoder) { p_encoder->DirectionComp = (p_encoder->Config.IsALeadBPositive == true) ? 1 : -1; }

/******************************************************************************/
/*!
    Config Units
    todo split module
    Units
*/
/******************************************************************************/

/* One point of access to reduce flash use */
// uint32_t math_muldiv64_unsigned(uint32_t value, uint32_t factor, uint32_t divisor) { return (uint64_t)value * factor / divisor; }


/*
    UnitAngle[DEGREES/EncoderCount] = [DEGREES]/CountsPerRevolution[EncoderCounts/1]

    Angle = Counts * [(DEGREES << SHIFT) / CountsPerRevolution] >> SHIFT
*/
void _Encoder_ResetUnitsAngle(Encoder_State_T * p_encoder)
{
    p_encoder->UnitAngleD = UINT32_MAX / p_encoder->Config.CountsPerRevolution + 1U;
}

/*
    Speed
*/
/*
    determine ModeT or ModeD/DT Mode Units
    Max = 2 * ScalarSpeedRef_Rpm * CountsPerRevolution / (60 * UnitTime_Freq)
*/
static uint32_t MaxDeltaD(Encoder_State_T * p_encoder)
    { return p_encoder->Config.ScalarSpeedRef_Rpm * 2U * p_encoder->Config.CountsPerRevolution / (60U * p_encoder->UnitTime_Freq); }

/*
    ScalarSpeed[Fract16] = Speed_Rpm * FRACT16_MAX / ScalarSpeedRef_Rpm
    UnitScalarSpeed => DeltaD * [UnitTime_Freq * FRACT16_MAX * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)] / DeltaT
    UnitScalarSpeed => [DeltaD / DeltaT] * [UnitTime_Freq] * [FRACT16_MAX * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)]

    e.g.
    UnitScalarSpeedShift = 1:
    UnitTime_Freq = 625000, CountsPerRevolution = 60,
        ScalarSpeedRef_Rpm = 2500 => 16,384,000
        ScalarSpeedRef_Rpm = 10000 => 4,095,937.5
    UnitTime_Freq = 1000, CountsPerRevolution = 8192,
        ScalarSpeedRef_Rpm = 5000 => 96


    ModeD/DT
    FreqD = Speed_Rpm * CountsPerRevolution / 60

    e.g.
    CountsPerRevolution = 24, ScalarSpeedRef_Rpm = 4000 =>
       671,088 <=> 40 << Shift
*/
void _Encoder_ResetUnitsScalarSpeed(Encoder_State_T * p_encoder)
{
    uint32_t unitsFactor = (uint32_t)32768U * 60U;
    uint32_t unitsDivisor = (uint32_t)p_encoder->Config.CountsPerRevolution * p_encoder->Config.ScalarSpeedRef_Rpm;
    p_encoder->UnitScalarSpeedShift = 14U; //todo as shift max of MaxDeltaD
    p_encoder->UnitScalarSpeed = ((uint64_t)unitsFactor << p_encoder->UnitScalarSpeedShift) * p_encoder->UnitTime_Freq / unitsDivisor;
}

/*
    /Second
    In range for electrical speed only
    Angle16/S same as RPS normalized to [0:65536]

    UnitAngularSpeed => DeltaD * [DEGREES * UnitTime_Freq / CountsPerRevolution] / DeltaT
        <=> [UnitAngleD * UnitTime_Freq >> SHIFT]

    AngularSpeed[DEGREES/s]     = DegreesPerRevolution[DEGREES/1] / CountsPerRevolution[EncoderCounts/1] * DeltaD[EncoderCounts] * UnitTime_Freq[Hz] / DeltaT[TimerTicks]
        DeltaAngle[DEGREES]     = DegreesPerRevolution[DEGREES/1] / CountsPerRevolution[EncoderCounts/1] * DeltaD[EncoderCounts]
    RotationalSpeed[N/s]        = 1 / CountsPerRevolution[EncoderCounts/1] * DeltaD[EncoderCounts] * UnitTime_Freq[Hz] / DeltaT[TimerTicks]
        Revolutions[N]          = 1 / CountsPerRevolution[EncoderCounts/1] * DeltaD[EncoderCounts]

    e.g. DEGREES_BITS = 16,
        UnitAngularSpeed = 160,000          : UnitTime_Freq = 20000, CountsPerRevolution = 8192
        UnitAngularSpeed = 131,072          : UnitTime_Freq = 20000, CountsPerRevolution = 10000
        UnitAngularSpeed = 8,000            : UnitTime_Freq = 1000, CountsPerRevolution = 8192
        UnitAngularSpeed = 819,200,000      : UnitTime_Freq = 750000, CountsPerRevolution = 60

*/
void _Encoder_ResetUnitsAngularSpeed(Encoder_State_T * p_encoder)
{
    // p_encoder->UnitAngularSpeed = math_muldiv64_unsigned(ENCODER_ANGLE_DEGREES, p_encoder->UnitTime_Freq, p_encoder->Config.CountsPerRevolution);
    p_encoder->UnitAngularSpeed = (uint64_t)ENCODER_ANGLE_DEGREES * p_encoder->UnitTime_Freq / p_encoder->Config.CountsPerRevolution;
    // p_encoder->UnitAngularSpeedShift = 0U;
}

/*
    UnitPollingAngle == [DEGREES/EncoderCount/POLLING_FREQ] = [(DEGREES / CountsPerRevolution) / POLLING_FREQ]
    PollingAngle = (Count/Time) * [(DEGREES / CountsPerRevolution) / POLLING_FREQ]

    AngularSpeed / POLLING_FREQ
        (1 / POLLING_FREQ) * (Count/Time) * (DEGREES / CountsPerRevolution)

    DeltaT => (UnitTime_Freq / DeltaT)
        AngleIndex * [DEGREES * TIMER_FREQ / CountsPerRevolution / POLLING_FREQ] / DeltaT

    ModeDT => FreqD
            [FreqD * [DEGREES / CountsPerRevolution]] / POLLING_FREQ
            [FreqD * [(DEGREES << SHIFT) / CountsPerRevolution / POLLING_FREQ]] >> SHIFT
            <=> [FreqD * UnitAngleD / POLLING_FREQ]
            <=> as UnitAngularSpeed / POLLING_FREQ
*/
void _Encoder_ResetUnitsPollingAngle(Encoder_State_T * p_encoder)
{
    // if (p_encoder->UnitTime_Freq == p_const->TIMER_FREQ) //todo as get mode
    // {
    //     /* altneratively TIMER_FREQ / POLLING_FREQ << Shift */
    //     p_encoder->UnitPollingAngle = math_muldiv64_unsigned(ENCODER_ANGLE_DEGREES * p_encoder->Config.PartitionsPerRevolution, p_const->TIMER_FREQ, p_const->POLLING_FREQ * p_encoder->Config.CountsPerRevolution);
    //     p_encoder->InterpolateAngleLimit = ENCODER_ANGLE_DEGREES * p_encoder->Config.PartitionsPerRevolution / p_encoder->Config.CountsPerRevolution;
    // }
    // else if (p_encoder->UnitTime_Freq == 1U)
    // {
        /* using ANGLE_SHIFT */
        p_encoder->UnitPollingAngle = p_encoder->UnitAngleD * p_encoder->Config.PartitionsPerRevolution / p_encoder->PollingFreq;
        p_encoder->InterpolateAngleLimit = p_encoder->UnitAngleD * p_encoder->Config.PartitionsPerRevolution;
    // }
}

/*
    Surface/Encoder Ratio <=> gearRatioOutput/gearRatioInput

    SurfaceSpeed_DistancePerHour = (RPM * gearRatioInput/gearRatioOutput) * surfaceDiameter_Units * 314 / 100 * 60 / DistanceConversion
    RPM = 60 *(deltaD_Ticks * UnitTime_Freq) / (CountsPerRevolution * deltaT_Ticks)

    SurfaceSpeed_UnitsPerSecond = (deltaD_Ticks * UnitTime_Freq * gearRatioInput * surfaceDiameter_Units * 314) / (CountsPerRevolution * deltaT_Ticks * gearRatioOutput * 100))
    SurfaceSpeed_UnitsPerSecond = deltaD_Ticks * [[UnitTime_Freq * gearRatioInput * surfaceDiameter_Units * 314] / [CountsPerRevolution * gearRatioOutput * 100]] / deltaT_Ticks

    SurfaceSpeed_DistancePerHour = ((60 * deltaD_Ticks * UnitTime_Freq * gearRatioInput * surfaceDiameter_Units * 314 * 60) / (CountsPerRevolution * deltaT_Ticks * gearRatioOutput * 100 * DistanceConversion))

    Units => mm
    DistanceConversion:
    mph => 1,609,344
    kmh => 1,000,000

    e.g.
        UnitTime_Freq = 1000, GearRatio = 8, SurfaceDiameter = 20in, 508mm, CountsPerRevolution = 8192
        UnitSurfaceSpeed = 24.3395996094
        1000 RPM => 7.44 mph, 11.97352 kmh
            => DeltaD = 136.533333333
        mph = 136.533333333 * 24.3395996094 * 60 * 60 / 1609344 = 7.4337121212
        mph = 136 * 24 * 60 * 60 / 1609344 = 7.3
        kmh = 136 * 24 * 60 * 60 / 1000000 = 11.7504

    DistancePerRevolution = [gearRatioInput * surfaceDiameter_Mm * 314 / gearRatioOutput * 100]
*/
void _Encoder_ResetUnitsLinearSpeed(Encoder_State_T * p_encoder)
{
    // uint32_t unitsFactor = (p_encoder->Config.GearRatioInput * p_encoder->Config.SurfaceDiameter * 314);
    // uint32_t unitsDivisor = (p_encoder->Config.GearRatioOutput * 100);
    // p_encoder->UnitSurfaceSpeedShift = fixed_lshift_max_signed(math_muldiv64_unsigned(unitsFactor, p_encoder->Config.ScalarSpeedRef_Rpm * 2U, 60U * unitsDivisor));
    // p_encoder->UnitSurfaceSpeed = math_muldiv64_unsigned(unitsFactor, p_encoder->UnitTime_Freq << p_encoder->UnitSurfaceSpeedShift, unitsDivisor * p_encoder->Config.CountsPerRevolution);
}



/*

*/
// void _Encoder_ResetUnits(const Encoder_T * p_encoder) using const def
void _Encoder_ResetUnits(Encoder_State_T * p_encoder)
{
    // p_encoder->DirectionComp = _Encoder_GetDirectionComp(p_encoder->P_STATE);
    _Encoder_ResetUnitsAngle(p_encoder);
    _Encoder_ResetUnitsPollingAngle(p_encoder);
    _Encoder_ResetUnitsScalarSpeed(p_encoder);
    _Encoder_ResetUnitsAngularSpeed(p_encoder);
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

void Encoder_SetCountsPerRevolution(Encoder_State_T * p_encoder, uint16_t countsPerRevolution)
{
    p_encoder->Config.CountsPerRevolution = countsPerRevolution;
    _Encoder_ResetUnits(p_encoder);
}

void Encoder_SetPartitionsPerRevolution(Encoder_State_T * p_encoder, uint16_t count)
{
    p_encoder->Config.PartitionsPerRevolution = count;
    _Encoder_ResetUnitsPollingAngle(p_encoder);
}

void Encoder_SetScalarSpeedRef(Encoder_State_T * p_encoder, uint16_t speedRef)
{
    p_encoder->Config.ScalarSpeedRef_Rpm = speedRef;
    _Encoder_ResetUnitsScalarSpeed(p_encoder);
    // if dependent on MaxDeltaD
    // _Encoder_ResetUnitsAngularSpeed(p_encoder);
    // _Encoder_ResetUnitsLinearSpeed(p_encoder);
}


/*
    gearRatio as Surface/Encoder
*/
void Encoder_SetSurfaceRatio(Encoder_State_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatioSurface, uint32_t gearRatioDrive)
{
    p_encoder->Config.SurfaceDiameter = surfaceDiameter;
    p_encoder->Config.GearRatioOutput = gearRatioSurface;
    p_encoder->Config.GearRatioInput = gearRatioDrive;
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

void Encoder_SetGroundRatio_US(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelRatio, uint32_t motorRatio)
{
    Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Inch10 * 254 / 100, wheelRatio, motorRatio);
}

void Encoder_SetGroundRatio_Metric(Encoder_State_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelRatio, uint32_t motorRatio)
{
    Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Mm, wheelRatio, motorRatio);
}


//   void Encoder_SetUnitsHall_ElSpeed(Encoder_State_T * p_encoder, uint32_t speedRef_Rpm)
// {
//     p_encoder->Config.IsQuadratureCaptureEnabled = false;
//     if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_Rpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_Rpm); }
//     if (p_encoder->Config.CountsPerRevolution != 6U) { Encoder_SetCountsPerRevolution(p_encoder, 6U); }
//     if (p_encoder->Config.PartitionsPerRevolution != 1U) { Encoder_SetPartitionsPerRevolution(p_encoder, 1U); }
// }

//   void Encoder_SetUnitsHall_MechSpeed(Encoder_State_T * p_encoder, uint16_t speedRef_Rpm, uint8_t polePairs)
// {
//     p_encoder->Config.IsQuadratureCaptureEnabled = false;
//     if (p_encoder->Config.ScalarSpeedRef_Rpm != speedRef_Rpm) { Encoder_SetScalarSpeedRef(p_encoder, speedRef_Rpm); }
//     if (p_encoder->Config.CountsPerRevolution != polePairs * 6U) { Encoder_SetCountsPerRevolution(p_encoder, polePairs * 6U); }
//     if (p_encoder->Config.PartitionsPerRevolution != polePairs) { Encoder_SetPartitionsPerRevolution(p_encoder, polePairs); }     /* Set for electrical cycle */
// }


/******************************************************************************/
/*!
    Set reference by homing or aligning
*/
/******************************************************************************/
void Encoder_StartHoming(Encoder_State_T * p_encoder)
{
    p_encoder->IndexCount = 0U;
    p_encoder->CounterD = 0U;
    // p_encoder->IndexAngleRef = 0U;
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
    /* DirectionComp */ p_encoder->CounterD > p_encoder->Config.CountsPerRevolution;
}

//enum Encoder_HomingStatus { Encoder_HomingStatus_None, Encoder_HomingStatus_Found, Encoder_HomingStatus_Error };
bool Encoder_PollHomingComplete(Encoder_State_T * p_encoder)
{
    bool isComplete = false;

    if (Encoder_IsHomingIndexFound(p_encoder) == true)
    {
        // p_encoder->IndexCount = 0U;
        // p_encoder->CounterD = 0U;
        p_encoder->IsHomed = true;
        isComplete = true;
    }
    else if (Encoder_IsHomingIndexError(p_encoder) == true)
    {
        // p_encoder->IndexCount = 0U;
        // p_encoder->CounterD = 0U;
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

void Encoder_CalibrateIndexZeroRef(Encoder_State_T * p_encoder)
{
    if (p_encoder->IsHomed == true)
    {
        // Encoder_SetIndexZeroRef(p_encoder, p_encoder->Angle32);

        // p_encoder->Config.IndexAngleRef = p_encoder->Angle32 - p_encoder->IndexAngleRef;
        // p_encoder->IndexAngleRef = p_encoder->Config.IndexAngleRef;
        p_encoder->Config.IndexAngleRef = p_encoder->Angle32 - p_encoder->Config.IndexAngleRef;
        p_encoder->Angle32 = 0U;
    }
}




/******************************************************************************/
/*!
    Align to phase without homing
*/
/******************************************************************************/
void Encoder_CheckAlignRef(Encoder_State_T * p_encoder)
{
    // if (Encoder_IsPositionRefSet(p_encoder) == false)
    // {
    // }
    //     p_encoder->AlignOffsetRef = p_encoder->Config.AlignOffsetRef;
}

//
// void Encoder_CalibrateAlignRef(Encoder_State_T * p_encoder)
// {
//     // p_encoder->Angle32 = 0U;
//     p_encoder->AlignOffsetRef = p_encoder->Angle32 /* - p_encoder->Config.IndexAngleRef */;
//     p_encoder->Config.AlignOffsetRef = p_encoder->AlignOffsetRef;
//     _Encoder_ZeroPulseCount(p_encoder);
// }

void Encoder_CaptureAlignZero(Encoder_State_T * p_encoder)
{
    p_encoder->AlignOffsetRef = p_encoder->Angle32;
    p_encoder->AlignOffsetRef = p_encoder->Angle32 /* - p_encoder->Config.IndexAngleRef */;
    if (p_encoder->IsHomed == true)
    {
        p_encoder->Config.AlignOffsetRef = p_encoder->AlignOffsetRef;
        _Encoder_ZeroPulseCount(p_encoder);
    }
}

/* this way angle starts from a known pole */
uint16_t Encoder_GetAngleAligned(Encoder_State_T * p_encoder)
{
    return (p_encoder->Angle32 - p_encoder->AlignOffsetRef) >> ENCODER_ANGLE_SHIFT;
}

// uint16_t Encoder_GetElectricalAngleOffset(Encoder_State_T * p_encoder)
// {
//     return % p_encoder->Config.PartitionsPerRevolution;
// }

bool Encoder_ProcAlignValidate(Encoder_State_T * p_encoder)
{

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
#if     defined(CONFIG_ENCODER_HW_DECODER)
    p_encoder->CounterD = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->P_HAL_ENCODER_COUNTER, 0);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->CounterD = 0;
#endif
}

/*
    Call after having moved in the positive direction
*/
void Encoder_CalibrateQuadraturePositive(Encoder_State_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    uint32_t counterValue = HAL_Encoder_ReadCounter(p_encoder->P_HAL_ENCODER_COUNTER);
    // #ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
    p_encoder->Config.IsALeadBPositive = (counterValue > p_encoder->CounterD);
    // #elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
    // p_encoder->Config.IsALeadBPositive = !(counterValue > p_encoder->CounterD);
    // #endif
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->Config.IsALeadBPositive = (p_encoder->CounterD > 0);
#endif
}

void Encoder_CalibrateQuadratureDirection(Encoder_State_T * p_encoder, bool isPositive)
{
    p_encoder->Config.IsALeadBPositive = ((Encoder_GetCounterD(p_encoder) > 0) == isPositive);
}


/******************************************************************************/
/*!

*/
/******************************************************************************/



// //
// uint16_t CountsPerRevolution;       /* Derive Angular Units. */
// uint16_t ScalarSpeedRef_Rpm;        /* Derive Fract16 Units. */
// uint16_t SurfaceDiameter;           /* Derive Linear Units. */
// uint16_t GearRatioInput;            /* DistancePerRevolution_Divider */
// uint16_t GearRatioOutput;           /* DistancePerRevolution_Factor */
// uint16_t ExtendedDeltaTStop;        /* ExtendedTimer time read as deltaT stopped, default as 1s */
// uint32_t PartitionsPerRevolution;    /* Sets UnitPollingAngle Scalar and InterpolateAngleLimit. e.g electrical angle conversion */

// uint32_t IndexAngleRef;             /* Virtual Index - Index, VirtualIndexOffset */
// uint32_t AlignOffsetRef;            /* Align - Index */

// #if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
// bool IsQuadratureCaptureEnabled;    /* Quadrature Mode - enable hardware/emulated quadrature speed capture */
// bool IsALeadBPositive;              /* User runtime calibration for encoder install direction. Accounts for LUT calibration */
// /* Optionally combine with compile time defined QUADRATURE_A_LEAD_B_INCREMENT */
//
int32_t _Encoder_ConfigId_Get(const Encoder_State_T * p_encoder, Encoder_ConfigId_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case ENCODER_CONFIG_COUNTS_PER_REVOLUTION:             value = p_encoder->Config.CountsPerRevolution;            break;
        case ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_encoder->Config.IsQuadratureCaptureEnabled;     break;
        case ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE:              value = p_encoder->Config.IsALeadBPositive;               break;
        case ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP:       value = p_encoder->Config.ExtendedDeltaTStop;             break;
        case ENCODER_CONFIG_INTERPOLATE_ANGLE_SCALAR:          value = 0;    break;
        case ENCODER_CONFIG_INDEX_ZERO_REF:                    value = Encoder_GetIndexZeroRef(p_encoder);             break;
        // case ENCODER_CONFIG_CALIBRATE_ZERO_REF:                value = p_encoder->Config.IndexAngleRef;                  break;
    }
    return value;
}

void _Encoder_ConfigId_Set(Encoder_State_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue)
{
    switch (varId)
    {
        case ENCODER_CONFIG_COUNTS_PER_REVOLUTION:             p_encoder->Config.CountsPerRevolution = varValue;            break;
        case ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED:     p_encoder->Config.IsQuadratureCaptureEnabled = varValue;     break;
        case ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE:              p_encoder->Config.IsALeadBPositive = varValue;               break;
        case ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP:       p_encoder->Config.ExtendedDeltaTStop = varValue;             break;
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
    _Encoder_ConfigId_Set(p_encoder->P_STATE, varId, varValue);
    // _Encoder_ResetUnits(p_encoder);
}