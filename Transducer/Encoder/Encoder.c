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
    @version V0
*/
/******************************************************************************/
#include "Encoder.h"


/******************************************************************************/
/*!
    Init
*/
/******************************************************************************/
void Encoder_InitInterrupts_Quadrature(Encoder_T * p_encoder)
{
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID);
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID);
    HAL_Encoder_InitPinInterruptRisingEdge(p_encoder->CONST.P_HAL_PIN_Z, p_encoder->CONST.PIN_Z_ID);
}

void Encoder_InitInterrupts_ABC(Encoder_T * p_encoder)
{
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID);
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->CONST.P_HAL_PIN_B, p_encoder->CONST.PIN_B_ID);
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->CONST.P_HAL_PIN_Z, p_encoder->CONST.PIN_Z_ID);
}

void Encoder_InitInterrupts_Incremental(Encoder_T * p_encoder)
{
    HAL_Encoder_InitPinInterruptDualEdge(p_encoder->CONST.P_HAL_PIN_A, p_encoder->CONST.PIN_A_ID);
}


/******************************************************************************/
/*!
    Units
*/
/******************************************************************************/
/*
    UnitAngle[DEGREES/EncoderCount] = DegreesPerRevolution[DEGREES/1]/CountsPerRevolution[EncoderCounts/1]
    UnitAngle => Counts * [(DEGREES << SHIFT) / CountsPerRevolution] >> SHIFT
*/
void _Encoder_ResetUnitsAngle(Encoder_T * p_encoder)
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
static uint32_t MaxDeltaD(Encoder_T * p_encoder)
    { return  p_encoder->Config.ScalarSpeedRef_Rpm * 2U * p_encoder->Config.CountsPerRevolution / (60U * p_encoder->UnitTime_Freq); }

/*
    ScalarSpeed[Percent16] = Speed_Rpm * 65536 / ScalarSpeedRef_Rpm
    UnitScalarSpeed => DeltaD * [UnitTime_Freq * 65536 * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)] / DeltaT
    UnitScalarSpeed => [DeltaD / DeltaT] * [UnitTime_Freq] * [65536 * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)]
        671,088

    e.g.
    UnitScalarSpeedShift = 1:
    UnitTime_Freq = 625000, CountsPerRevolution = 60,
        ScalarSpeedRef_Rpm = 2500 => 16,384,000
        ScalarSpeedRef_Rpm = 10000 => 4,095,937.5
    UnitTime_Freq = 1000, CountsPerRevolution = 8192,
        ScalarSpeedRef_Rpm = 5000 => 96


    ModeD/DT
    FreqD = Speed_Rpm * CountsPerRevolution / 60
    [MaxD * 2] = [2 * ScalarSpeedRef_Rpm] * CountsPerRevolution / 60 / UnitTime_Freq
    SpeedMax = 131,072 = [MaxD * 2] * [UnitScalarSpeed_Factor / UnitScalarSpeed_Divisor]

    e.g.
    CountsPerRevolution = 24, ScalarSpeedRef_Rpm = 4000 =>
       671,088 <=> 40 << Shift
*/
uint32_t Encoder_GetScalarSpeedFactor(Encoder_T * p_encoder) { return (uint32_t)65536U * 60U; }
uint32_t Encoder_GetScalarSpeedDivisor(Encoder_T * p_encoder) { return p_encoder->Config.CountsPerRevolution * p_encoder->Config.ScalarSpeedRef_Rpm; }

void _Encoder_ResetUnitsScalarSpeed(Encoder_T * p_encoder)
{
    uint32_t unitsFactor = Encoder_GetScalarSpeedFactor(p_encoder);
    uint32_t unitsDivisor = Encoder_GetScalarSpeedDivisor(p_encoder);
    p_encoder->UnitScalarSpeedShift = 14U;
    p_encoder->UnitScalarSpeed = ((uint64_t)unitsFactor << p_encoder->UnitScalarSpeedShift) * p_encoder->UnitTime_Freq / unitsDivisor;
}

/*
    As Factor,Shift of Rotational Speed
        => RPM * DEGREES / 60 ~= RPM * 10000

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
void _Encoder_ResetUnitsAngularSpeed(Encoder_T * p_encoder)
{
    p_encoder->UnitAngularSpeed = math_muldiv64_unsigned(ENCODER_ANGLE_DEGREES, p_encoder->UnitTime_Freq, p_encoder->Config.CountsPerRevolution);
    // p_encoder->UnitAngularSpeedShift = 0U;
}

/*
    AngularSpeed / POLLING_FREQ
    (AngleIndex / POLLING_FREQ) * (Count/Time) * (DEGREES / CountsPerRevolution)

    DeltaT => (UnitTime_Freq / DeltaT)
        AngleIndex * [DEGREES * TIMER_FREQ / CountsPerRevolution / POLLING_FREQ] / DeltaT

    ModeDT => FreqD
        AngleIndex * [FreqD * [DEGREES / CountsPerRevolution]] / POLLING_FREQ
            <=> as UnitAngularSpeed

        AngleIndex * [FreqD * [(DEGREES << SHIFT) / CountsPerRevolution / POLLING_FREQ]] >> SHIFT
            <=> [FreqD * UnitAngleD / POLLING_FREQ]


    DEGREES * InterpolateAngleScalar * TIMER_FREQ / (POLLING_FREQ * CountsPerRevolution)
    Scalar for electrical speed
*/
void _Encoder_ResetUnitsInterpolateAngle(Encoder_T * p_encoder)
{
    /* always TIMER_FREQ for DeltaT for now */
    p_encoder->UnitInterpolateAngle = math_muldiv64_unsigned(ENCODER_ANGLE_DEGREES * p_encoder->Config.InterpolateAngleScalar, p_encoder->CONST.TIMER_FREQ, p_encoder->CONST.POLLING_FREQ * p_encoder->Config.CountsPerRevolution);
    p_encoder->InterpolateAngleLimit = ENCODER_ANGLE_DEGREES * p_encoder->Config.InterpolateAngleScalar / p_encoder->Config.CountsPerRevolution;

    /* using ANGLE_SHIFT */
    // p_encoder->UnitInterpolateAngle = p_encoder->UnitAngleD * p_encoder->Config.InterpolateAngleScalar / p_encoder->CONST.POLLING_FREQ;
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
void _Encoder_ResetUnitsLinearSpeed(Encoder_T * p_encoder)
{
    // uint32_t unitsFactor = (p_encoder->Config.GearRatioInput * p_encoder->Config.SurfaceDiameter * 314);
    // uint32_t unitsDivisor = (p_encoder->Config.GearRatioOutput * 100);
    // p_encoder->UnitSurfaceSpeedShift = fixed_lshift_max_signed(math_muldiv64_unsigned(unitsFactor, p_encoder->Config.ScalarSpeedRef_Rpm * 2U, 60U * unitsDivisor));
    // p_encoder->UnitSurfaceSpeed = math_muldiv64_unsigned(unitsFactor, p_encoder->UnitTime_Freq << p_encoder->UnitSurfaceSpeedShift, unitsDivisor * p_encoder->Config.CountsPerRevolution);
}

void _Encoder_ResetUnits(Encoder_T * p_encoder)
{
    _Encoder_ResetUnitsAngle(p_encoder);
    _Encoder_ResetUnitsInterpolateAngle(p_encoder);
    _Encoder_ResetUnitsScalarSpeed(p_encoder);
    _Encoder_ResetUnitsAngularSpeed(p_encoder);
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

void Encoder_SetCountsPerRevolution(Encoder_T * p_encoder, uint16_t countsPerRevolution)
{
    p_encoder->Config.CountsPerRevolution = countsPerRevolution;
    _Encoder_ResetUnits(p_encoder);
}

void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef)
{
    p_encoder->Config.ScalarSpeedRef_Rpm = speedRef;
    _Encoder_ResetUnitsScalarSpeed(p_encoder);
    _Encoder_ResetUnitsAngularSpeed(p_encoder);
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

/*
    gearRatio as Surface/Encoder
*/
void Encoder_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatioSurface, uint32_t gearRatioDrive)
{
    p_encoder->Config.SurfaceDiameter = surfaceDiameter;
    p_encoder->Config.GearRatioOutput = gearRatioSurface;
    p_encoder->Config.GearRatioInput = gearRatioDrive;
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

void Encoder_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelRatio, uint32_t motorRatio)
{
    Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Inch10 * 254 / 100, wheelRatio, motorRatio);
}

void Encoder_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelRatio, uint32_t motorRatio)
{
    Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Mm, wheelRatio, motorRatio);
}

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled)   { p_encoder->Config.IsQuadratureCaptureEnabled = isEnabled; }
void Encoder_EnableQuadratureMode(Encoder_T * p_encoder)                { p_encoder->Config.IsQuadratureCaptureEnabled = true; }
void Encoder_DisableQuadratureMode(Encoder_T * p_encoder)               { p_encoder->Config.IsQuadratureCaptureEnabled = false; }
/*! isALeadBPositive - User runtime calibrate */
void Encoder_SetQuadratureDirection(Encoder_T * p_encoder, bool isALeadBPositive) { p_encoder->Config.IsALeadBPositive = isALeadBPositive; }


void Encoder_CaptureQuadratureReference(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    p_encoder->CounterD = HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER, 0);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->CounterD = 0;
#endif
}

/*
    Call after having moved in the positive direction
*/
void Encoder_CaptureQuadraturePositive(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    uint32_t counterValue = HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    // #ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
    // p_encoder->Config.IsALeadBPositive = (counterValue > p_encoder->CounterD);
    // #elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
    // p_encoder->Config.IsALeadBPositive = !(counterValue > p_encoder->CounterD);
    // #endif
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->Config.IsALeadBPositive = (p_encoder->CounterD > 0);
#endif
}


/******************************************************************************/
/*!
    Set reference by homing or aligning
*/
/******************************************************************************/
void Encoder_StartHoming(Encoder_T * p_encoder)
{
    p_encoder->IndexCount = 0U;
    p_encoder->Angle32 = 0U;
    p_encoder->CounterD = 0U;
    p_encoder->CounterPrev = 0U;
}

bool Encoder_ProcHoming(Encoder_T * p_encoder)
{
    bool isIndex = p_encoder->IndexCount > 0U;
    // bool isIndex = p_encoder->Angle32 == p_encoder->Config.IndexAngleRef;

    if (isIndex == true)
    {
        p_encoder->IndexCount = 0U;
        // p_encoder->Angle32 = p_encoder->Config.IndexAngleRef;
    }

    /* caller handle */
    // assert(p_encoder->CounterD < p_encoder->Config.CountsPerRevolution);
    return isIndex;
}

bool Encoder_ProcHomingVirtualIndex(Encoder_T * p_encoder)
{
    // bool isVirtualIndex = p_encoder->Angle32 < p_encoder->Config.IndexAngleRef;
    // bool isVirtualIndex = p_encoder->Angle32 > p_encoder->Config.IndexAngleRef;
}

void Encoder_CaptureAlignZero(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;
    // p_encoder->Angle32 = 0U + p_encoder->Config.IndexAngleRef;
    _Encoder_ZeroPulseCount(p_encoder);
}



// void Encoder_CalibrateZeroReference(Encoder_T * p_encoder)

/******************************************************************************/
void Encoder_CalibrateAlignZero(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;
    _Encoder_ZeroPulseCount(p_encoder);
}

/* todo */
void Encoder_CalibrateAlignValidate(Encoder_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_PHASE;
    _Encoder_ZeroPulseCount(p_encoder);
}

void Encoder_CalibrateIndexStart(Encoder_T * p_encoder)
{
    // _Encoder_ZeroPulseCount(p_encoder);
}



void Encoder_ClearAlign(Encoder_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_NO;
}


/******************************************************************************/
/*!
    Calibration for Reference
    Determine the values initially
*/
/******************************************************************************/
/*
    Run on calibration routine start
*/
// void Encoder_CalibrateZeroReference(Encoder_T * p_encoder)
void Encoder_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    p_encoder->CounterD = HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER, 0);
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->CounterD = 0;
#endif
}

/*
    Call after having moved in the positive direction
*/
void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    uint32_t counterValue = HAL_Encoder_ReadCounter(p_encoder->CONST.P_HAL_ENCODER_COUNTER);
    // #ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
    // p_encoder->Config.IsALeadBPositive = (counterValue > p_encoder->CounterD);
    // #elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
    // p_encoder->Config.IsALeadBPositive = !(counterValue > p_encoder->CounterD);
    // #endif
#elif   defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->Config.IsALeadBPositive = (p_encoder->CounterD > 0);
#endif
}

void Encoder_SetVirtualZero(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;


}

void Encoder_CalibrateVirtualZero(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;
}


void Encoder_StartCalibrateIndex(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;
}

void Encoder_CalibrateIndex(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;
// p_encoder->Angle32 = p_encoder->Config.IndexAngleRef
}

// void Encoder_CalibrateIndex(Encoder_T * p_encoder)
// {
//     p_encoder->Align = ENCODER_ALIGN_ABSOLUTE;
//     // p_encoder->AbsoluteOffset = p_encoder->IndexOffset;
// }

