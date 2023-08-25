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

/* Phase in ABAB order, ALeadB as increment */
const int8_t _ENCODER_TABLE[_ENCODER_TABLE_LENGTH] =
{
    0,-1,1,_ENCODER_TABLE_ERROR,
    1,0,_ENCODER_TABLE_ERROR,-1,
    -1,_ENCODER_TABLE_ERROR,0,1,
    _ENCODER_TABLE_ERROR,1,-1,0
};

/* Phase A Both Edge */
const int8_t _ENCODER_TABLE_PHASE_A[_ENCODER_TABLE_LENGTH] =
{
    0,_ENCODER_TABLE_ERROR,_ENCODER_TABLE_ERROR,-1,
    _ENCODER_TABLE_ERROR,0,1,_ENCODER_TABLE_ERROR,
    _ENCODER_TABLE_ERROR,1,0,_ENCODER_TABLE_ERROR,
    -1,_ENCODER_TABLE_ERROR,_ENCODER_TABLE_ERROR,0
};

/******************************************************************************/
/*!
    Init
*/
/******************************************************************************/
static void InitPhases(Encoder_T * p_encoder)
{
    HAL_Encoder_InitPhase(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
    HAL_Encoder_InitPhase(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
    HAL_Encoder_InitPhase(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
}

void Encoder_InitInterrupts_Quadrature(Encoder_T * p_encoder)
{
    InitPhases(p_encoder);
    HAL_Encoder_EnablePhaseInterruptDualEdge(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
    HAL_Encoder_EnablePhaseInterruptDualEdge(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
    HAL_Encoder_EnablePhaseInterruptFallingEdge(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
}

void Encoder_InitInterrupts_ABC(Encoder_T * p_encoder)
{
    InitPhases(p_encoder);
    HAL_Encoder_EnablePhaseInterruptDualEdge(p_encoder->CONFIG.P_HAL_ENCODER_A, p_encoder->CONFIG.PHASE_A_ID);
    HAL_Encoder_EnablePhaseInterruptDualEdge(p_encoder->CONFIG.P_HAL_ENCODER_B, p_encoder->CONFIG.PHASE_B_ID);
    HAL_Encoder_EnablePhaseInterruptDualEdge(p_encoder->CONFIG.P_HAL_ENCODER_Z, p_encoder->CONFIG.PHASE_Z_ID);
}

void Encoder_Init_Quadrature(Encoder_T * p_encoder)
{

}

void Encoder_Init_SinglePhase(Encoder_T * p_encoder)
{

}

/* Outer module sets direction */
void Encoder_SetSinglePhaseDirection(Encoder_T * p_encoder, bool isPositive) { p_encoder->IsSinglePhasePositive = isPositive; }
void Encoder_SetSinglePhaseDirectionPositive(Encoder_T * p_encoder) { p_encoder->IsSinglePhasePositive = true; }
void Encoder_SetSinglePhaseDirectionNegative(Encoder_T * p_encoder) { p_encoder->IsSinglePhasePositive = false; }

void Encoder_CalibrateAlignZero(Encoder_T * p_encoder)
{
    p_encoder->Angle32 = 0U;
    _Encoder_ZeroPulseCount(p_encoder);
}

void Encoder_CalibrateAlignValidate(Encoder_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_PHASE;
    _Encoder_ZeroPulseCount(p_encoder);
}

void Encoder_CalibrateIndexStart(Encoder_T * p_encoder)
{
    // _Encoder_ZeroPulseCount(p_encoder);
}

void Encoder_CalibrateIndex(Encoder_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_ABSOLUTE;
    // p_encoder->AbsoluteOffset = p_encoder->IndexOffset;
}

void Encoder_ClearAlign(Encoder_T * p_encoder)
{
    p_encoder->Align = ENCODER_ALIGN_NO;
}


#if defined(CONFIG_ENCODER_QUADRATURE_MODE_ENABLE)
void Encoder_SetQuadratureMode(Encoder_T * p_encoder, bool isEnabled) { p_encoder->Params.IsQuadratureCaptureEnabled = isEnabled; }
void Encoder_EnableQuadratureMode(Encoder_T * p_encoder) { p_encoder->Params.IsQuadratureCaptureEnabled = true; }
/*!    isALeadBPositive - User runtime calibrate */
void Encoder_SetQuadratureDirectionCalibration(Encoder_T * p_encoder, bool isALeadBPositive) { p_encoder->Params.IsALeadBPositive = isALeadBPositive; }

/*
    Run on calibration routine start
*/
void Encoder_CalibrateQuadratureReference(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    p_encoder->CounterD = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
    HAL_Encoder_WriteCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER, 0);
#elif     defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->CounterD = 0;
#endif
}

/*
    Call after having moved in the positive direction
*/
void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
#if     defined(CONFIG_ENCODER_HW_DECODER)
    uint32_t counterValue = HAL_Encoder_ReadCounter(p_encoder->CONFIG.P_HAL_ENCODER_COUNTER);
    #ifdef CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_INCREMENT
    p_encoder->Params.IsALeadBPositive = (counterValue > p_encoder->CounterD);
    #elif defined(CONFIG_ENCODER_HW_QUADRATURE_A_LEAD_B_DECREMENT)
    p_encoder->Params.IsALeadBPositive = !(counterValue > p_encoder->CounterD);
    #endif
#elif     defined(CONFIG_ENCODER_HW_EMULATED)
    p_encoder->Params.IsALeadBPositive = (p_encoder->CounterD > 0);
#endif
}

#endif

/******************************************************************************/
/*!
    Units
*/
/******************************************************************************/
/*
    Rotational/Angular
    User Input:
    DEGREES_BITS            Angle unit conversion. Degrees per revolution in Bits. e.g. 16 for 65535 degrees cycle
    CountsPerRevolution     Angle unit conversion. Encoder Resolution
*/
/*
    UnitAngle[DEGREES/EncoderTick] = DegreesPerRevolution[DEGREES/1]/CountsPerRevolution[EncoderCounts/1]
    UnitAngle => CounterD * [((1 << 32) / CountsPerRevolution] >> (32 - DEGREES_BITS)
*/
void _Encoder_ResetUnitsAngle(Encoder_T * p_encoder)
{
    p_encoder->UnitAngularD = UINT32_MAX / p_encoder->Params.CountsPerRevolution + 1U;
}

/*
    ENCODER_ANGLE_DEGREES * InterpolateAngleScalar * TIMER_FREQ / (POLLING_FREQ * CountsPerRevolution)
*/
void _Encoder_ResetUnitsInterpolateAngle(Encoder_T * p_encoder)
{
    p_encoder->UnitInterpolateAngle = math_muldiv64_unsigned(ENCODER_ANGLE_DEGREES * p_encoder->Params.InterpolateAngleScalar, p_encoder->CONFIG.TIMER_FREQ, p_encoder->CONFIG.POLLING_FREQ * p_encoder->Params.CountsPerRevolution);
    p_encoder->InterpolateAngleLimit = ENCODER_ANGLE_DEGREES * p_encoder->Params.InterpolateAngleScalar / p_encoder->Params.CountsPerRevolution;
}

/*
    Speed
*/
/*
    Use to determine ModeT or ModeD/DT Mode Units
    MaxDeltaD = 2 * ScalarSpeedRef_Rpm * CountsPerRevolution / (60 * UnitT_Freq)
*/
static uint32_t GetMaxDeltaD(Encoder_T * p_encoder)
{
    return (p_encoder->UnitT_Freq == p_encoder->CONFIG.TIMER_FREQ) ?
        1U : math_muldiv64_unsigned(p_encoder->Params.ScalarSpeedRef_Rpm * 2U, p_encoder->Params.CountsPerRevolution, 60U * p_encoder->UnitT_Freq);
}

static void SetUnitsSpeed(Encoder_T * p_encoder, uint32_t * p_unitsSpeed, uint8_t * p_unitsSpeedShift, uint32_t unitsFactor, uint32_t unitsDivisor)
{
    uint32_t speedDivisor = p_encoder->Params.CountsPerRevolution * unitsDivisor;

    if (p_encoder->UnitT_Freq == p_encoder->CONFIG.TIMER_FREQ)
    {
        *p_unitsSpeedShift = 0U;
        *p_unitsSpeed = math_muldiv64_unsigned(p_encoder->UnitT_Freq, unitsFactor, speedDivisor);
    }
    else
    {
        /* max = unitsFactor * ScalarSpeedRef_Rpm/60 * 2 / unitsDivisor = deltaDMax * speedFactor / speedDivisor */
        *p_unitsSpeedShift = q_maxshift_signed(math_muldiv64_unsigned(unitsFactor * 2U - 1U, p_encoder->Params.ScalarSpeedRef_Rpm, 60U * unitsDivisor));
        *p_unitsSpeed = math_muldiv64_unsigned(unitsFactor, p_encoder->UnitT_Freq << (*p_unitsSpeedShift), speedDivisor); /* UnitT_Freq < 65536, shift < 16  */
    }
}

/*
    ScalarSpeed[Frac16] = Speed * 65536 / ScalarSpeedRef
    UnitScalarSpeed => DeltaD * [UnitT_Freq * 65536 * 60 / (CountsPerRevolution * ScalarSpeedRef_Rpm)] / DeltaT

    e.g.
    UnitT_Freq = 625000, CountsPerRevolution = 60,
        ScalarSpeedRef_Rpm = 2500 => 16,384,000
        ScalarSpeedRef_Rpm = 10000 => 4,095,937.5
    UnitT_Freq = 1000, CountsPerRevolution = 8192,
        ScalarSpeedRef_Rpm = 5000 => 96

    ModeD/DT
    deltaDMax = ScalarSpeedRef_Rpm * 2 * CountsPerRevolution / (60 * UnitT_Freq)
    deltaDMax * speedFactor / speedDivisor = 65536 * 2;
    Shift == 14, UnitScalarSpeed == 131,072 / deltaDMax
*/
void _Encoder_ResetUnitsScalarSpeed(Encoder_T * p_encoder)
{
    SetUnitsSpeed(p_encoder, &p_encoder->UnitScalarSpeed, &p_encoder->UnitScalarSpeedShift, (uint32_t)65536U * 60U, p_encoder->Params.ScalarSpeedRef_Rpm);

    p_encoder->_FreqDMax = p_encoder->Params.ScalarSpeedRef_Rpm * 2 * p_encoder->Params.CountsPerRevolution / (60 * p_encoder->UnitT_Freq);
}

/*
    AngularSpeed[DEGREES/s]    = DeltaD[EncoderCounts] * DegreesPerRevolution[DEGREES/1] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderCounts/1] / DeltaT[TimerTicks]
        DeltaAngle[DEGREES]    = DeltaD[EncoderCounts] * DegreesPerRevolution[DEGREES/1] / CountsPerRevolution[EncoderCounts/1]
    RotationalSpeed[N/s]     = DeltaD[EncoderCounts] * UnitT_Freq[Hz] / CountsPerRevolution[EncoderCounts/1] / DeltaT[TimerTicks]
        Revolutions[N]        = DeltaD[EncoderCounts] / CountsPerRevolution[EncoderCounts/1]

    UnitAngularSpeed => DeltaD * [(1 << DEGREES_BITS) * UnitT_Freq / CountsPerRevolution] / DeltaT
        <=> UnitAngularD * UnitT_Freq >> (32 - DEGREES_BITS)

    e.g.
        UnitAngularSpeed = 819,200,000     { DEGREES_BITS = 16, UnitT_Freq = 750000, CountsPerRevolution = 60 }
        UnitAngularSpeed = 160,000         { DEGREES_BITS = 16, UnitT_Freq = 20000, CountsPerRevolution = 8192 }
        UnitAngularSpeed = 131,072        { DEGREES_BITS = 16, UnitT_Freq = 20000, CountsPerRevolution = 10000 }
        UnitAngularSpeed = 8,000         { DEGREES_BITS = 16, UnitT_Freq = 1000, CountsPerRevolution = 8192 }

    ModeD
    deltaDMax * speedFactor / speedDivisor = ENCODER_ANGLE_DEGREES * rps * 2;
*/
void _Encoder_ResetUnitsAngularSpeed(Encoder_T * p_encoder)
{
    SetUnitsSpeed(p_encoder, &p_encoder->UnitAngularSpeed, &p_encoder->UnitAngularSpeedShift, ENCODER_ANGLE_DEGREES, 1U);
}

/*
    SurfaceToEncoder Ratio <=> gearRatio_Factor/gearRatio_Divisor

    SurfaceSpeed_DistancePerHour = (RPM * gearRatio_Divisor/gearRatio_Factor) * surfaceDiameter_Units * 314 / 100 * 60 / DistanceConversion
    RPM = 60 *(deltaD_Ticks * UnitT_Freq) / (CountsPerRevolution * deltaT_Ticks)

    SurfaceSpeed_UnitsPerSecond = (deltaD_Ticks * UnitT_Freq * gearRatio_Divisor * surfaceDiameter_Units * 314) / (CountsPerRevolution * deltaT_Ticks * gearRatio_Factor * 100))
    SurfaceSpeed_UnitsPerSecond = deltaD_Ticks * [[UnitT_Freq * gearRatio_Divisor * surfaceDiameter_Units * 314] / [CountsPerRevolution * gearRatio_Factor * 100]] / deltaT_Ticks

    SurfaceSpeed_DistancePerHour = ((60 * deltaD_Ticks * UnitT_Freq * gearRatio_Divisor * surfaceDiameter_Units * 314 * 60) / (CountsPerRevolution * deltaT_Ticks * gearRatio_Factor * 100 * DistanceConversion))

    Units => mm
    DistanceConversion:
    mph => 1,609,344
    kmh => 1,000,000

    e.g.
        UnitT_Freq = 1000, GearRatio = 8, SurfaceDiameter = 20in, 508mm, CountsPerRevolution = 8192
        UnitSurfaceSpeed = 24.3395996094
        1000 RPM => 7.44 mph, 11.97352 kmh
            => DeltaD = 136.533333333
        mph = 136.533333333 * 24.3395996094 * 60 * 60 / 1609344 = 7.4337121212
        mph = 136 * 24 * 60 * 60 / 1609344 = 7.3
        kmh = 136 * 24 * 60 * 60 / 1000000 = 11.7504

    DistancePerRevolution = [gearRatio_Divisor * surfaceDiameter_Mm * 314 / gearRatio_Factor * 100]
*/
void _Encoder_ResetUnitsLinearSpeed(Encoder_T * p_encoder)
{
    SetUnitsSpeed(p_encoder, &p_encoder->UnitSurfaceSpeed, &p_encoder->UnitSurfaceSpeedShift, (p_encoder->Params.GearRatio_Divisor * p_encoder->Params.SurfaceDiameter * 314), (p_encoder->Params.GearRatio_Factor * 100));
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
    p_encoder->Params.CountsPerRevolution = countsPerRevolution;
    _Encoder_ResetUnits(p_encoder);
}

void Encoder_SetScalarSpeedRef(Encoder_T * p_encoder, uint16_t speedRef)
{
    p_encoder->Params.ScalarSpeedRef_Rpm = speedRef;
    _Encoder_ResetUnitsScalarSpeed(p_encoder);
    _Encoder_ResetUnitsAngularSpeed(p_encoder);
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

/*
    gearRatio is Surface:Encoder
*/
void Encoder_SetSurfaceRatio(Encoder_T * p_encoder, uint32_t surfaceDiameter, uint32_t gearRatio_Factor, uint32_t gearRatio_Divisor)
{
    p_encoder->Params.SurfaceDiameter = surfaceDiameter;
    p_encoder->Params.GearRatio_Factor = gearRatio_Factor;
    p_encoder->Params.GearRatio_Divisor = gearRatio_Divisor;
    _Encoder_ResetUnitsLinearSpeed(p_encoder);
}

void Encoder_SetGroundRatio_US(Encoder_T * p_encoder, uint32_t wheelDiameter_Inch10, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Inch10 * 254 / 100, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);
}

void Encoder_SetGroundRatio_Metric(Encoder_T * p_encoder, uint32_t wheelDiameter_Mm, uint32_t wheelToMotorRatio_Factor, uint32_t wheelToMotorRatio_Divisor)
{
    Encoder_SetSurfaceRatio(p_encoder, wheelDiameter_Mm, wheelToMotorRatio_Factor, wheelToMotorRatio_Divisor);
}

