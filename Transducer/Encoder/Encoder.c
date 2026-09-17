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
#if defined(ENCODER_HW_EMULATED)
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
#endif


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

void Encoder_SetCounterZero(const Encoder_T * p_encoder)
{
    _Encoder_ZeroPulseCount(p_encoder);
}

void Encoder_InitDirection(Encoder_T * p_encoder) { p_encoder->P_STATE->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder->P_STATE); }

/******************************************************************************/
/*!
    Set reference by homing or aligning
*/
/******************************************************************************/
/*
    Mechanical angle per open loop step, polling freq. Caller scales by PolePairs for the electrical command.
*/
uint16_t Encoder_GetHomingDelta(const Encoder_State_T * p_encoder)
{
    return ENCODER_ANGLE_DEGREES / math_max(p_encoder->Config.CountsPerRevolution, 1000U);
}

/*
    Search travel budget in open loop steps. 1.5 mechanical revolutions - a full revolution
    reaches any index position, the remainder covers the start offset.
*/
static inline uint32_t Encoder_GetHomingStepLimit(const Encoder_State_T * p_encoder)
{
    return (ENCODER_ANGLE_DEGREES / Encoder_GetHomingDelta(p_encoder)) * 3U / 2U;
}

/*
    Arms IndexAngleCapture. The ISR overwrites it on the next index edge.
*/
void Encoder_StartHoming(Encoder_State_T * p_encoder)
{
    p_encoder->IndexAngleCapture = ENCODER_INDEX_NOT_CAPTURED;
    p_encoder->HomingStepsRemaining = Encoder_GetHomingStepLimit(p_encoder);
}

/*
    Call once per open loop step of a homing sweep. Spends the travel budget.
    Status is a separate pure query, so the state machine's transition function stays side effect free.
*/
void Encoder_ProcHoming(Encoder_State_T * p_encoder)
{
    if (p_encoder->HomingStepsRemaining > 0U) { p_encoder->HomingStepsRemaining--; }
}

Encoder_HomingStatus_T Encoder_GetHomingStatus(const Encoder_State_T * p_encoder)
{
    if (Encoder_IsHomed(p_encoder) == true) { return ENCODER_HOMING_FOUND; }
    return (p_encoder->HomingStepsRemaining == 0U) ? ENCODER_HOMING_TIMEOUT : ENCODER_HOMING_SEARCHING;
}

Encoder_HomingStatus_T Encoder_PollHoming(Encoder_State_T * p_encoder)
{
    // Encoder_PollIndexCapture(p_encoder);
    if (Encoder_IsHomed(p_encoder) == true) { return ENCODER_HOMING_FOUND; }
    if (p_encoder->HomingStepsRemaining > 0U) { p_encoder->HomingStepsRemaining--; }
    return (p_encoder->HomingStepsRemaining == 0U) ? ENCODER_HOMING_TIMEOUT : ENCODER_HOMING_SEARCHING;
}

void Encoder_ClearPositionRef(Encoder_State_T * p_encoder)
{
    p_encoder->RefState = ENCODER_REF_STATE_NONE;
    p_encoder->IndexAngleCapture = ENCODER_INDEX_NOT_CAPTURED;
}

/*
    Derived on either offset write. Application zero expressed in the d-axis frame, so the
    position read is one subtraction and the commutation read stays free.
    VirtualHomeOffset may change at runtime with no other effect - nothing to re-snap or invalidate.
*/
void Encoder_ResolveUserZero(Encoder_State_T * p_encoder)
{
    p_encoder->UserZeroAngle = p_encoder->Config.IndexAngleOffset + p_encoder->Config.VirtualHomeOffset;
}

/*
    Commit the measured index position. The capture is the offset - there is nothing to converge,
    the prior value does not participate.
    Precondition: the pass started from a fresh align, so Capture is measured in the d-axis frame.

    The ISR snapped Base.Angle to the old offset at the index edge, so the live angle trails the
    committed frame by the error until the next Z. Carry it over, or every read between here and
    the next index edge - a virtual home taught right after calibrating included - inherits it.
    Bind the error first: the commit is what makes it zero.
*/
void Encoder_CalibrateIndexAngleOffset(Encoder_State_T * p_encoder)
{
    if (Encoder_IsHomed(p_encoder) == true)
    {
        p_encoder->Config.IndexAngleOffset = p_encoder->IndexAngleCapture;
        p_encoder->AngleCounter.Base.Angle += _Encoder_GetIndexAngleError(p_encoder);
        p_encoder->Config.IsIndexCalibrated = true;
        Encoder_ResolveUserZero(p_encoder);
    }
}

/*
    Teach the application zero at the current shaft position:
        VirtualHomeOffset = theta_now - theta_Z
    so that UserZeroAngle resolves to Base.Angle and Encoder_GetAngle_User reads 0 from here.

    Requires HOMED, not merely ALIGNED - the d-axis frame alone is ambiguous by a whole number of
    pole pitches, so an offset stored from it would be wrong by k * (360 deg / PolePairs) next run.
*/
void Encoder_CalibrateVirtualHomeOffset(Encoder_State_T * p_encoder)
{
    if (Encoder_IsHomed(p_encoder) == true)
    {
        p_encoder->Config.VirtualHomeOffset = p_encoder->AngleCounter.Base.Angle - p_encoder->Config.IndexAngleOffset;
        Encoder_ResolveUserZero(p_encoder);
    }
}


/*
    single store ISR
*/
// static inline void Encoder_CaptureIndex(Encoder_State_T * p_encoder)
// {
//     p_encoder->AngleCounter.Base.Angle = p_encoder->Config.IndexAngleOffset; /* Snap to Z */
// }

// void Encoder_StartCalibrateIndexAngleOffset(Encoder_State_T * p_encoder)
// {
//     p_encoder->Config.IndexAngleOffset = 0U;
//     HAL_Encoder_DisablePinInterrupt(p_encoder->P_HAL_PIN_Z, p_encoder->PIN_Z_ID);
// }

// void Encoder_PollCalibrateIndexAngle(Encoder_State_T * p_encoder)
// {
//     if (Pin_Input_ReadPhysical(&p_encoder->PIN_Z))
//     {
//         p_encoder->Config.IndexAngleOffset = p_encoder->AngleCounter.Base.Angle;
//         p_encoder->RefState = ENCODER_REF_STATE_HOMED;
//     }
// }



/******************************************************************************/
/*!
    Calibration for Reference
    Determine the values initially
*/
/******************************************************************************/
void Encoder_SetQuadratureMode(Encoder_State_T * p_encoder, bool isEnabled)
{
    p_encoder->Config.IsQuadratureCaptureEnabled = isEnabled;
    p_encoder->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder);
}
void Encoder_EnableQuadratureMode(Encoder_State_T * p_encoder) { Encoder_SetQuadratureMode(p_encoder, true); }
void Encoder_DisableQuadratureMode(Encoder_State_T * p_encoder) { Encoder_SetQuadratureMode(p_encoder, false); }

/*! isALeadBPositive - User runtime calibrate */
void Encoder_SetQuadratureDirection(Encoder_State_T * p_encoder, bool isALeadBPositive)
{
    p_encoder->Config.IsALeadBPositive = isALeadBPositive;
    p_encoder->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder);
}

/*
    Run on calibration routine start
*/
void Encoder_CaptureQuadratureReference(Encoder_T * p_encoder) { _Encoder_SetCounterD(p_encoder, 0); }

/*
    Call after having moved in the positive direction
*/
void Encoder_CalibrateQuadraturePositive(Encoder_T * p_encoder)
{
    p_encoder->P_STATE->Config.IsALeadBPositive = (Encoder_GetCounterD(p_encoder) > 0);
    p_encoder->P_STATE->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder->P_STATE);
}

void Encoder_CalibrateQuadratureDirection(Encoder_T * p_encoder, bool isPositive)
{
    p_encoder->P_STATE->Config.IsALeadBPositive = ((Encoder_GetCounterD(p_encoder) > 0) == isPositive);
    p_encoder->P_STATE->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder->P_STATE);
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

void Encoder_SetSpeedPerUnitRef(Encoder_State_T * p_encoder, uint16_t speedRef)
{
    p_encoder->Config.SpeedPerUnitRef_Rpm = speedRef;
    // Angle_SetSpeedRef_Rpm(&p_encoder->Base, speedRef);
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
static inline uint16_t Encoder_GetIndexAngleOffset(const Encoder_Config_T * p_encoder) { return p_encoder->IndexAngleOffset >> ENCODER_ANGLE_SHIFT; }
static inline void Encoder_SetIndexAngleOffset(Encoder_Config_T * p_encoder, uint16_t angle) { p_encoder->IndexAngleOffset = (uint32_t)angle << ENCODER_ANGLE_SHIFT; }

static inline uint16_t Encoder_GetVirtualHomeOffset(const Encoder_Config_T * p_encoder) { return p_encoder->VirtualHomeOffset >> ENCODER_ANGLE_SHIFT; }
static inline void Encoder_SetVirtualHomeOffset(Encoder_Config_T * p_encoder, uint16_t angle) { p_encoder->VirtualHomeOffset = (uint32_t)angle << ENCODER_ANGLE_SHIFT; }


int32_t _Encoder_ConfigId_Get(const Encoder_Config_T * p_encoder, Encoder_ConfigId_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case ENCODER_CONFIG_COUNTS_PER_REVOLUTION:             value = p_encoder->CountsPerRevolution;            break;
        case ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED:     value = p_encoder->IsQuadratureCaptureEnabled;     break;
        case ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE:              value = p_encoder->IsALeadBPositive;               break;
        case ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP:       value = p_encoder->ExtendedDeltaTStop;             break;
        case ENCODER_CONFIG_INDEX_ANGLE_OFFSET:                value = Encoder_GetIndexAngleOffset(p_encoder);    break;
        case ENCODER_CONFIG_VIRTUAL_HOME_OFFSET:               value = Encoder_GetVirtualHomeOffset(p_encoder);   break;
        case ENCODER_CONFIG_IS_INDEX_CALIBRATED:               value = p_encoder->IsIndexCalibrated;              break;
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
        case ENCODER_CONFIG_INDEX_ANGLE_OFFSET:                Encoder_SetIndexAngleOffset(p_encoder, varValue);  break;
        case ENCODER_CONFIG_VIRTUAL_HOME_OFFSET:               Encoder_SetVirtualHomeOffset(p_encoder, varValue); break;
        case ENCODER_CONFIG_IS_INDEX_CALIBRATED:               p_encoder->IsIndexCalibrated = varValue;           break;
    }
}

/*
    Set with propagate
*/
void Encoder_ConfigId_Set(const Encoder_T * p_encoder, Encoder_ConfigId_T varId, int32_t varValue)
{
    _Encoder_ConfigId_Set(&p_encoder->P_STATE->Config, varId, varValue);
    if (varId == ENCODER_CONFIG_EXTENDED_TIMER_DELTA_T_STOP)
    {
        PulseTimer_SetExtendedWatchStop_Millis(&p_encoder->TIMER, p_encoder->P_STATE->Config.ExtendedDeltaTStop);
    }
    if ((varId == ENCODER_CONFIG_IS_QUADRATURE_CAPTURE_ENABLED) || (varId == ENCODER_CONFIG_IS_A_LEAD_B_POSITIVE))
    {
        p_encoder->P_STATE->DirectionComp = _Encoder_ResolveDirectionComp(p_encoder->P_STATE);
    }
    if ((varId == ENCODER_CONFIG_INDEX_ANGLE_OFFSET) || (varId == ENCODER_CONFIG_VIRTUAL_HOME_OFFSET))
    {
        Encoder_ResolveUserZero(p_encoder->P_STATE);
    }
    // _Encoder_ResetUnits(p_encoder);
}

/*
    alternatively move to outer file
*/
static inline int32_t Encoder_ModeDT_GetRotationalSpeed_RPM(const Encoder_State_T * p_encoder) { return rpm_of_count_freq(p_encoder->Config.CountsPerRevolution, p_encoder->AngleCounter.FreqD); }


/******************************************************************************/
/*!

*/
/******************************************************************************/
int32_t Encoder_VarId_Get(const Encoder_State_T * p_encoder, Encoder_VarId_T varId)
{
    int32_t value = 0;
    switch (varId)
    {
        case ENCODER_VAR_FREQ:                  value = p_encoder->AngleCounter.FreqD;     break;
        case ENCODER_VAR_COUNTER_D:             value = p_encoder->AngleCounter.CounterD;  break;
        case ENCODER_VAR_RPM:                   value = Encoder_ModeDT_GetRotationalSpeed_RPM(p_encoder);     break;
        // case ENCODER_VAR_DELTA_T_SPEED:      value = Encoder_DeltaT_GetRotationalSpeed_RPM(p_encoder);     break;
        // case ENCODER_VAR_DELTA_D_SPEED:      value = Encoder_DeltaD_GetRotationalSpeed_RPM(p_encoder);     break;
        case ENCODER_VAR_DELTA_T_SPEED:         break; /* not yet implemented */
        case ENCODER_VAR_DELTA_D_SPEED:         break; /* not yet implemented */

        case ENCODER_VAR_ANGLE:                 value = Angle_Value(&p_encoder->AngleCounter.Base);      break;
        case ENCODER_VAR_ANGLE_USER:            value = Encoder_GetAngle_User(p_encoder);                break;

        case ENCODER_VAR_REF_STATE:             value = p_encoder->RefState;                             break;
        case ENCODER_VAR_HOMING_STATUS:         value = Encoder_GetHomingStatus(p_encoder);              break;
        case ENCODER_VAR_IS_INDEX_CAPTURED:     value = Encoder_IsIndexCaptured(p_encoder);              break;

        /* Meaningless until an edge lands - the armed sentinel is not an angle */
        case ENCODER_VAR_INDEX_ANGLE_ERROR:     value = Encoder_IsIndexCaptured(p_encoder) ? Encoder_GetIndexAngleError(p_encoder) : 0; break;
        case ENCODER_VAR_ERROR_COUNT:           value = p_encoder->ErrorCount;                           break;
        case ENCODER_VAR_DIRECTION_COMP:        value = p_encoder->DirectionComp;                        break;
        case ENCODER_VAR_PHASES:                value = p_encoder->Phases.Value;                         break;
    }
    return value;
}
