/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   SinCos_Sensor.c
    @author FireSourcery
    @brief  SinCos resolver RotorSensor implementation.
            Direct-sample sensor: each polling tick reads ADC -> atan2 -> electrical angle.
            Delta is derived from successive captures; speed is filtered from delta.
*/
/******************************************************************************/
#include "SinCos_Sensor.h"
#include "Math/Angle/Angle.h"


static void SinCos_RotorSensor_Init(const SinCos_RotorSensor_T * p_sensor)
{
    SinCos_Init(&p_sensor->SIN_COS);
}

/*
    Polling-rate angle capture.
    Each tick: read ADC -> mech angle -> electrical angle.
    Delta = wrapped (current - previous) gives signed shortest-path step,
    integrated into the AngleSpeed accumulator for downstream speed/IRP.
*/
static void SinCos_RotorSensor_CaptureAngle(const SinCos_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    SinCos_State_T * p_sinCosState = p_sensor->SIN_COS.P_STATE;

    SinCos_CaptureAngle(p_sinCosState, SinCos_Analog_GetSin(&p_sensor->ANALOG), SinCos_Analog_GetCos(&p_sensor->ANALOG));
    angle16_t electrical = SinCos_GetElectricalAngle(p_sinCosState);
    angle16_t delta = (angle16_t)(electrical - Angle_Value(&p_state->AngleSpeed));

    Angle_CaptureAngle(&p_state->AngleSpeed, electrical);
    Angle_CaptureDelta(&p_state->AngleSpeed, delta);

    p_state->MechanicalAngle = SinCos_GetMechanicalAngle(p_sinCosState);
}

/*
    1ms speed update — project Delta to fract16 via SpeedFractRef, low-pass filter.
*/
static void SinCos_RotorSensor_CaptureSpeed(const SinCos_RotorSensor_T * p_sensor)
{
    RotorSensor_State_T * p_state = p_sensor->BASE.P_STATE;
    accum32_t speed = Angle_ResolveSpeed_Fract16(&p_state->AngleSpeed, &p_state->SpeedFractRef);
    p_state->Speed_Fract16 = (speed + p_state->Speed_Fract16) / 2;
}

/* Direct-sample sensor: feedback always available once init/calibrated */
static bool SinCos_RotorSensor_IsFeedbackAvailable(const SinCos_RotorSensor_T * p_sensor) { (void)p_sensor; return true; }

static void SinCos_RotorSensor_ZeroInitial(const SinCos_RotorSensor_T * p_sensor)
{
    SinCos_State_T * p_sinCosState = p_sensor->SIN_COS.P_STATE;
    SinCos_CaptureAngle(p_sinCosState, SinCos_Analog_GetSin(&p_sensor->ANALOG), SinCos_Analog_GetCos(&p_sensor->ANALOG));
    Angle_CaptureAngle(&p_sensor->BASE.P_STATE->AngleSpeed, SinCos_GetElectricalAngle(p_sinCosState));
    Angle_StopDelta(&p_sensor->BASE.P_STATE->AngleSpeed);
    p_sensor->BASE.P_STATE->Speed_Fract16 = 0;
}

/* Min/Max ADC must form a valid scaling window */
static bool SinCos_RotorSensor_VerifyCalibration(const SinCos_RotorSensor_T * p_sensor)
{
    const SinCos_Config_T * p_config = &p_sensor->SIN_COS.P_STATE->Config;
    return (p_config->Max_Adcu > p_config->Min_Adcu) && (p_config->ElectricalRotationsRatio > 0U);
}

/*
    Wire mech-domain SpeedRated -> AngleSpeed projection via PolePairs.
*/
static void SinCos_RotorSensor_InitUnitsFrom(const SinCos_RotorSensor_T * p_sensor, const RotorSensor_Config_T * p_config)
{
    SinCos_Config_SetAngleRatio(p_sensor->SIN_COS.P_STATE, p_config->PolePairs);
}


const RotorSensor_VTable_T SIN_COS_VTABLE =
{
    .INIT                  = (RotorSensor_Proc_T)SinCos_RotorSensor_Init,
    .INIT_UNITS_FROM       = (RotorSensor_InitFrom_T)SinCos_RotorSensor_InitUnitsFrom,
    .CAPTURE_ANGLE         = (RotorSensor_Proc_T)SinCos_RotorSensor_CaptureAngle,
    .CAPTURE_SPEED         = (RotorSensor_Proc_T)SinCos_RotorSensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)SinCos_RotorSensor_IsFeedbackAvailable,
    .ZERO_INITIAL          = (RotorSensor_Proc_T)SinCos_RotorSensor_ZeroInitial,
    .VERIFY_CALIBRATION    = (RotorSensor_Test_T)SinCos_RotorSensor_VerifyCalibration,
};
