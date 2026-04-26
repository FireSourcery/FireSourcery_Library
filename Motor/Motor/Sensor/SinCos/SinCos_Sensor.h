#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   SinCos_Sensor.h
    @author FireSourcery
    @brief  Implement the RotorSensor interface for analog Sin/Cos resolver sensors.
            Composes: SinCos_T (decode core) + SinCos_Analog_T (ADC conversions).
*/
/******************************************************************************/
#include "../RotorSensor.h"
#include "SinCos.h"
#include "SinCos_Analog.h"

typedef const struct SinCos_RotorSensor
{
    RotorSensor_T BASE;             /* P_STATE->AngleSpeed is the final output */
    SinCos_T SIN_COS;               /* Decode core */
    SinCos_Analog_T ANALOG;         /* ADC conversion handles */
    uint32_t POLLING_FREQ;          /* Control loop frequency [Hz] */
}
SinCos_RotorSensor_T;

extern const RotorSensor_VTable_T SIN_COS_VTABLE;

#define SIN_COS_ROTOR_SENSOR_INIT(SinCosStruct, AnalogStruct, PollingFreq, p_State) (SinCos_RotorSensor_T) \
{                                                                                                         \
    .BASE         = ROTOR_SENSOR_INIT(&SIN_COS_VTABLE, p_State),                                          \
    .SIN_COS      = (SinCosStruct),                                                                       \
    .ANALOG       = (AnalogStruct),                                                                       \
    .POLLING_FREQ = (PollingFreq),                                                                        \
}

/*
    Convenience: bundle compose from raw fields.
    AdcSin/AdcCos: ADC peripheral structs.  IndexSin/IndexCos: channel indices.
*/
#define SIN_COS_ROTOR_SENSOR_INIT_FROM(p_SinCosState, p_SinCosNvm, AdcSin, IndexSin, AdcCos, IndexCos, PollingFreq, p_State) \
    SIN_COS_ROTOR_SENSOR_INIT(SIN_COS_INIT(p_SinCosState, p_SinCosNvm), SIN_COS_ANALOG_INIT(AdcSin, IndexSin, AdcCos, IndexCos), PollingFreq, p_State)

static inline void SinCos_RotorSensor_MarkAnalog(const SinCos_RotorSensor_T * p_sensor)
{
    SinCos_Analog_Mark(&p_sensor->ANALOG);
}
