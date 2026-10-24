#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   Sensorless_Sensor.h
    @author FireSourcery
    @brief  RotorSensor reading the rotor through the stator [EmfObserver_T].

    Differs from Hall/Encoder/SinCos: the transducer is push-driven, it needs i_αβ and v_αβ from FOC, not a peripheral.
    The integration layer invokes Sensorless_Sensor_Proc once per control tick, after the FOC proc. The vtable's
    CAPTURE_ANGLE/CAPTURE_SPEED publish the observer's θ̂, ω̂ into RotorSensor_State idempotently.

    Only the sensorless start-up substates (Motor_Sensorless.c) drive Proc; RotorSensor_CaptureAngle alone would publish a frozen θ̂.

    Or before the FOC proc, from the phase image and the voltage held across the period: Sensorless_Sensor_ProcEemf,
    by [Motor_Sensorless_CaptureStator]. CAPTURE_ANGLE then publishes the prediction the last step made for this sample.
*/
/******************************************************************************/
#include "../../Model/EmfObserver.h"
#include "../RotorSensor.h"
#include "../../Math/FOC.h"


typedef const struct Sensorless_Sensor
{
    const RotorSensor_T BASE;
    EmfObserver_T * P_OBSERVER;
    const EmfObserver_Config_T * P_NVM_CONFIG;
}
Sensorless_Sensor_T;

extern const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE;

/* Observer state of static duration, held by the const table entry */
#define SENSORLESS_SENSOR_INIT(p_RotorState, p_NvmConfig) (Sensorless_Sensor_T)    \
{                                                                                   \
    .BASE           = ROTOR_SENSOR_INIT(&SENSORLESS_SENSOR_VTABLE, (p_RotorState)), \
    .P_OBSERVER     = &(EmfObserver_T){},                                           \
    .P_NVM_CONFIG   = (p_NvmConfig),                                                \
}


/******************************************************************************/
/*!
    Push-driven entry
*/
/******************************************************************************/
/*
    One control tick, after the FOC proc: the period its sample closes and the command it made, from the αβ it carries.
*/
static inline void Sensorless_Sensor_Proc(Sensorless_Sensor_T * p_sensor, const FOC_T * p_foc)
{
    EmfObserver_AlphaBeta_Proc(p_sensor->P_OBSERVER, p_foc->Ialpha, p_foc->Ibeta, p_foc->Valpha, p_foc->Vbeta);
}

/*
    One control tick, before the FOC proc: the αβ sample, and the αβ voltage held across the period it closes.
    The EEMF front end in the frame θ̂ the tracker holds at the call [EmfObserver_Eemf_Proc]: i at θ̂, v at the period's mean angle θ̂ − Δθ̂/2.
*/
static inline void Sensorless_Sensor_ProcEemf(Sensorless_Sensor_T * p_sensor, fract16_t i_alpha, fract16_t i_beta, fract16_t v_alpha, fract16_t v_beta)
{
    EmfObserver_T * p_obs = p_sensor->P_OBSERVER;
    struct fract16_xy frame = Angle_UnitVector(&p_obs->Pll.Base);
    struct fract16_xy hold = fract16_vector((angle16_t)(Angle_Value(&p_obs->Pll.Base) - Angle_Delta(&p_obs->Pll.Base) / 2));
    struct foc_dq i = foc_park(i_alpha, i_beta, frame.y, frame.x);
    struct foc_dq v = foc_park(v_alpha, v_beta, hold.y, hold.x);
    EmfObserver_Eemf_Proc(p_obs, fract16_sat(i.d), fract16_sat(i.q), fract16_sat(v.d), fract16_sat(v.q));
}
