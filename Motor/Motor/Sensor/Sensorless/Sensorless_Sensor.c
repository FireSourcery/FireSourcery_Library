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
    @file   Sensorless_Sensor.c
    @author FireSourcery
*/
/******************************************************************************/
#include "Sensorless_Sensor.h"



/******************************************************************************/
/*!
    Vtable methods
*/
/******************************************************************************/
/* Tuning from NVM, and per-tick state cleared. The winding is not tuning, see [EmfObserver_Config_T] */
static void Sensorless_Sensor_Init(Sensorless_Sensor_T * p_sensor)
{
    EmfObserver_Init(p_sensor->P_OBSERVER, p_sensor->P_NVM_CONFIG);
}

/*
    Every control tick: θ̂ += ω̂, then publish. As Hall interpolates between edges, the correction lands once per sample [CAPTURE_STATOR].
    Once per tick: the only caller is Motor_CaptureSensor.
*/
static void Sensorless_Sensor_CaptureAngle(Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;
    AnglePll_Predict(&p_sensor->P_OBSERVER->Pll);

    p_rotor->AngleSpeed.Angle = p_sensor->P_OBSERVER->Pll.Base.Angle;
    _RotorSensor_SetDelta(p_rotor, p_sensor->P_OBSERVER->Pll.Base.Delta);
}

/*
    After the state machine [Motor_CaptureStator]: the correction of ω̂, for the next prediction. P_FOC this tick's take and write, the caller's constraint.
    i(k) the take's Id, Iq: in the frame θ̂ where the FOC Parked at θ̂, as RUN and INTERVENTION do.
    Elsewhere in the FOC's frame: the start-up chain's at the ramp, θ̂ itself while re-seeded. A take at another frame Parks Iα, Iβ at θ̂ [EmfObserver_ParkI].
    Under [FOC_I_SAMPLE_DELAY] the take is at θ̂ − ω·delay: not built for the observer.
    v(k−1) the command the observer holds [EmfObserver_Eemf_ProcHeld]; then holds v(k), the FOC's write, for the next sample.
*/
static void Sensorless_Sensor_CaptureStator(Sensorless_Sensor_T * p_sensor)
{
    EmfObserver_T * p_obs = p_sensor->P_OBSERVER;
    const FOC_T * p_foc = p_sensor->P_FOC;

    EmfObserver_Eemf_ProcHeld(p_obs, FOC_Id(p_foc), FOC_Iq(p_foc));
    EmfObserver_HoldV(p_obs, p_foc->Valpha, p_foc->Vbeta);
}

/* Project observer ω̂ → fract16 speed via the configured SpeedPuRef.
   Assumes CAPTURE_ANGLE already published Delta into AngleSpeed this tick. */
static void Sensorless_Sensor_CaptureSpeed(Sensorless_Sensor_T * p_sensor)
{
    RotorSensor_State_T * p_rotor = p_sensor->BASE.P_STATE;

    _RotorSensor_SetSpeedPu(p_rotor, Angle_ResolveSpeed_Pu(&p_rotor->AngleSpeed, &p_rotor->UnitRef.SpeedPuRef));
}

static bool Sensorless_Sensor_IsFeedbackAvailable(Sensorless_Sensor_T * p_sensor)
{
    return EmfObserver_IsLocked(p_sensor->P_OBSERVER);
}

static void Sensorless_Sensor_ZeroInitial(Sensorless_Sensor_T * p_sensor)
{
    EmfObserver_Reset(p_sensor->P_OBSERVER);
}

/* Observer is "calibrated" if the tuning gains and the plant are populated. K_smo == 0
   means K·sat(·) produces no forcing, LpfCoef == 0 freezes the EMF
   estimate, and G == 0 (no plant) holds î — each alone leaves the observer non-functional. */
static bool Sensorless_Sensor_VerifyCalibration(Sensorless_Sensor_T * p_sensor)
{
    const EmfObserver_Config_T * p_config = &p_sensor->P_OBSERVER->Config;
    return (p_config->K_smo != 0) && (p_config->LpfCoef != 0) && (p_sensor->P_OBSERVER->Winding.G != 0);
}

/*
    The winding, the observer's one unit: the plant from the motor's model, on the same view as FOC's [Motor_FOC_InitParams]: Lq on both axes, the extended EMF.
    With the other sensor units, on Motor_Reset: an electrical var applies to the FOC at once, to the winding on Motor_Reinit, as ψ does to the speed base.
    An invalid model keeps the prior winding, as it keeps FOC's view; VERIFY_CALIBRATION then fails on G == 0.
    UnitRef.SpeedPuRef, the only field CAPTURE_SPEED reads, is the RotorSensor base's.
*/
static void Sensorless_Sensor_InitUnits(Sensorless_Sensor_T * p_sensor)
{
    if (Motor_Electrical_IsValid(p_sensor->P_ELECTRICAL) == true)
    {
        EmfObserver_InitWinding(p_sensor->P_OBSERVER, p_sensor->P_ELECTRICAL->Rs, Motor_Lq_PuTurn(p_sensor->P_ELECTRICAL));
    }
}

/* No sensor-local unit state: the observer works entirely in the motor's per-unit
   basis. UnitRef.SpeedPuRef — the only field CAPTURE_SPEED reads — is held by the
   RotorSensor base, and the winding needs the motor's electrical model. */
// static void Sensorless_Sensor_InitUnits(Sensorless_Sensor_T * p_sensor)
// {
//     (void)p_sensor;
// }


const RotorSensor_VTable_T SENSORLESS_SENSOR_VTABLE =
{
    .INIT                  = (RotorSensor_Proc_T)Sensorless_Sensor_Init,
    .INIT_UNITS            = (RotorSensor_Proc_T)Sensorless_Sensor_InitUnits,
    .CAPTURE_ANGLE         = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureAngle,
    .CAPTURE_STATOR        = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureStator,
    .CAPTURE_SPEED         = (RotorSensor_Proc_T)Sensorless_Sensor_CaptureSpeed,
    .IS_FEEDBACK_AVAILABLE = (RotorSensor_Test_T)Sensorless_Sensor_IsFeedbackAvailable,
    .ZERO_INITIAL          = (RotorSensor_Proc_T)Sensorless_Sensor_ZeroInitial,
    .VERIFY_CALIBRATION    = (RotorSensor_Test_T)Sensorless_Sensor_VerifyCalibration,
};
