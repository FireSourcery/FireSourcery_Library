// #pragma once
// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2023 FireSourcery

//     This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
// */
// /******************************************************************************/
// /******************************************************************************/
// /*!
//     @file   Motor.h
//     @author FireSourcery
//     @brief  Per Motor State Control.
// */
// /******************************************************************************/
// #include "Phase/Phase_VOut.h"
// #include "Phase_Input/Phase_Input.h"
// #include "Phase_Input/Phase_Analog.h"
// #include "Phase_Input/Phase_Calibration.h"
// #include "VBus/VBus.h"
// #include "VBus/VBus_Monitor.h"

// #include "Sensor/RotorSensor_Table.h"
// #include "Sensor/RotorSensor.h"

// #include "Math/FOC.h"
// // #ifdef
// #include "Math/FOC_Sensorless.h"

// #include "Peripheral/ADC/ADC_Conversion.h"
// #include "Peripheral/ADC/Linear_ADC.h"

// #include "Transducer/Encoder/Encoder_ModeDT.h"
// #include "Transducer/Encoder/Encoder_ISR.h"
// #include "Transducer/Monitor/Heat/HeatMonitor.h"

// #include "Framework/StateMachine/StateMachine.h"
// #include "Framework/StateMachine/_StateMachine.h" /* Include the private header to contain StateMachine_Active_T within Motor_Context_T */
// #include "Framework/Timer/Timer.h"

// #include "Math/Fixed/fixed.h"
// #include "Math/Angle/Angle.h"
// #include "Math/Angle/Angle_SpeedPu.h"
// #include "Math/Linear/Linear.h"
// #include "Math/Accumulator/Accumulator.h"
// #include "Math/Ramp/Ramp.h"
// #include "Math/PID/PID.h"

// #include <stdint.h>
// #include <stdbool.h>
// #include <assert.h>




// /*
//     StateMachine Context shrinks to mutable state only.
//     Phase_T output must be buffered, cannot write directly in state machine
//         Disable has to go through a buffered value
//     RotorSensor must map to Motor mutable state, which it already it.
// */
// typedef struct
// {
//     /*
//         State and SubStates
//     */
//     StateMachine_Active_T StateMachine;     /* Compile time mapped address */
//     uint32_t ControlTimerBase;              /* Control Freq ~ 20kHz, state counter. Overflow 20Khz: 59 hours */

//     /* Effectively Substates StateMachine Controlled */
//     Motor_Direction_T Direction;            /* Direction of applied/cmd V. now shadows FOC.VLimit */
//     Motor_FeedbackMode_T FeedbackMode;      /* Active FeedbackMode, Control/Run SubState Flags */
//     Motor_FaultFlags_T FaultFlags;          /* Fault SubState */

//     /*
//         Position Sensor
//     */
//     const RotorSensor_T * p_ActiveSensor;   /* Pointer to entry in SENSOR_TABLE */
//     RotorSensor_State_T SensorState;        /* Compile time configured address. Sensor State includes [Angle_T] */

//     /*
//         Speed Feedback
//     */
//     Ramp_T SpeedRamp;                   /* { Target, Output, Limit, Coefficient } — full speed setpoint contract */
//     PID_T PidSpeed;                     /* Input PidSpeed(RampCmd - Speed_Fract16), Output => VPwm, Vq, Iq. */
//     // PID_T PidPosition;

//     volatile Phase_Input_T PhaseInput;
//     // Phase_Triplet_T VOut; /* output buffer */

//     /*
//         FOC
//     */
//     Ramp_T TorqueRamp;                      /* { Target, Output, Limit, Coefficient } — full torque setpoint contract */
//     FOC_T Foc;                              /* d-q vectors AND inner-loop PIDs (Foc.PidIq, Foc.PidId) */
//     // PID_T PidIPhase; /* Align, or use getter */
//     // Ramp_T VRamp;    /* Optional VRamp */

//     /*
//         Active Limit inputs. Unsigned user frame. Ramp.Limits holds the materialized [Cw:Ccw] output.
//         Value:  per-motor physical cap in PU, single user channel (user/OptDin/protocol overwrite). FRACT16_MAX => no cap.
//         Derate: system derate as pushed by the upper layer arbitration. FRACT16_MAX => none.
//             Held because Direction/FeedbackMode/Value changes re-resolve without the upper layer,
//             and Ramp.Limits cannot be read back — [0:0] on Direction NULL, V limits on TorqueRamp in voltage mode.
//     */
//     struct { uint16_t Motoring; uint16_t Generating; ufract16_t Derate; } ILimit;
//     struct { uint16_t Forward; uint16_t Reverse; ufract16_t Derate; } SpeedLimit;

//     /* OpenLoop Preset, StartUp. No boundary checking */
//     Ramp_T OpenLoopSpeedRamp;       /* Preset Speed Ramp */
//     Ramp_T OpenLoopIRamp;           /* Preset I Ramp */
//     // Ramp_T OpenLoopTorqueRamp;   /* Preset V/I Ramp */
//     Angle_T OpenLoopAngle;
//     Angle_SpeedUnitRef_T OpenLoopSpeedRef;

//     /*  */
//     HeatMonitor_State_T HeatMonitorState;

// }
// Motor_StateMachine_T;


// typedef const struct Motor
// {
//     Motor_Context_T * P_MOTOR;
//     const VBus_T * P_VBUS; /* Read-only. static instance. */
//     Phase_VOut_T PHASE;
//     Phase_Analog_T PHASE_ANALOG;
//     // RotorSensor_T SENSOR; /* Compile time default */
//     RotorSensor_Table_T SENSOR_TABLE; /* Runtime selection. Init macros in Motor_Sensor.h */
//     TimerT_T CONTROL_TIMER;     /* State Timer. Map to ControlTimerBase */
//     TimerT_T SPEED_TIMER;       /* Outer Speed Loop Timer. Millis */
//     const Motor_Config_T * P_NVM_CONFIG;
//     const FOC_Config_T * P_FOC_NVM_CONFIG; /* config for the FOC struct with a nested config field, without including a 3rd copy in Motor_Config */
//     /*  */
//     HeatMonitor_T HEAT_MONITOR;
//     ADC_Conversion_T HEAT_MONITOR_CONVERSION;
//     void * P_EXTENSION;
// }
// Motor_T;
