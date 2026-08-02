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
    @file   MotCan.h
    @author FireSourcery
    @brief  CAN service layer for motor controller.
            RX: control commands (throttle, brake).
            TX: periodic telemetry broadcasts.

    Frame values use the library's internal fract16 representation.
    Scaling to engineering units (RPM, Amps, Volts) is done on the host
    using the rated values readable via the serial protocol.

    IDs follow CANopen Tx/Rx PDO conventions (standard 11-bit):
        0x001         Control   RX  (throttle, brake)
        0x181 = 0x180 + node    Telemetry1 TX  (speed, current, voltage, vbus)
        0x182 = 0x180 + node    Telemetry2 TX  (heat, faults, state)
*/
/******************************************************************************/
#include "Motor/MotorController/MotorController.h"
#include "Motor/MotorController/Traction/MotorController_Traction.h"
#include "Motor/Motor/Motor_User.h"
#include "Peripheral/CanBus/CanBus.h"

#include <stdint.h>

/******************************************************************************/
/*! CAN IDs */
/******************************************************************************/
#define MOT_CAN_TX_TELEMETRY1_ID     (0x181U)   /* speed, IPhase, VPhase, VBus */
#define MOT_CAN_TX_TELEMETRY2_ID     (0x182U)   /* heat, fault flags, state */
#define MOT_CAN_RX_CONTROL_ID        (0x001U)
#define MOT_CAN_RX_VAR_ID            (0x1A0U)
#define MOT_CAN_RX_CONFIG_ID         (0x1B0U)

/******************************************************************************/
/*! Frame types */
/******************************************************************************/
/*!
    @brief  RX — throttle and brake commands from the host.
    Throttle and Brake: uint8 linear scale, 0x00 = 0%, 0xFF = 100%.
*/
typedef struct __attribute__((packed))
{
    uint8_t Throttle;
    uint8_t Brake;
    uint8_t Resv[6];
}
MotCan_TractionControl_T;

typedef struct __attribute__((packed))
{
    uint8_t CmdValue;
    uint8_t FeedbackMode;
    uint8_t Resv[6];
}
MotCan_MotorControl_T;

typedef struct __attribute__((packed))
{
    uint8_t ClearFaults;
    uint8_t StopAll;
    uint8_t Resv[6];
}
MotCan_StateControl_T;


/******************************************************************************/
/*!  */
/******************************************************************************/
// typedef struct __attribute__((packed))
// {
//     uint32_t Id;
// }
// MotCan_VarRead_T;

// typedef struct __attribute__((packed))
// {
//     uint32_t Value;
// }
// MotCan_VarReadResp_T;

// typedef struct __attribute__((packed))
// {
//     uint32_t Id;
//     uint32_t Value;
// }
// MotCan_VarWrite_T;

// typedef struct __attribute__((packed))
// {
//     uint16_t Status;
// }
// MotCan_VarWriteResp_T;



/******************************************************************************/
/*! TX broadcasts — call periodically (e.g. every 20 ms) */
/******************************************************************************/
/*!
    @brief  TX — primary motion telemetry.
    All values are fract16 fractions of rated quantities, little-endian.
        Speed:  fract16  [-32767, 32767] = [- , + ], direction signed.
        IPhase: fract16  [-32767, 32767] = [-I_calib, +I_calib], direction signed.
        VPhase: fract16  [-32767, 32767] = [-V_calib, +V_calib], direction signed.
        VBus:   fract16  [-32767, 32767] = [-V_calib, +V_calib], direction signed.
*/
typedef struct __attribute__((packed))
{
    int16_t Speed;
    int16_t IPhase;
    int16_t VPhase;
    int16_t VBus;
}
MotCan_Telemetry1_T;

/*!
    @brief  TX — secondary system status.
    Heat values: raw ADC counts scaled to uint8 (ADCU >> (ADC_BITS - 8)).
    FaultFlags: lower 8 bits of MotorController_FaultFlags_T.
    MotorState: active state machine state ID.
*/
typedef struct __attribute__((packed))
{
    uint8_t ControllerHeat;
    uint8_t MotorHeat;
    uint8_t FaultFlags;
    uint8_t MotorState;
    uint8_t StatusFlags;
    // uint8_t MotorFeedback;
    // uint8_t VOutState;
    uint8_t Resv;
}
MotCan_Telemetry2_T;

typedef struct __attribute__((packed))
{
    int16_t Id;
    int16_t Iq;
    int16_t Vd;
    int16_t Vq;
}
MotCan_TelemetryFoc_T;
