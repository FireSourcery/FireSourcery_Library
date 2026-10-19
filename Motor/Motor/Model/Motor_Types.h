#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Motor_Drive.h
    @author FireSourcery
    @brief
*/
/******************************************************************************/
#include "Math/Fixed/fract16.h"
#include "Math/Angle/angle_speed_math.h"
#include "Motor/Motor/Math/motor_electrical_math.h"

#include <stdint.h>
#include <stdbool.h>

/*
    Data object interfaces
*/

/******************************************************************************/
/*
    Unit p_basis-> 1.0 pu = V, I, F. The others derive, as in the host app's MotUnitBases:
        R = V / I    ψ = V / ω    L = ψ / I    P = V · I    ω = F [rad/s]
    P is the dq product, the 3/2 belongs to the quantity [foc_active_power].

    One basis per meaning of 1.0:
        Calib   V_MAX, I_MAX, speed base        what the loops, ramps and limits hold
        Turn    V_MAX, I_MAX, ω = π · Fs        ψ and L as the FOC holds them
        Rated   VNominal, I rated peak, speed at VNominal       100% to the user
*/
/******************************************************************************/
typedef struct Motor_UnitBasis
{
    uint16_t V;         /* [V] */
    uint16_t I;         /* [A] */
    angle_freq_t F;     /* [angle16/s] electrical */  /* or F_Hz */
}
Motor_UnitBasis_T;

/*
    Conversions at the user boundary. Units into pu scale 1.0 to INT16_MAX, as [Phase_V_PuOfVolts].
*/
static inline accum32_t Motor_UnitBasis_PuOfVolts(Motor_UnitBasis_T * p_basis, int16_t volts) { return volts * INT16_MAX / p_basis->V; }
static inline int16_t Motor_UnitBasis_VoltsOfPu(Motor_UnitBasis_T * p_basis, accum32_t pu) { return pu * p_basis->V / 32768; }

static inline accum32_t Motor_UnitBasis_PuOfAmps(Motor_UnitBasis_T * p_basis, int16_t amps) { return amps * INT16_MAX / p_basis->I; }
static inline int16_t Motor_UnitBasis_AmpsOfPu(Motor_UnitBasis_T * p_basis, accum32_t pu) { return pu * p_basis->I / 32768; }

/* Mechanical rpm at 1.0 pu */
static inline uint32_t Motor_UnitBasis_Rpm(Motor_UnitBasis_T * p_basis, uint8_t polePairs) { return mech_rpm_of_el_angle_freq(polePairs, p_basis->F); }
static inline accum32_t Motor_UnitBasis_PuOfRpm(Motor_UnitBasis_T * p_basis, uint8_t polePairs, int16_t rpm) { return rpm * INT16_MAX / (int32_t)Motor_UnitBasis_Rpm(p_basis, polePairs); }
static inline int16_t Motor_UnitBasis_RpmOfPu(Motor_UnitBasis_T * p_basis, uint8_t polePairs, accum32_t pu) { return pu * (int32_t)Motor_UnitBasis_Rpm(p_basis, polePairs) / 32768; }

static inline int32_t Motor_UnitBasis_WattsOfPu(Motor_UnitBasis_T * p_basis, accum32_t pu) { return (int64_t)pu * p_basis->V * p_basis->I / 32768; }

static inline uint32_t Motor_UnitBasis_PuOfMilliOhms(Motor_UnitBasis_T * p_basis, uint32_t mOhm) { return rs_pu_of_mohm(p_basis->V, p_basis->I, mOhm); }
static inline uint32_t Motor_UnitBasis_MilliOhmsOfPu(Motor_UnitBasis_T * p_basis, uint32_t pu) { return rs_mohm_of_pu(p_basis->V, p_basis->I, pu); }

static inline uint32_t Motor_UnitBasis_PuOfMicroWebers(Motor_UnitBasis_T * p_basis, uint32_t uWb) { return psi_pu_of_wb(p_basis->V, p_basis->F, uWb, 1000000UL); }
static inline uint32_t Motor_UnitBasis_MicroWebersOfPu(Motor_UnitBasis_T * p_basis, uint32_t pu) { return psi_wb_of_pu(p_basis->V, p_basis->F, pu, 1000000UL); }

static inline uint32_t Motor_UnitBasis_PuOfMicroHenries(Motor_UnitBasis_T * p_basis, uint32_t uH) { return l_pu_of_h(p_basis->V, p_basis->I, p_basis->F, uH, 1000000UL); }
static inline uint32_t Motor_UnitBasis_MicroHenriesOfPu(Motor_UnitBasis_T * p_basis, uint32_t pu) { return l_h_of_pu(p_basis->V, p_basis->I, p_basis->F, pu, 1000000UL); }


/*
    sign encodes direction. bits [0:1] encode torque
    state magnitude: 1=Motoring, 2=Regen, 3=Plugging
*/
typedef enum Motor_QuadrantId
{
    MOTOR_QUADRANT_REVERSE_PLUGGING = -3,    /* Q4: +Vq, +Iq, -ω - Vq opposes back-EMF */
    MOTOR_QUADRANT_REVERSE_REGEN    = -2,    /* Q4: -Vq, +Iq, -ω - VBemf < Vq < 0 */
    MOTOR_QUADRANT_REVERSE_MOTORING = -1,    /* Q3: -Vq, -Iq, -ω */
    MOTOR_QUADRANT_IDLE             = 0,
    MOTOR_QUADRANT_FORWARD_MOTORING = 1,    /* Q1: +Vq, +Iq, +ω */
    MOTOR_QUADRANT_FORWARD_REGEN    = 2,    /* Q2: +Vq, -Iq, +ω - VBemf > Vq > 0 */
    MOTOR_QUADRANT_FORWARD_PLUGGING = 3,    /* Q2: -Vq, -Iq, +ω - Vq opposes back-EMF */
}
Motor_QuadrantId_T;

/* motion: sign(ω), deadbanded by the caller. Plugging is generating with v_q against the motion. */
static inline Motor_QuadrantId_T Motor_QuadrantId(sign_t motion, sign_t torque, sign_t vq)
{
    bool generating = (torque * motion < 0);
    bool plugging = generating && (vq * motion < 0);
    return (Motor_QuadrantId_T)(motion * (1 + generating + plugging));
//     if (torque == 0 && vq == 0) { return MOTOR_QUADRANT_IDLE; }
//     if (torque > 0)  { return (vq >= 0) ? MOTOR_QUADRANT_FORWARD_MOTORING : MOTOR_QUADRANT_REVERSE_REGEN; }
//     else                { return (vq <= 0) ? MOTOR_QUADRANT_REVERSE_MOTORING : MOTOR_QUADRANT_FORWARD_REGEN; }
}

static inline sign_t Motor_Quadrant_Motion(Motor_QuadrantId_T q) { return math_sign(q); }
static inline sign_t Motor_Quadrant_Mechanical(Motor_QuadrantId_T q) { return (abs(q) == 1) - (abs(q) >= 2); } /* sign(τ·ω) */
static inline sign_t Motor_Quadrant_Electrical(Motor_QuadrantId_T q) { return (abs(q) & 1) - (abs(q) == 2); } /* sign(v_q·i_q): only regen returns power */
static inline sign_t Motor_Quadrant_Torque(Motor_QuadrantId_T q) { return Motor_Quadrant_Motion(q) * Motor_Quadrant_Mechanical(q); }

// typedef struct Motor_Quadrant { sign_t Motion; sign_t Torque; sign_t Power; } Motor_Quadrant_T;


/*
    Request. Stored raw; Motor applies the frame key and the envelope each tick.
    The raw value is what makes "limit active" observable. A request clamped when stored can't report the clamp.
*/
typedef enum Motor_CmdFrame
{
    MOTOR_CMD_FRAME_FORWARD,    /* signed by Config.DirectionForward: [Motor_SetTorqueCmd], [Motor_SetSpeedCmd] */
    MOTOR_CMD_FRAME_MOTORING,   /* signed by Direction: [Motor_SetICmd], [Motor_SetSpeedMotoringCmd] */
}
Motor_CmdFrame_T;

// typedef struct Motor_Cmd
// {
//     Motor_FeedbackMode_T Mode;  /* the quantity: V, I, or speed */
//     Motor_CmdFrame_T Frame;
//     int16_t Value;
// }
// Motor_Cmd_T;

/* [Cw:Ccw] */
// static inline int32_t Motor_Cmd_Signed(Motor_Cmd_T cmd, sign_t forward, sign_t motoring) { return ((cmd.Frame == MOTOR_CMD_FRAME_FORWARD) ? forward : motoring) * cmd.Value; }

// static inline Motor_Cmd_T Motor_Cmd_Motoring(int16_t Value) { return (Motor_Cmd_T){ .Frame = MOTOR_CMD_FRAME_MOTORING, .Value = Value, .Mode = MOTOR_FEEDBACK_MODE_CURRENT }; }
// static inline Motor_Cmd_T Motor_Cmd_Torque(int16_t Value) { return (Motor_Cmd_T){ .Frame = MOTOR_CMD_FRAME_FORWARD, .Value = Value, .Mode = MOTOR_FEEDBACK_MODE_CURRENT }; }
// static inline Motor_Cmd_T Motor_Cmd_Speed(int16_t Value) { return (Motor_Cmd_T){ .Frame = MOTOR_CMD_FRAME_FORWARD, .Value = Value, .Mode = MOTOR_FEEDBACK_MODE_SPEED_CURRENT }; }


// typedef struct Motor_FOC_OperatingPoint
// {
//     alignas(4) fract16_t Vd, Vq;    /* [V_base] applied, or the back-EMF while freewheeling */
//     alignas(4) fract16_t Id, Iq;    /* [I_base] */
//     angle16_t Delta;                /* ω [angle16/Ts], the decoupling basis */
//     ufract16_t VBus_Pu;             /* V_dc the voltages were produced under */
// }
// Motor_FOC_OperatingPoint_T;             /* 12 bytes */