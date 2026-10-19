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
    @file   VBus_FOC.h
    @author FireSourcery
    @brief  DC bus and dq frame. Power balance between the supply and the FOC vectors.
*/
/******************************************************************************/
#include "VBus.h"
#include "../Math/foc_math.h"

/*
    Supply limit enforcement
        VBUS_I_LIMIT_POWER_BALANCE  q-axis limit at the operating point (v_d, i_d, v_q), exact
        VBUS_I_LIMIT_PHASE_BOUND    phase-current bound keyed by the motion, conservative below full modulation
        VBUS_I_LIMIT_V_DERATE       voltage ramps as ratios of each motor's I limits, no DC limit. Composed into the derate pair by the upper layer.
    The first two take different inputs. The caller selects at the operating point it holds.
*/
#if !defined(VBUS_I_LIMIT_POWER_BALANCE) && !defined(VBUS_I_LIMIT_PHASE_BOUND) && !defined(VBUS_I_LIMIT_V_DERATE)
#define VBUS_I_LIMIT_POWER_BALANCE
#endif

/******************************************************************************/
/*!
    Supply side of the power balance, [foc_i_dc] with the live V_bus.
    power in v·i units, [foc_active_power].
*/
/******************************************************************************/
static inline accum32_t _VBus_FOC_IBusOf(const VBus_T * p_vbus, accum32_t power) { return _foc_i_dc(power, VBus_Inv_Pu(p_vbus)); }
static inline accum32_t VBus_FOC_IBusOf(const VBus_T * p_vbus, accum32_t power) { return foc_i_dc(power, VBus_Inv_Pu(p_vbus)); }



/* VBus is the conceptual source. FOC_T acts as collaborator */
/******************************************************************************/
/*!
    Supply limits as q-axis limits. i_q along v_q draws from the supply, i_q against v_q returns to it.
*/
/******************************************************************************/
/* Power balance at the operating point (powerD = v_d·i_d, vq) */
// static inline uint16_t VBus_IqDischargeLimit_Pu(const VBus_T * p_vbus, accum32_t powerD, fract16_t vq) { return foc_iq_discharge_limit(VBus_IDischargeLimit_Pu(p_vbus), VBus_Pu(p_vbus), powerD, vq); }
// static inline uint16_t VBus_IqChargeLimit_Pu(const VBus_T * p_vbus, accum32_t powerD, fract16_t vq) { return foc_iq_charge_limit(VBus_IChargeLimit_Pu(p_vbus), VBus_Pu(p_vbus), powerD, vq); }
static inline uint16_t VBus_FOC_IqDischargeLimit_Pu(const VBus_T * p_vbus, fract16_t vd, fract16_t id, fract16_t vq) { return foc_iq_discharge_limit(VBus_IDischargeLimit_Pu(p_vbus), VBus_Pu(p_vbus), foc_power_d(vd, id), vq); }
static inline uint16_t VBus_FOC_IqChargeLimit_Pu(const VBus_T * p_vbus, fract16_t vd, fract16_t id, fract16_t vq) { return foc_iq_charge_limit(VBus_IChargeLimit_Pu(p_vbus), VBus_Pu(p_vbus), foc_power_d(vd, id), vq); }

// /* Phase-current bound on |i_dq| */
static inline uint16_t VBus_IPhaseDischargeLimit_Pu(const VBus_T * p_vbus) { return foc_i_phase_of_i_dc(VBus_IDischargeLimit_Pu(p_vbus)); }
static inline uint16_t VBus_IPhaseChargeLimit_Pu(const VBus_T * p_vbus) { return foc_i_phase_of_i_dc(VBus_IChargeLimit_Pu(p_vbus)); }


/******************************************************************************/
/*!
    Supply limits as q-axis limits, [Cw:Ccw]
*/
/******************************************************************************/
/* [foc_iq_limits] with the live V_bus */
/* v_q·i_q ∈ P_q = 2/3·V_dc·[−I_chg, I_dis] − v_d·i_d  ⟹  i_q ∈ P_q / v_q */
static inline interval_t VBus_FOC_IqLimits(const VBus_T * p_vbus, fract16_t vd, fract16_t id, fract16_t vq) { return foc_iq_limits(VBus_ILimitInterval(p_vbus), VBus_Pu(p_vbus), foc_power_d(vd, id), vq); }

/* [foc_iq_phase_limits], motion as the sign of the rotor speed */
static inline interval_t VBus_FOC_IqPhaseLimits(const VBus_T * p_vbus, sign_t motion, fract16_t id) { return foc_iq_phase_limits(VBus_ILimitInterval(p_vbus), motion, id); }



// /*
//     Supply. The bus as one motor sees it: one sample of V_dc, and this motor's share of the DC window.
//     Sampled once, so V and 1/V agree within the step. The split across motors is the upper layer's policy.
// */
// typedef struct Motor_Supply
// {
//     ufract16_t V;       /* V_dc */
//     uint32_t VInv;      /* 1/V_dc, accum32 */
//     interval_t I;       /* [−I_chg : I_dis] */
// }
// Motor_Supply_T;

// /* Even split. Demand-weighted is the alternative */
// static inline Motor_Supply_T Motor_Supply_Of(const VBus_T * p_vbus, uint8_t motorCount)
// {
//     interval_t window = VBus_ILimitInterval(p_vbus);
//     return (Motor_Supply_T) { .V = VBus_Pu(p_vbus), .VInv = VBus_Inv_Pu(p_vbus), .I = { .low = window.low / motorCount, .high = window.high / motorCount, }, };
// }

// /* [foc_iq_limits] at the sample */
// static inline interval_t Motor_Supply_IqLimits(const Motor_Supply_T * p_supply, fract16_t vd, fract16_t id, fract16_t vq) { return foc_iq_limits(p_supply->I, p_supply->V, foc_power_d(vd, id), vq); }