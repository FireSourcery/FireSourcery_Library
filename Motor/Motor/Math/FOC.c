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
    @file   FOC.c
    @author FireSourcery

*/
/******************************************************************************/
#include "FOC.h"

// static inline interval_t FOC_VqBandOf(const FOC_T * p_foc, ufract16_t vBus, accum32_t omega_psi, int)
// {
//     int32_t window = vBus / 8; /* plugging clamped beyond 1/2 VPhaseRef. e.g 20VBus -> [-2.5, 2.5] or [0, 5] */
//     return (interval_t) { .low = omega_psi - window, .high = omega_psi + window, };
// }

/* he controller to pick up the requested direction through zero */
static inline sign_t _FOC_VMotionSignOf(accum32_t omega_psi, int32_t iqReq)
{
    sign_t motionSign = math_sign(omega_psi);
    return (motionSign != SIGN_ZERO) ? motionSign : math_sign(iqReq);
}

static inline interval_t _FOC_VBrakeOverlay(accum32_t omega_psi, int32_t iqReq, int32_t vqLimit)
{
    sign_t motionSign = _FOC_VMotionSignOf(omega_psi, iqReq);
    /* Only narrow the band when the request is braking/generating. */
    if (motionSign * iqReq < 0) { return interval_of_sign(motionSign, vqLimit); }
    return interval_symmetric(0, vqLimit);
}

/* symmetric by default. dynamic half-plane anti-plugging derived per cycle */
static inline interval_t _FOC_VRegenOnly(int32_t omega_psi, int32_t iqReq, int32_t vqLimit)
{
    if (omega_psi * iqReq <= 0) { return interval_of_sign(omega_psi * iqReq, vqLimit); }
    return interval_symmetric(0, vqLimit);
}

// static inline interval_t _FOC_VRegenOnly(int32_t omega_psi, int32_t iqReq, int32_t vq) { return interval_of_sign(_FOC_VMotionSignOf(omega_psi, iqReq), vq); }
/* Applied Vq is forced opposite to EEMF for explicit plugging behavior. */
// static inline interval_t _FOC_VPluggingOnly(int32_t omega_psi, int32_t iqReq, int32_t vq) { return interval_of_sign((sign_t)(0 - _FOC_VMotionSignOf(omega_psi, iqReq)), vq); }

// static inline interval_t _FOC_VDynamicBand(const FOC_T * p_foc, accum32_t omega_psi, int32_t iqReq, ufract16_t vqCircleLimit)
// {
//     interval_t band = interval_symmetric(0, vqCircleLimit);

//     band = interval_intersect(band, _FOC_VRegenOnly(omega_psi, iqReq, vqCircleLimit));

//     if (p_foc->VWindow > 0)
//     {
//         band = interval_intersect(band, _FOC_VBemfWindow(vqCircleLimit, omega_psi, p_foc->VWindow));
//     }

//     return band;
// }

// /*
//     VqCircleLimit = 50
//     omega_psi = 80
//     window = 10
//     desired window = [70, 90]
//     feasible slice = [-50, 50]
//     intersection = [70, 50], invalid
// */
// static inline interval_t _FOC_VqBandOf(ufract16_t vqCircleLimit, accum32_t omega_psi, int32_t window)
// {
//     interval_t vqBand = interval_intersect(interval_symmetric(0, vqCircleLimit), interval_symmetric(omega_psi, window));

//     if (vqBand.high < vqBand.low)
//     {
//         int32_t vqEdge = math_clamp(omega_psi, -(int32_t)vqCircleLimit, (int32_t)vqCircleLimit);
//         return (interval_t) { .low = vqEdge, .high = vqEdge };
//     }
//     return vqBand;
//     // low = math_clamp(omega_psi - window, -(int32_t)vqCircleLimit, (int32_t)vqCircleLimit);
//     // high = math_clamp(omega_psi + window, -(int32_t)vqCircleLimit, (int32_t)vqCircleLimit);

//     // center = clamp(omega_psi, -VqCircleLimit, +VqCircleLimit)
//     // band = intersect([-VqCircleLimit, +VqCircleLimit], [center - window, center + window])
//