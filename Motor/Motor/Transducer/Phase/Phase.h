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
    @file   Phase.h
    @author FireSourcery
    @version V0
    @brief  3-Phase PWM functions. Includes 2-Phase Polar implementation

    Treats each phase as a complementary PWM output.
    2-Phase: PWM positive side MOSFETs, ground side bottom MOSFET stays on.
    e.g. PhaseAB -> PWM phase A MOSFETs, phase B bottom MOSFET stays on.

*/
/******************************************************************************/
#ifndef PHASE_H
#define PHASE_H

#include "Config.h"
#include "Peripheral/PWM/PWM.h"
#include "Peripheral/Pin/Pin.h"
#include <stdint.h>
#include <assert.h>


typedef enum Phase_State
{
    PHASE_STATE_FLOAT,  /* Disable  */
    PHASE_STATE_GROUND, /* VDuty 0 */
    PHASE_STATE_ACTIVE, /* VDuty */
}
Phase_State_T;

// typedef union Phase_Id
typedef union Phase_Flags
{
    struct
    {
        uint8_t A : 1U;
        uint8_t B : 1U;
        uint8_t C : 1U;
        uint8_t Resv : 5U;
    };
    uint8_t Value;
}
Phase_Flags_T;

typedef enum Phase_Mode
{
    PHASE_MODE_UNIPOLAR_1,    /*!<   */
    PHASE_MODE_UNIPOLAR_2,    /*!<   */
    PHASE_MODE_BIPOLAR        /*!<   */
}
Phase_Mode_T;

/* 3-Phase Active, Align */
typedef enum Phase_Align
{
    // PHASE_ID_0_GROUND,
    // PHASE_ID_7_ACTIVE,
    PHASE_ID_A,
    PHASE_ID_INV_C,
    PHASE_ID_B,
    PHASE_ID_INV_A,
    PHASE_ID_C,
    PHASE_ID_INV_B,
// #define    PHASE_ALIGN_A         (0b001U)
// #define    PHASE_ALIGN_B         (0b010U)
// #define    PHASE_ALIGN_C         (0b100U)
// #define    PHASE_ALIGN_INV_A     (0b110U)
// #define    PHASE_ALIGN_INV_B     (0b101U)
// #define    PHASE_ALIGN_INV_C     (0b011U)
}
Phase_Align_T;



static inline uint16_t Phase_AngleOf(Phase_Align_T id)
{
    static const uint16_t ANGLE_TABLE[] =
    {
        [PHASE_ID_A]        = 0U,         /* 0 */
        [PHASE_ID_INV_C]    = 10922U,     /* 60 */
        [PHASE_ID_B]        = 21845U,     /* 120 */
        [PHASE_ID_INV_A]    = 32768U,     /* 180 */
        [PHASE_ID_C]        = 43690U,     /* 240 */
        [PHASE_ID_INV_B]    = 54613U,     /* 300 */
        // [PHASE_ID_0]    = 0U,
        // [PHASE_ID_7]    = 0U,
    };

    return  ANGLE_TABLE[id];
}


/* 2-Phase Active, Six-Step Commutation */
typedef enum Phase_Polar
{
    PHASE_ID_0 = 0U,
    PHASE_ID_1_AC = 1U,
    PHASE_ID_2_BC = 2U,
    PHASE_ID_3_BA = 3U,
    PHASE_ID_4_CA = 4U,
    PHASE_ID_5_CB = 5U,
    PHASE_ID_6_AB = 6U,
    PHASE_ID_7 = 7U,
}
Phase_Polar_T;

typedef struct Phase
{
    PWM_T PwmA;
    PWM_T PwmB;
    PWM_T PwmC;
    PWM_T PwmModule;
#ifdef CONFIG_PHASE_PIN_SWITCH
    Pin_T PinA;
    Pin_T PinB;
    Pin_T PinC;
#endif
    Phase_Mode_T PhaseMode;
}
Phase_T;

//p_PwmAHal, p_PwmBHal, p_PwmCHal
#define PHASE_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel, PwmBChannel, PwmCChannel, p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId)    \
{                                                                   \
    .PwmModule = PWM_MODULE_INIT(p_PwmHal, PwmPeriodTicks),         \
    .PwmA = PWM_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel),        \
    .PwmB = PWM_INIT(p_PwmHal, PwmPeriodTicks, PwmBChannel),        \
    .PwmC = PWM_INIT(p_PwmHal, PwmPeriodTicks, PwmCChannel),        \
    .PinA = PIN_INIT(p_PinAHal, PinAId),                            \
    .PinB = PIN_INIT(p_PinBHal, PinBId),                            \
    .PinC = PIN_INIT(p_PinCHal, PinCId),                            \
}

static inline void Phase_ClearInterrupt(const Phase_T * p_phase)    { PWM_ClearInterrupt(&p_phase->PwmModule); }
static inline void Phase_DisableInterrupt(const Phase_T * p_phase)  { PWM_DisableInterrupt(&p_phase->PwmModule); }
static inline void Phase_EnableInterrupt(const Phase_T * p_phase)   { PWM_EnableInterrupt(&p_phase->PwmModule); }


/******************************************************************************/
/*! Extern */
/******************************************************************************/
extern void Phase_WriteDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);
extern void Phase_WriteDuty_Fract16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);
extern void Phase_WriteDuty_Percent16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);
extern void Phase_ActivateOutputABC(const Phase_T * p_phase);
extern void Phase_Float(const Phase_T * p_phase);
extern void Phase_Ground(const Phase_T * p_phase);
extern bool Phase_IsGround(const Phase_T * p_phase);
extern bool Phase_IsFloat(const Phase_T * p_phase);
extern Phase_State_T Phase_ReadState(const Phase_T * p_phase);
extern void Phase_ActivateState(const Phase_T * p_phase, Phase_State_T state);
extern void Phase_ActivateStateDuty(const Phase_T * p_phase, Phase_State_T state, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);

extern void Phase_Align_ActivateA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Align_ActivateB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Align_ActivateC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Align_ActivateInvA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Align_ActivateInvB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Align_ActivateInvC(const Phase_T * p_phase, uint16_t duty);

extern void Phase_Align_ActivateDuty(const Phase_T * p_phase, Phase_Align_T id, uint16_t duty);

extern void Phase_Polar_Ground(const Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputAC(const Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputBC(const Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputBA(const Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputCA(const Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputCB(const Phase_T * p_phase);
extern void Phase_Polar_ActivateOutputAB(const Phase_T * p_phase);

extern void Phase_Unipolar1_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar1_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty);

extern void Phase_Unipolar2_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Unipolar2_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty);

extern void Phase_Bipolar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Bipolar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty);

extern void Phase_Polar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty);

extern void Phase_Polar_ActivateAC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateBC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateBA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateCA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateCB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_Polar_ActivateAB(const Phase_T * p_phase, uint16_t duty);

extern void Phase_Init(Phase_T * p_phase);
extern void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode);
extern void Phase_Polar_Activate(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty);
extern void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty);
extern void Phase_Polar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId);
/******************************************************************************/
/*! @} */
/******************************************************************************/

#endif


