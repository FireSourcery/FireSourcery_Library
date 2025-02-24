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

typedef uint8_t phase_id_t; /* for interface casting */

// Phase_Id/State
typedef union Phase_Bits
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
Phase_Bits_T;

/*
    3-Phase Active/Align
*/
typedef enum Phase_Id
{
    PHASE_ID_DISABLE    = (0b000U),
    PHASE_ID_A          = (0b001U),
    PHASE_ID_B          = (0b010U),
    PHASE_ID_INV_C      = (0b011U),
    PHASE_ID_C          = (0b100U),
    PHASE_ID_INV_B      = (0b101U),
    PHASE_ID_INV_A      = (0b110U),
    PHASE_ID_ABC        = (0b111U),
}
Phase_Id_T;

// static inline Phase_Id_T Phase_IdOf(Phase_Bits_T bits) { return (Phase_Id_T)bits.Value; }
static inline Phase_Bits_T Phase_BitsOf(Phase_Id_T id) { return (Phase_Bits_T){ .Value = id }; }

static inline Phase_Id_T Phase_NextOf(Phase_Id_T id)
{
    Phase_Id_T next;
    switch (id)
    {
        case PHASE_ID_A:        next = PHASE_ID_INV_C;      break;
        case PHASE_ID_INV_C:    next = PHASE_ID_B;          break;
        case PHASE_ID_B:        next = PHASE_ID_INV_A;      break;
        case PHASE_ID_INV_A:    next = PHASE_ID_C;          break;
        case PHASE_ID_C:        next = PHASE_ID_INV_B;      break;
        case PHASE_ID_INV_B:    next = PHASE_ID_A;          break;
        case PHASE_ID_DISABLE:  next = PHASE_ID_DISABLE;    break;
        case PHASE_ID_ABC:      next = PHASE_ID_ABC;        break;
        default: assert(false); break;
    }
    return next;
}

static inline Phase_Id_T Phase_PrevOf(Phase_Id_T id)
{
    Phase_NextOf(Phase_NextOf(~id));
}

static inline uint16_t Phase_AngleOf(Phase_Id_T id)
{
    static const uint16_t ANGLE_TABLE[] =
    {
        [PHASE_ID_A]        = 0U,         /* 0 */
        [PHASE_ID_INV_C]    = 10922U,     /* 60 */
        [PHASE_ID_B]        = 21845U,     /* 120 */
        [PHASE_ID_INV_A]    = 32768U,     /* 180 */
        [PHASE_ID_C]        = 43690U,     /* 240 */
        [PHASE_ID_INV_B]    = 54613U,     /* 300 */
        [PHASE_ID_DISABLE]  = 0U,
        [PHASE_ID_ABC]      = 0U,
    };

    return ANGLE_TABLE[id];
}

/*
    Output State
    As collective or single-phase.
*/
typedef enum Phase_Output
{
    PHASE_OUTPUT_FLOAT,  /* Disable, 0 as High-Z, since it is the result of Pin 0/Low */
    PHASE_OUTPUT_GROUND, /* VDuty 0, Pin High/1 */
    /* PHASE_STATE_RESV = 0b10 */ /* This way bit 1 reflects pin on/off, bit 2 reflects pwm value */
    PHASE_OUTPUT_VPWM,   /* VDuty +, Pin High/1 */
}
Phase_Output_T;

/*
    2-Phase Active, Six-Step Commutation
*/
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

/* PolarMode */
typedef enum Phase_Mode
{
    PHASE_MODE_UNIPOLAR_1,    /*!<   */
    PHASE_MODE_UNIPOLAR_2,    /*!<   */
    PHASE_MODE_BIPOLAR        /*!<   */
}
Phase_Mode_T;

typedef struct Phase
{
    PWM_Module_T PwmModule;
    PWM_T PwmA;
    PWM_T PwmB;
    PWM_T PwmC;
    Pin_T PinA;
    Pin_T PinB;
    Pin_T PinC;
    Phase_Mode_T PolarMode;
}
Phase_T;

//p_PwmAHal, p_PwmBHal, p_PwmCHal

/* Module same as Channels */
#define PHASE_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel, PwmBChannel, PwmCChannel, p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId)    \
{                                                                                   \
    .PwmModule = PWM_MODULE_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel, PwmBChannel, PwmCChannel),  \
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
/*!
    Phase State
*/
/******************************************************************************/
/******************************************************************************/
/*!  */
/******************************************************************************/
static inline void _Phase_Enable(const Pin_T * p_pin, const PWM_T * p_pwm)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    Pin_Output_High(p_pin);
#else
    PWM_Enable(p_pwm);
#endif
}

static inline void _Phase_Disable(const Pin_T * p_pin, const PWM_T * p_pwm)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    Pin_Output_Low(p_pin);
#else
    PWM_Disable(p_pwm); /* alt for high z */
#endif
}

static inline bool _Phase_IsEnabled(const Pin_T * p_pin, const PWM_T * p_pwm)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    return Pin_Output_ReadPhysical(p_pin);
#else
    return PWM_ReadOutputState(p_pwm);
#endif
}

static inline void _Phase_WriteEnabled(const Pin_T * p_pin, const PWM_T * p_pwm, bool isOn)
{
    if (isOn == true) { _Phase_Enable(p_pin, p_pwm); } else { _Phase_Disable(p_pin, p_pwm); }
}

static inline void _Phase_EnableA(const Phase_T * p_phase) { _Phase_Enable(&p_phase->PinA, &p_phase->PwmA); }
static inline void _Phase_EnableB(const Phase_T * p_phase) { _Phase_Enable(&p_phase->PinB, &p_phase->PwmB); }
static inline void _Phase_EnableC(const Phase_T * p_phase) { _Phase_Enable(&p_phase->PinC, &p_phase->PwmC); }
static inline void _Phase_DisableA(const Phase_T * p_phase) { _Phase_Disable(&p_phase->PinA, &p_phase->PwmA); }
static inline void _Phase_DisableB(const Phase_T * p_phase) { _Phase_Disable(&p_phase->PinB, &p_phase->PwmB); }
static inline void _Phase_DisableC(const Phase_T * p_phase) { _Phase_Disable(&p_phase->PinC, &p_phase->PwmC); }
static inline bool _Phase_IsEnabledA(const Phase_T * p_phase) { _Phase_IsEnabled(&p_phase->PinA, &p_phase->PwmA); }
static inline bool _Phase_IsEnabledB(const Phase_T * p_phase) { _Phase_IsEnabled(&p_phase->PinB, &p_phase->PwmB); }
static inline bool _Phase_IsEnabledC(const Phase_T * p_phase) { _Phase_IsEnabled(&p_phase->PinC, &p_phase->PwmC); }
static inline bool _Phase_WriteStateA(const Phase_T * p_phase, bool isOn) { _Phase_WriteEnabled(&p_phase->PinA, &p_phase->PwmA, isOn); }
static inline bool _Phase_WriteStateB(const Phase_T * p_phase, bool isOn) { _Phase_WriteEnabled(&p_phase->PinB, &p_phase->PwmB, isOn); }
static inline bool _Phase_WriteStateC(const Phase_T * p_phase, bool isOn) { _Phase_WriteEnabled(&p_phase->PinC, &p_phase->PwmC, isOn); }

/******************************************************************************/
/*! Sync Write */
/******************************************************************************/
static inline uint32_t _Phase_PwmSyncOf(const Phase_T * p_phase, Phase_Id_T id)
{
    const Phase_Bits_T state = Phase_BitsOf(id);
    return (_PWM_ChannelMaskOf(&p_phase->PwmA, state.A) | _PWM_ChannelMaskOf(&p_phase->PwmB, state.B) | _PWM_ChannelMaskOf(&p_phase->PwmC, state.C));
// #else
// return _PWM_Module_ChannelMaskOf(&p_phase->PwmModule, state.Value);
// #endif
}

static inline uint32_t _Phase_PinSyncOf(const Phase_T * p_phase, Phase_Id_T id)
{
    const Phase_Bits_T state = Phase_BitsOf(id);
    return (Pin_Module_MaskOf(&p_phase->PinA, state.A) | Pin_Module_MaskOf(&p_phase->PinB, state.B) | Pin_Module_MaskOf(&p_phase->PinC, state.C));
}

/* optionally disable for counter up/down sync */
static inline void _Phase_SyncPwmDuty(const Phase_T * p_phase, Phase_Id_T state)
{
// #ifdef CONFIG_PHASE_SYNC_DUTY_UPDATE
    _PWM_Module_WriteSyncDuty(&p_phase->PwmModule, _Phase_PwmSyncOf(p_phase, state));
// #endif
}

/* Sync activation of Switch and Invert Polarity */
/* Bit Reg operation need to be in single write for sync update  */
static inline void _Phase_SyncPwmInvert(const Phase_T * p_phase, Phase_Id_T state)
{
#ifdef CONFIG_PHASE_SYNC_INVERT_UPDATE
    _PWM_Module_WriteSyncInvert(&p_phase->PwmModule, _Phase_PwmSyncOf(p_phase, state));
#endif
}

static inline void _Phase_SyncPwmOnOff(const Phase_T * p_phase, Phase_Id_T state)
{
#ifdef CONFIG_PHASE_PIN_SYNC
    // _Phase_PinSyncOf(p_phase, state);
#endif
#ifndef CONFIG_PHASE_PIN_SWITCH
    _PWM_Module_WriteSyncOnOff(&p_phase->PwmModule, _Phase_PwmSyncOf(p_phase, state));
#endif
}

/******************************************************************************/
/*!   */
/******************************************************************************/
static inline Phase_Bits_T _Phase_ReadState(const Phase_T * p_phase)
{
    return (Phase_Bits_T) { .A = _Phase_IsEnabledA(p_phase), .B = _Phase_IsEnabledB(p_phase), .C = _Phase_IsEnabledC(p_phase) };
}

/* Let the compiler optimize/expand into the 8 derived functions */
static inline void _Phase_WriteState(const Phase_T * p_phase, Phase_Id_T id)
{
#ifdef CONFIG_PHASE_PIN_SYNC
    _Phase_SyncPwmOnOff(p_phase, id);
#else
    const Phase_Bits_T state = Phase_BitsOf(id);
    _Phase_WriteStateA(p_phase, state.A);
    _Phase_WriteStateB(p_phase, state.B);
    _Phase_WriteStateC(p_phase, state.C);
#endif
}

/* Voltage/Output State */
static inline Phase_Bits_T _Phase_ReadDutyState(const Phase_T * p_phase)
{
    return (Phase_Bits_T) { .A = (PWM_ReadDuty_Ticks(&p_phase->PwmA) != 0U), .B = (PWM_ReadDuty_Ticks(&p_phase->PwmB) != 0U), .C = (PWM_ReadDuty_Ticks(&p_phase->PwmC) != 0U) };
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*! Extern */
/******************************************************************************/
extern void Phase_WriteDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);
extern void Phase_WriteDuty_Fract16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);
extern void Phase_WriteDuty_Percent16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC);
extern void Phase_ActivateOutput(const Phase_T * p_phase);
extern void Phase_Float(const Phase_T * p_phase);
extern void Phase_Ground(const Phase_T * p_phase);
extern bool Phase_IsGround(const Phase_T * p_phase);
extern bool Phase_IsFloat(const Phase_T * p_phase);
extern Phase_Output_T Phase_ReadOutputState(const Phase_T * p_phase);
extern void Phase_ActivateOutputState(const Phase_T * p_phase, Phase_Output_T state);

Phase_Id_T Phase_ReadAlign(const Phase_T * p_phase);
Phase_Id_T Phase_ReadAlignNext(const Phase_T * p_phase);

Phase_Id_T Phase_JogDirection(const Phase_T * p_phase, uint16_t duty, bool ccw);

extern void Phase_Align(const Phase_T * p_phase, Phase_Id_T id, uint16_t duty);

extern void Phase_ActivateAlignA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_ActivateAlignB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_ActivateAlignC(const Phase_T * p_phase, uint16_t duty);
extern void Phase_ActivateAlignInvA(const Phase_T * p_phase, uint16_t duty);
extern void Phase_ActivateAlignInvB(const Phase_T * p_phase, uint16_t duty);
extern void Phase_ActivateAlignInvC(const Phase_T * p_phase, uint16_t duty);


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


