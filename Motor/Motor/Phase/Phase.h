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
    @file   Phase.h
    @author FireSourcery
    @brief  3-Phase PWM functions. Submodule for Motor VOut.
*/
/******************************************************************************/
#include "Phase_Types.h"

#include "Peripheral/PWM/PWM.h"
#include "Peripheral/Pin/Pin.h"

#include <stdint.h>
#include <assert.h>
#include <sys/types.h>


/*
    VOutput State
    As collective ABC ouput or 3-state interpretation of a single-phase.
*/
// typedef enum Phase_VOutId
/* PHASE_VOUT_Z */ /* PHASE_VOUT_0 */ /* PHASE_VOUT_DUTY */
typedef enum Phase_Output
{
    PHASE_OUTPUT_FLOAT,  /* Disable, 0 as High-Z, it is the result of Pin Low/0 */
    PHASE_OUTPUT_V0,     /* VDuty 0, Pin High/1 */
    PHASE_OUTPUT_VPWM,   /* VDuty +, Pin High/1 */
}
Phase_Output_T;

/******************************************************************************/
/*!
    Phase Actuator - V PWM Out / Control / Inverter
    Phase_VOut
*/
/******************************************************************************/
typedef const struct Phase
{
    const PWM_Module_T PWM_MODULE;
    const PWM_T PWM_A;
    const PWM_T PWM_B;
    const PWM_T PWM_C;
    const Pin_T PIN_A;
    const Pin_T PIN_B;
    const Pin_T PIN_C;
}
Phase_T;

/* Module common for all Channels */
#define PHASE_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel, PwmBChannel, PwmCChannel, p_PinAHal, PinAId, p_PinBHal, PinBId, p_PinCHal, PinCId)    \
{                                                                                   \
    .PWM_MODULE = PWM_MODULE_INIT(p_PwmHal, PwmPeriodTicks, PwmAChannel, PwmBChannel, PwmCChannel),  \
    .PWM_A = PWM_INIT(p_PwmHal, PwmAChannel, PwmPeriodTicks),        \
    .PWM_B = PWM_INIT(p_PwmHal, PwmBChannel, PwmPeriodTicks),        \
    .PWM_C = PWM_INIT(p_PwmHal, PwmCChannel, PwmPeriodTicks),        \
    .PIN_A = PIN_INIT(p_PinAHal, PinAId),                            \
    .PIN_B = PIN_INIT(p_PinBHal, PinBId),                            \
    .PIN_C = PIN_INIT(p_PinCHal, PinCId),                            \
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void Phase_ClearInterrupt(const Phase_T * p_phase)    { PWM_ClearInterrupt(&p_phase->PWM_MODULE); }
static inline void Phase_DisableInterrupt(const Phase_T * p_phase)  { PWM_DisableInterrupt(&p_phase->PWM_MODULE); }
static inline void Phase_EnableInterrupt(const Phase_T * p_phase)   { PWM_EnableInterrupt(&p_phase->PWM_MODULE); }

/******************************************************************************/
/*!
    Private
*/
/******************************************************************************/
// #if defined(PHASE_PIN_SWITCH)
#define _PHASE_PIN_DEF(code, ...) code
#define _PHASE_PWM_ONLY_DEF(code)
// #define _PHASE_PIN_DEF_INIT(code) code,
// #else
// #define _PHASE_PIN_DEF(code, ...) __VA_ARGS__
// #define _PHASE_PWM_ONLY_DEF(code) code
// #endif

/******************************************************************************/
/*!
    Phase On/Off Output Control
*/
/******************************************************************************/
static inline void _Phase_Enable(const Pin_T * p_pin, const PWM_T * p_pwm) { _PHASE_PIN_DEF((void)p_pwm; Pin_Output_High(p_pin), PWM_Enable(p_pwm)); }
/* PWM_Disable as alt for high z */
static inline void _Phase_Disable(const Pin_T * p_pin, const PWM_T * p_pwm) { _PHASE_PIN_DEF((void)p_pwm; Pin_Output_Low(p_pin), PWM_Disable(p_pwm)); }
// static inline void _Phase_Enable(const Pin_T * p_pin, const PWM_T * p_pwm) { Pin_Output_High(p_pin); PWM_Enable(p_pwm); }
// static inline void _Phase_Disable(const Pin_T * p_pin, const PWM_T * p_pwm) { Pin_Output_Low(p_pin); PWM_Disable(p_pwm); }

static inline void _Phase_WriteOnOff(const Pin_T * p_pin, const PWM_T * p_pwm, bool isOn)
{
    if (isOn == true) { _Phase_Enable(p_pin, p_pwm); } else { _Phase_Disable(p_pin, p_pwm); }
    // _PHASE_PIN_DEF(Pin_Output_WritePhysical(p_pin, isOn), PWM_WriteEnable(p_pwm, isOn));
}

/* ReadOnOff using register state */
static inline bool _Phase_ReadOnOff(const Pin_T * p_pin, const PWM_T * p_pwm) { return _PHASE_PIN_DEF(Pin_Output_ReadPhysical(p_pin), PWM_ReadOutputState(p_pwm)); }

static inline void _Phase_EnableA(const Phase_T * p_phase) { _Phase_Enable(&p_phase->PIN_A, &p_phase->PWM_A); }
static inline void _Phase_EnableB(const Phase_T * p_phase) { _Phase_Enable(&p_phase->PIN_B, &p_phase->PWM_B); }
static inline void _Phase_EnableC(const Phase_T * p_phase) { _Phase_Enable(&p_phase->PIN_C, &p_phase->PWM_C); }
static inline void _Phase_DisableA(const Phase_T * p_phase) { _Phase_Disable(&p_phase->PIN_A, &p_phase->PWM_A); }
static inline void _Phase_DisableB(const Phase_T * p_phase) { _Phase_Disable(&p_phase->PIN_B, &p_phase->PWM_B); }
static inline void _Phase_DisableC(const Phase_T * p_phase) { _Phase_Disable(&p_phase->PIN_C, &p_phase->PWM_C); }
static inline void _Phase_WriteOnOffA(const Phase_T * p_phase, bool isOn) { _Phase_WriteOnOff(&p_phase->PIN_A, &p_phase->PWM_A, isOn); }
static inline void _Phase_WriteOnOffB(const Phase_T * p_phase, bool isOn) { _Phase_WriteOnOff(&p_phase->PIN_B, &p_phase->PWM_B, isOn); }
static inline void _Phase_WriteOnOffC(const Phase_T * p_phase, bool isOn) { _Phase_WriteOnOff(&p_phase->PIN_C, &p_phase->PWM_C, isOn); }
static inline bool _Phase_ReadOnOffA(const Phase_T * p_phase) { return _Phase_ReadOnOff(&p_phase->PIN_A, &p_phase->PWM_A); }
static inline bool _Phase_ReadOnOffB(const Phase_T * p_phase) { return _Phase_ReadOnOff(&p_phase->PIN_B, &p_phase->PWM_B); }
static inline bool _Phase_ReadOnOffC(const Phase_T * p_phase) { return _Phase_ReadOnOff(&p_phase->PIN_C, &p_phase->PWM_C); }

/******************************************************************************/
/*!
    Sync Write
*/
/******************************************************************************/
/*!
    Bits/Mask
*/
static inline uint32_t _Phase_PwmSyncOf(const Phase_T * p_phase, Phase_Id_T id)
{
    const Phase_Bitmask_T state = Phase_Bitmask(id);
// #ifdef PHASE_INDIVIDUAL_CHANNEL_CONTROL
    return (_PWM_ChannelMaskOf(&p_phase->PWM_A, state.A) | _PWM_ChannelMaskOf(&p_phase->PWM_B, state.B) | _PWM_ChannelMaskOf(&p_phase->PWM_C, state.C));
// #else
// return _PWM_Module_ChannelMaskOf(&p_phase->PWM_MODULE, state.Bits);
// #endif
}

static inline uint32_t _Phase_PinSyncOf(const Phase_T * p_phase, Phase_Id_T id)
{
    const Phase_Bitmask_T state = Phase_Bitmask(id);
    return (Pin_Module_MaskOf(&p_phase->PIN_A, state.A) | Pin_Module_MaskOf(&p_phase->PIN_B, state.B) | Pin_Module_MaskOf(&p_phase->PIN_C, state.C));
}

/*!

*/
/*
    Software Sync when counter up/down reload sync is not configured
*/
static inline void _Phase_SyncPwmDuty(const Phase_T * p_phase, Phase_Id_T state)
{
// #ifdef PHASE_SYNC_DUTY_UPDATE
    _PWM_Module_WriteSyncDuty(&p_phase->PWM_MODULE, _Phase_PwmSyncOf(p_phase, state));
// #endif
}

/* Sync activation of Switch and Invert Polarity */
/* Bit Reg operation need to be in single write for sync update */
static inline void _Phase_SyncPwmInvert(const Phase_T * p_phase, Phase_Id_T state)
{
#ifdef PHASE_SYNC_INVERT_UPDATE
    _PWM_Module_WriteSyncInvert(&p_phase->PWM_MODULE, _Phase_PwmSyncOf(p_phase, state));
#endif
}

static inline void _Phase_SyncOnOff(const Phase_T * p_phase, Phase_Id_T state)
{
#ifdef PHASE_PIN_SYNC /* caller enable when PINs are of the same module */
    _Pin_WriteSyncOnOff(&p_phase->PIN_A, _Phase_PinSyncOf(p_phase, state));
#endif
// #ifndef PHASE_PIN_SWITCH
//     _PWM_Module_WriteSyncOnOff(&p_phase->PWM_MODULE, _Phase_PwmSyncOf(p_phase, state));
// #endif
}

/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
/* Let the compiler optimize/expand into the 8 derived functions */
static inline void _Phase_WriteState(const Phase_T * p_phase, Phase_Id_T id)
{
#ifdef PHASE_PIN_SYNC
    _Phase_SyncOnOff(p_phase, id);
#else
    const Phase_Bitmask_T state = Phase_Bitmask(id);
    _Phase_WriteOnOffA(p_phase, state.A);
    _Phase_WriteOnOffB(p_phase, state.B);
    _Phase_WriteOnOffC(p_phase, state.C);
#endif
}

/* ReadChannelsOutput, ReadChannelSwitches */
static inline Phase_Bitmask_T _Phase_ReadState(const Phase_T * p_phase)
{
    return (Phase_Bitmask_T) { .A = _Phase_ReadOnOffA(p_phase), .B = _Phase_ReadOnOffB(p_phase), .C = _Phase_ReadOnOffC(p_phase) };
}

/* Voltage/Output State */
static inline Phase_Bitmask_T _Phase_ReadDutyState(const Phase_T * p_phase)
{
    return (Phase_Bitmask_T)
    {
        .A = (PWM_ReadDuty_Ticks(&p_phase->PWM_A) != 0U),
        .B = (PWM_ReadDuty_Ticks(&p_phase->PWM_B) != 0U),
        .C = (PWM_ReadDuty_Ticks(&p_phase->PWM_C) != 0U),
    };
}

/******************************************************************************/
/*  */
/******************************************************************************/
/* Using Register State */
static inline void Phase_WriteDuty_Fract16_Thread(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    Phase_Bitmask_T state = _Phase_ReadState(p_phase);

    if (state.A == 1U) { PWM_WriteDuty_Fract16(&p_phase->PWM_A, pwmDutyA); }
    if (state.B == 1U) { PWM_WriteDuty_Fract16(&p_phase->PWM_B, pwmDutyB); }
    if (state.C == 1U) { PWM_WriteDuty_Fract16(&p_phase->PWM_C, pwmDutyC); }
    if (state.Bits != PHASE_ID_0) { _Phase_SyncPwmDuty(p_phase, state.Bits); }
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
/*
    Duty 100% == PWM_DUTY_MAX
*/
static inline void Phase_WriteDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty(&p_phase->PWM_A, pwmDutyA);
    PWM_WriteDuty(&p_phase->PWM_B, pwmDutyB);
    PWM_WriteDuty(&p_phase->PWM_C, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

static inline void Phase_WriteDuty_Fract16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty_Fract16(&p_phase->PWM_A, pwmDutyA);
    PWM_WriteDuty_Fract16(&p_phase->PWM_B, pwmDutyB);
    PWM_WriteDuty_Fract16(&p_phase->PWM_C, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

static inline void Phase_WriteDuty_Percent16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty_Percent16(&p_phase->PWM_A, pwmDutyA);
    PWM_WriteDuty_Percent16(&p_phase->PWM_B, pwmDutyB);
    PWM_WriteDuty_Percent16(&p_phase->PWM_C, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase, PHASE_ID_ABC);
}

static inline void Phase_WriteDuty_Vector(const Phase_T * p_phase, Phase_Triplet_T * p_values)
{
    Phase_WriteDuty_Fract16(p_phase, p_values->A, p_values->B, p_values->C);
}

// static inline void Phase_Deactivate(const Phase_T * p_phase)
static inline void Phase_Float(const Phase_T * p_phase)
{
    // _Phase_WriteState(p_phase, PHASE_ID_0);
    _Phase_DisableA(p_phase);
    _Phase_DisableB(p_phase);
    _Phase_DisableC(p_phase);
}

// static inline void _Phase_Activate(const Phase_T * p_phase) active stored
static inline void Phase_ActivateOutput(const Phase_T * p_phase)
{
    // _Phase_WriteState(p_phase, PHASE_ID_ABC);
    _Phase_EnableA(p_phase);
    _Phase_EnableB(p_phase);
    _Phase_EnableC(p_phase);
}

/* Enable all at 0 duty. NOT for Bipolar active. */
static inline void Phase_ActivateOutputV0(const Phase_T * p_phase)
{
    // assert(p_phase->PolarMode != PHASE_MODE_BIPOLAR);
    Phase_WriteDuty(p_phase, 0U, 0U, 0U);
    Phase_ActivateOutput(p_phase);
}

// static inline void Phase_ActivateOutputVMidpoint(const Phase_T * p_phase)
static inline void Phase_ActivateOutputT0(const Phase_T * p_phase)
{
    Phase_WriteDuty_Fract16(p_phase, INT16_MAX / 2U, INT16_MAX / 2U, INT16_MAX / 2U);
    Phase_ActivateOutput(p_phase);
}

static inline bool Phase_IsFloat(const Phase_T * p_phase)
{
    return (_Phase_ReadState(p_phase).Bits == PHASE_ID_0);
}

static inline bool Phase_IsVDuty(const Phase_T * p_phase)
{
    // return !Phase_IsFloat(p_phase);
    return !Phase_IsFloat(p_phase) && (_Phase_ReadDutyState(p_phase).Bits != PHASE_ID_0);
}

static inline bool Phase_IsV0(const Phase_T * p_phase)
{
    // return ((_Phase_ReadState(p_phase).Bits == PHASE_ID_ABC) && (_Phase_ReadDutyState(p_phase).Bits == PHASE_ID_0)); //alternatively check active channels only
    return (!Phase_IsFloat(p_phase) && (_Phase_ReadDutyState(p_phase).Bits == PHASE_ID_0));
}

/* Collective state */
static inline Phase_Output_T Phase_ReadOutputState(const Phase_T * p_phase)
{
    Phase_Output_T state;

    switch (_Phase_ReadState(p_phase).Bits)
    {
        case PHASE_ID_0: state = PHASE_OUTPUT_FLOAT; break;
        //alternatively check active channels only
        // case PHASE_ID_ABC:  break;
        /* Including Polar Ouputs */
        default: state = (_Phase_ReadDutyState(p_phase).Bits == PHASE_ID_0) ? PHASE_OUTPUT_V0 : PHASE_OUTPUT_VPWM; break;
    }
    return state;
}

static inline void Phase_ActivateOutputState(const Phase_T * p_phase, Phase_Output_T state)
{
    switch (state)
    {
        case PHASE_OUTPUT_FLOAT:    Phase_Float(p_phase);               break;
        case PHASE_OUTPUT_V0:       Phase_ActivateOutputV0(p_phase);    break;
        case PHASE_OUTPUT_VPWM:     Phase_ActivateOutputT0(p_phase);    break;
        // case PHASE_OUTPUT_VPWM:     Phase_ActivateOutput(p_phase);      break; /* restore last active */
        default: break;
    }
}


/******************************************************************************/
/*!
    Extern
*/
/******************************************************************************/
extern void Phase_Init(const Phase_T * p_phase);

extern void Phase_Align(const Phase_T * p_phase, Phase_Id_T id, uint16_t duty);

extern Phase_Id_T Phase_ReadAlign(const Phase_T * p_phase);
extern Phase_Id_T Phase_ReadAlignNext(const Phase_T * p_phase);
extern Phase_Id_T Phase_ReadAlignPrev(const Phase_T * p_phase);

extern Phase_Id_T Phase_JogNext(const Phase_T * p_phase, uint16_t duty);
extern Phase_Id_T Phase_JogPrev(const Phase_T * p_phase, uint16_t duty);
extern Phase_Id_T Phase_JogSigned(const Phase_T * p_phase, int16_t dutySigned);