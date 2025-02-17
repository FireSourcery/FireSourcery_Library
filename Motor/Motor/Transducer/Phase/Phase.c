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
    @file   Phase.c
    @author FireSourcery
    @version V0
    @brief     Phase module conventional function definitions
*/
/******************************************************************************/
#include "Phase.h"

void Phase_Init(Phase_T * p_phase)
{
    PWM_InitModule(&p_phase->PwmModule);
    PWM_InitChannel(&p_phase->PwmA);
    PWM_InitChannel(&p_phase->PwmB);
    PWM_InitChannel(&p_phase->PwmC);
#ifdef CONFIG_PHASE_PIN_SWITCH
    Pin_Output_Init(&p_phase->PinA);
    Pin_Output_Init(&p_phase->PinB);
    Pin_Output_Init(&p_phase->PinC);
#endif
    p_phase->PhaseMode = PHASE_MODE_UNIPOLAR_1;
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
static inline void _Phase_SyncPwmDuty(const Phase_T * p_phase)
{
    PWM_ActuateSync(&p_phase->PwmModule); // optionally disable for sync triggers
}

/* Sync activation of Switch and Invert Polarity */
/* Bit Reg operation need to be in single write for sync update  */
// Need to PWM_module or HAL_Phase with context of all pwm channels for Sync on writes to 1 register
static inline void _Phase_SyncPwmSwitch(const Phase_T * p_phase)
{
#ifndef CONFIG_PHASE_PIN_SWITCH
    // PWM_ActuateSync(&p_phase->PwmModule);
    // PWM_Module_Enable(&p_phase->Pwm_Module, enumnstate[0,0,0]);
#endif
}

/* todo */
static inline void _Phase_SyncPwmInvert(const Phase_T * p_phase)
{
    // PWM_ActuateSync(&p_phase->PwmModule);
}

/******************************************************************************/
/*!
    2/3-Phase Polar common
*/
/*! @{ */
/******************************************************************************/
static inline void _Phase_EnablePwmSwitch(const PWM_T * p_pwm)
{
#ifndef CONFIG_PHASE_PIN_SWITCH
    PWM_Enable(p_pwm);
#endif
}

static inline void _Phase_DisablePwmSwitch(const PWM_T * p_pwm)
{
#ifndef CONFIG_PHASE_PIN_SWITCH
    PWM_Disable(p_pwm);
#endif
}

static inline void _Phase_EnablePinSwitch(const Pin_T * p_pin)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    Pin_Output_High(p_pin);
#endif
}

static inline void _Phase_DisablePinSwitch(const Pin_T * p_pin)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    Pin_Output_Low(p_pin);
#endif
}

static inline void _Phase_EnableA(const Phase_T * p_phase) { _Phase_EnablePwmSwitch(&p_phase->PwmA); _Phase_EnablePinSwitch(&p_phase->PinA); }
static inline void _Phase_EnableB(const Phase_T * p_phase) { _Phase_EnablePwmSwitch(&p_phase->PwmB); _Phase_EnablePinSwitch(&p_phase->PinB); }
static inline void _Phase_EnableC(const Phase_T * p_phase) { _Phase_EnablePwmSwitch(&p_phase->PwmC); _Phase_EnablePinSwitch(&p_phase->PinC); }
static inline void _Phase_DisableA(const Phase_T * p_phase) { _Phase_DisablePwmSwitch(&p_phase->PwmA); _Phase_DisablePinSwitch(&p_phase->PinA); }
static inline void _Phase_DisableB(const Phase_T * p_phase) { _Phase_DisablePwmSwitch(&p_phase->PwmB); _Phase_DisablePinSwitch(&p_phase->PinB); }
static inline void _Phase_DisableC(const Phase_T * p_phase) { _Phase_DisablePwmSwitch(&p_phase->PwmC); _Phase_DisablePinSwitch(&p_phase->PinC); }

static inline bool Phase_IsEnabledA(const Phase_T * p_phase)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    return Pin_Output_ReadPhysical(&p_phase->PinA);
#else
    return PWM_ReadOutputState(&p_phase->PwmA);
#endif
}

static inline bool Phase_IsEnabledB(const Phase_T * p_phase)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    return Pin_Output_ReadPhysical(&p_phase->PinB);
#else
    return PWM_ReadOutputState(&p_phase->PwmB);
#endif
}

static inline bool Phase_IsEnabledC(const Phase_T * p_phase)
{
#ifdef CONFIG_PHASE_PIN_SWITCH
    return Pin_Output_ReadPhysical(&p_phase->PinC);
#else
    return PWM_ReadOutputState(&p_phase->PwmC);
#endif
}

static inline bool Phase_IsEnabledABC(const Phase_T * p_phase)
{
    return ((Phase_IsEnabledA(p_phase) & Phase_IsEnabledB(p_phase) & Phase_IsEnabledC(p_phase)) != 0U);
}

static inline bool Phase_IsDisabledABC(const Phase_T * p_phase)
{
    return ((Phase_IsEnabledA(p_phase) | Phase_IsEnabledB(p_phase) | Phase_IsEnabledC(p_phase)) == 0U);
    // return (Phase_IsEnabledA(p_phase) == false) && (Phase_IsEnabledB(p_phase) == false) && (Phase_IsEnabledC(p_phase) == false);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!

*/
/******************************************************************************/
/*
    Duty 100% == CONFIG_PWM_DUTY_MAX
*/
void Phase_WriteDuty(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty(&p_phase->PwmA, pwmDutyA);
    PWM_WriteDuty(&p_phase->PwmB, pwmDutyB);
    PWM_WriteDuty(&p_phase->PwmC, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_WriteDuty_Fract16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty_Fract16(&p_phase->PwmA, pwmDutyA);
    PWM_WriteDuty_Fract16(&p_phase->PwmB, pwmDutyB);
    PWM_WriteDuty_Fract16(&p_phase->PwmC, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_WriteDuty_Percent16(const Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
    PWM_WriteDuty_Percent16(&p_phase->PwmA, pwmDutyA);
    PWM_WriteDuty_Percent16(&p_phase->PwmB, pwmDutyB);
    PWM_WriteDuty_Percent16(&p_phase->PwmC, pwmDutyC);
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_ActivateOutputABC(const Phase_T * p_phase)
{
    _Phase_EnableA(p_phase);
    _Phase_EnableB(p_phase);
    _Phase_EnableC(p_phase);
    _Phase_SyncPwmSwitch(p_phase);
}

void Phase_Float(const Phase_T * p_phase)
{
    _Phase_DisableA(p_phase);
    _Phase_DisableB(p_phase);
    _Phase_DisableC(p_phase);
    _Phase_SyncPwmSwitch(p_phase);
}

/* NOT for Bipolar active. Enable all at 0 duty */
void Phase_Ground(const Phase_T * p_phase)
{
    assert(p_phase->PhaseMode != PHASE_MODE_BIPOLAR);
    Phase_WriteDuty(p_phase, 0U, 0U, 0U);
    Phase_ActivateOutputABC(p_phase);
}

bool Phase_IsFloat(const Phase_T * p_phase)
{
    return Phase_IsDisabledABC(p_phase);
}

bool Phase_IsGround(const Phase_T * p_phase)
{
    return ((Phase_IsEnabledABC(p_phase) == true) && ((PWM_ReadDuty_Ticks(&p_phase->PwmA) == 0U) && (PWM_ReadDuty_Ticks(&p_phase->PwmB) == 0U) && (PWM_ReadDuty_Ticks(&p_phase->PwmC) == 0U)));
}


Phase_State_T Phase_ReadState(const Phase_T * p_phase)
{
    Phase_State_T state;
    if      (Phase_IsFloat(p_phase) == true)    { state = PHASE_STATE_FLOAT; }
    else if (Phase_IsGround(p_phase) == true)   { state = PHASE_STATE_GROUND; }
    else                                        { state = PHASE_STATE_ACTIVE; }
    return state;
}

void Phase_ActivateState(const Phase_T * p_phase, Phase_State_T state)
{
    switch (state)
    {
        case PHASE_STATE_FLOAT:     Phase_Float(p_phase); break;
        case PHASE_STATE_GROUND:    Phase_Ground(p_phase); break;
        case PHASE_STATE_ACTIVE:
            Phase_WriteDuty(p_phase, PWM_DUTY_MAX / 2, PWM_DUTY_MAX / 2, PWM_DUTY_MAX / 2);
            Phase_ActivateOutputABC(p_phase);
            break;
        default: break;
    }
}

// void Phase_ActivateStateDuty(const Phase_T * p_phase, Phase_State_T state, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
// {
//     switch (state)
//     {
//         case PHASE_STATE_FLOAT:     Phase_Float(p_phase); break;
//         case PHASE_STATE_GROUND:    Phase_Ground(p_phase); break;
//         case PHASE_STATE_ACTIVE:
//             Phase_WriteDuty(p_phase, pwmDutyA, pwmDutyB, pwmDutyC);
//             Phase_ActivateOutputABC(p_phase);
//             break;
//         default: break;
//     }
// }

/******************************************************************************/
/*
    3-Phase Polar
*/
/******************************************************************************/
void Phase_Align_ActivateA(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, duty, 0U, 0U); Phase_ActivateOutputABC(p_phase); }
void Phase_Align_ActivateB(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, duty, 0U); Phase_ActivateOutputABC(p_phase); }
void Phase_Align_ActivateC(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, 0U, duty); Phase_ActivateOutputABC(p_phase); }
void Phase_Align_ActivateInvA(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, 0U, duty, duty); Phase_ActivateOutputABC(p_phase); }
void Phase_Align_ActivateInvB(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty, 0U, duty); Phase_ActivateOutputABC(p_phase); }
void Phase_Align_ActivateInvC(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty, duty, 0U); Phase_ActivateOutputABC(p_phase); }

void Phase_Align_ActivateDutyA(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, duty, 0U, 0U); }
void Phase_Align_ActivateDutyB(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, duty, 0U); }
void Phase_Align_ActivateDutyC(const Phase_T * p_phase, uint16_t duty)     { Phase_WriteDuty(p_phase, 0U, 0U, duty); }
void Phase_Align_ActivateDutyInvA(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, 0U, duty, duty); }
void Phase_Align_ActivateDutyInvB(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty, 0U, duty); }
void Phase_Align_ActivateDutyInvC(const Phase_T * p_phase, uint16_t duty)  { Phase_WriteDuty(p_phase, duty, duty, 0U); }

void Phase_Align_ActivateDuty(const Phase_T * p_phase, Phase_Align_T id, uint16_t duty)
{
    switch (id)
    {
        case PHASE_ID_A:        Phase_Align_ActivateDutyA(p_phase, duty);       break;
        case PHASE_ID_INV_C:    Phase_Align_ActivateDutyInvC(p_phase, duty);    break;
        case PHASE_ID_B:        Phase_Align_ActivateDutyB(p_phase, duty);       break;
        case PHASE_ID_INV_A:    Phase_Align_ActivateDutyInvA(p_phase, duty);    break;
        case PHASE_ID_C:        Phase_Align_ActivateDutyC(p_phase, duty);       break;
        case PHASE_ID_INV_B:    Phase_Align_ActivateDutyInvB(p_phase, duty);    break;
        default: break;
    }
}

Phase_Flags_T _Phase_ReadOutputState(const Phase_T * p_phase)
{
    return (Phase_Flags_T) { .A = (PWM_ReadDuty_Ticks(&p_phase->PwmA) != 0U), .B = (PWM_ReadDuty_Ticks(&p_phase->PwmB) != 0U), .C = (PWM_ReadDuty_Ticks(&p_phase->PwmC) != 0U) };
}

Phase_Align_T Phase_ReadAlign(const Phase_T * p_phase)
{
    //  return _Phase_ReadOutputState(p_phase).Value; //dodo as index
}

Phase_Align_T Phase_GetNextAlign(const Phase_T * p_phase)
{
    switch (Phase_ReadAlign(p_phase))
    {
        case PHASE_ID_A:        return PHASE_ID_INV_C;
        case PHASE_ID_INV_C:    return PHASE_ID_B;
        case PHASE_ID_B:        return PHASE_ID_INV_A;
        case PHASE_ID_INV_A:    return PHASE_ID_C;
        case PHASE_ID_C:        return PHASE_ID_INV_B;
        case PHASE_ID_INV_B:    return PHASE_ID_A;
        default: break;
    }
}


/******************************************************************************/
/*! */
/******************************************************************************/

/******************************************************************************/
/*!
    2-Phase Polar PWM
    if Pwm is updated every cycle, no need to use buffered write
*/
/*! @{ */
/******************************************************************************/
/* Enable all at 0 duty */
void Phase_Polar_Ground(const Phase_T * p_phase)
{
    if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR)
    {
        Phase_Float(p_phase);
        PWM_DisableInvertPolarity(&p_phase->PwmA);
        PWM_DisableInvertPolarity(&p_phase->PwmB);
        PWM_DisableInvertPolarity(&p_phase->PwmC);
    }

    Phase_Ground(p_phase);
}

static void _Phase_ActivateOutputNotA(const Phase_T * p_phase) { _Phase_DisableA(p_phase); _Phase_EnableB(p_phase); _Phase_EnableC(p_phase); _Phase_SyncPwmSwitch(p_phase); }
static void _Phase_ActivateOutputNotB(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_DisableB(p_phase); _Phase_EnableC(p_phase); _Phase_SyncPwmSwitch(p_phase); }
static void _Phase_ActivateOutputNotC(const Phase_T * p_phase) { _Phase_EnableA(p_phase); _Phase_EnableB(p_phase); _Phase_DisableC(p_phase); _Phase_SyncPwmSwitch(p_phase); }

/*
    For Bipolar Pwm
*/
static void _Phase_ActivateInvertPolarityA(const Phase_T * p_phase) { PWM_EnableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC); _Phase_SyncPwmInvert(p_phase); }
static void _Phase_ActivateInvertPolarityB(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_EnableInvertPolarity(&p_phase->PwmB); PWM_DisableInvertPolarity(&p_phase->PwmC); _Phase_SyncPwmInvert(p_phase); }
static void _Phase_ActivateInvertPolarityC(const Phase_T * p_phase) { PWM_DisableInvertPolarity(&p_phase->PwmA); PWM_DisableInvertPolarity(&p_phase->PwmB); PWM_EnableInvertPolarity(&p_phase->PwmC); _Phase_SyncPwmInvert(p_phase); }

/*
    Activate Switches Only
*/
void Phase_Polar_ActivateOutputAC(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
void Phase_Polar_ActivateOutputBC(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityC(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
void Phase_Polar_ActivateOutputBA(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotC(p_phase); }
void Phase_Polar_ActivateOutputCA(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityA(p_phase); } _Phase_ActivateOutputNotB(p_phase); }
void Phase_Polar_ActivateOutputCB(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotA(p_phase); }
void Phase_Polar_ActivateOutputAB(const Phase_T * p_phase) { if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) { Phase_Float(p_phase); _Phase_ActivateInvertPolarityB(p_phase); } _Phase_ActivateOutputNotC(p_phase); }

/*
    Activate Duty Only
*/
void Phase_Unipolar1_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmA, duty); PWM_WriteDuty(&p_phase->PwmC, 0U); }
void Phase_Unipolar1_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmB, duty); PWM_WriteDuty(&p_phase->PwmC, 0U); }
void Phase_Unipolar1_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmB, duty); PWM_WriteDuty(&p_phase->PwmA, 0U); }
void Phase_Unipolar1_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmC, duty); PWM_WriteDuty(&p_phase->PwmA, 0U); }
void Phase_Unipolar1_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmC, duty); PWM_WriteDuty(&p_phase->PwmB, 0U); }
void Phase_Unipolar1_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_WriteDuty(&p_phase->PwmA, duty); PWM_WriteDuty(&p_phase->PwmB, 0U); }

/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 - PwmScalar/2
*/
void Phase_Unipolar2_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
void Phase_Unipolar2_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmC, duty); }
void Phase_Unipolar2_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
void Phase_Unipolar2_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmA, duty); }
void Phase_Unipolar2_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }
void Phase_Unipolar2_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidMinus(&p_phase->PwmB, duty); }

/*
    PwmPositive = PwmPeriodTotal/2 + PwmScalar/2
    PwmNegative = PwmPeriodTotal/2 + PwmScalar/2, Inverse polarity
*/
void Phase_Bipolar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
void Phase_Bipolar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); }
void Phase_Bipolar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
void Phase_Bipolar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); }
void Phase_Bipolar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmC, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }
void Phase_Bipolar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty) { PWM_ActuateDutyMidPlus(&p_phase->PwmA, duty); PWM_ActuateDutyMidPlus(&p_phase->PwmB, duty); }

void Phase_Polar_ActivateDutyAC(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PhaseMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyAC(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyAC(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyAC(p_phase, duty);    break;
        default: break;
    }
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_Polar_ActivateDutyBC(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PhaseMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyBC(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyBC(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyBC(p_phase, duty);    break;
        default: break;
    }
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_Polar_ActivateDutyBA(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PhaseMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyBA(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyBA(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyBA(p_phase, duty);    break;
        default: break;
    }
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_Polar_ActivateDutyCA(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PhaseMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyCA(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyCA(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyCA(p_phase, duty);    break;
        default: break;
    }
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_Polar_ActivateDutyCB(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PhaseMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyCB(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyCB(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyCB(p_phase, duty);    break;
        default: break;
    }
    _Phase_SyncPwmDuty(p_phase);
}

void Phase_Polar_ActivateDutyAB(const Phase_T * p_phase, uint16_t duty)
{
    switch(p_phase->PhaseMode)
    {
        case PHASE_MODE_UNIPOLAR_1: Phase_Unipolar1_ActivateDutyAB(p_phase, duty);    break;
        case PHASE_MODE_UNIPOLAR_2: Phase_Unipolar2_ActivateDutyAB(p_phase, duty);    break;
        case PHASE_MODE_BIPOLAR:    Phase_Bipolar_ActivateDutyAB(p_phase, duty);    break;
        default: break;
    }
    _Phase_SyncPwmDuty(p_phase);
}

/*
    Activate Duty and Sets On/Off State
*/
void Phase_Polar_ActivateAC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAC(p_phase, duty); Phase_Polar_ActivateOutputAC(p_phase); }
void Phase_Polar_ActivateBC(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBC(p_phase, duty); Phase_Polar_ActivateOutputBC(p_phase); }
void Phase_Polar_ActivateBA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyBA(p_phase, duty); Phase_Polar_ActivateOutputBA(p_phase); }
void Phase_Polar_ActivateCA(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCA(p_phase, duty); Phase_Polar_ActivateOutputCA(p_phase); }
void Phase_Polar_ActivateCB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyCB(p_phase, duty); Phase_Polar_ActivateOutputCB(p_phase); }
void Phase_Polar_ActivateAB(const Phase_T * p_phase, uint16_t duty) { Phase_Polar_ActivateDutyAB(p_phase, duty); Phase_Polar_ActivateOutputAB(p_phase); }
/******************************************************************************/
/*! @} */
/******************************************************************************/


/******************************************************************************/
/*!

*/
/******************************************************************************/
void Phase_Polar_ActivateMode(Phase_T * p_phase, Phase_Mode_T phaseMode)
{
    if(p_phase->PhaseMode == PHASE_MODE_BIPOLAR) /* If current phase mode is Bipolar, disable */
    {
        Phase_Float(p_phase);
        PWM_DisableInvertPolarity(&p_phase->PwmA);
        PWM_DisableInvertPolarity(&p_phase->PwmB);
        PWM_DisableInvertPolarity(&p_phase->PwmC);
    }
    p_phase->PhaseMode = phaseMode;
}

/*
    Commutation Phase Id handled by Phase module
    switch should be faster than CommutationTable[phaseID][p_phase->PhaseMode]();
*/
void Phase_Polar_Activate(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch(phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Polar_ActivateAC(p_phase, duty); break;
        case PHASE_ID_2_BC: Phase_Polar_ActivateBC(p_phase, duty); break;
        case PHASE_ID_3_BA: Phase_Polar_ActivateBA(p_phase, duty); break;
        case PHASE_ID_4_CA: Phase_Polar_ActivateCA(p_phase, duty); break;
        case PHASE_ID_5_CB: Phase_Polar_ActivateCB(p_phase, duty); break;
        case PHASE_ID_6_AB: Phase_Polar_ActivateAB(p_phase, duty); break;
        case PHASE_ID_7: break;
        default: break;
    }
}

void Phase_Polar_ActivateDuty(Phase_T * p_phase, Phase_Polar_T phaseId, uint16_t duty)
{
    switch(phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Polar_ActivateDutyAC(p_phase, duty); break;
        case PHASE_ID_2_BC: Phase_Polar_ActivateDutyBC(p_phase, duty); break;
        case PHASE_ID_3_BA: Phase_Polar_ActivateDutyBA(p_phase, duty); break;
        case PHASE_ID_4_CA: Phase_Polar_ActivateDutyCA(p_phase, duty); break;
        case PHASE_ID_5_CB: Phase_Polar_ActivateDutyCB(p_phase, duty); break;
        case PHASE_ID_6_AB: Phase_Polar_ActivateDutyAB(p_phase, duty); break;
        case PHASE_ID_7: break;
        default: break;
    }
}

void Phase_Polar_ActivateOutput(Phase_T * p_phase, Phase_Polar_T phaseId)
{
    switch(phaseId)
    {
        case PHASE_ID_0: break;
        case PHASE_ID_1_AC: Phase_Polar_ActivateOutputAC(p_phase); break;
        case PHASE_ID_2_BC: Phase_Polar_ActivateOutputBC(p_phase); break;
        case PHASE_ID_3_BA: Phase_Polar_ActivateOutputBA(p_phase); break;
        case PHASE_ID_4_CA: Phase_Polar_ActivateOutputCA(p_phase); break;
        case PHASE_ID_5_CB: Phase_Polar_ActivateOutputCB(p_phase); break;
        case PHASE_ID_6_AB: Phase_Polar_ActivateOutputAB(p_phase); break;
        case PHASE_ID_7: break;
        default: break;
    }
}
