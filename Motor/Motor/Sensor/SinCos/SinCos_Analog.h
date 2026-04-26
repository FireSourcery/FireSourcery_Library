#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   SinCos_Analog.h
    @author FireSourcery
    @brief  Analog conversion bundle for SinCos sensor inputs.
*/
/******************************************************************************/
#include "Peripheral/Analog/Analog.h"

#include <stdint.h>

/*
    [SinCos_Analog_T]
    Owns the two ADC conversion descriptors. Result buffers live inside each
    descriptor's channel state; SinCos_T points at those result fields.
*/
typedef const struct SinCos_Analog
{
    Analog_Conversion_T SIN;
    Analog_Conversion_T COS;
}
SinCos_Analog_T;

#define SIN_COS_ANALOG_INIT(AdcSin, IndexSin, AdcCos, IndexCos)         \
{                                                                       \
    .SIN = ANALOG_CONVERSION_INIT_FROM(AdcSin, IndexSin),               \
    .COS = ANALOG_CONVERSION_INIT_FROM(AdcCos, IndexCos),               \
}

static inline void SinCos_Analog_Mark(const SinCos_Analog_T * p_analog)
{
    Analog_Conversion_Mark(&p_analog->SIN);
    Analog_Conversion_Mark(&p_analog->COS);
}

static inline uint16_t SinCos_Analog_GetSin(const SinCos_Analog_T * p_analog) { return Analog_Conversion_GetResult(&p_analog->SIN); }
static inline uint16_t SinCos_Analog_GetCos(const SinCos_Analog_T * p_analog) { return Analog_Conversion_GetResult(&p_analog->COS); }
