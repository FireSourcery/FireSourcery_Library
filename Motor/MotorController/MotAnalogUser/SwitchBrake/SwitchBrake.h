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
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   SwitchBrake.h
    @author FireSourcery
    @brief  Digital "handbrake" augment. When the pin is active, the module
            reports a NVM-configured Percent16 brake floor that a policy layer
            can fuse (max) with the analog brake pedal value.

            Pure pin owner — no arbitration or drive-state awareness.
*/
/******************************************************************************/
#include "Transducer/UserIn/UserDIn.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/*
    Config
*/
/******************************************************************************/
typedef struct SwitchBrake_Config
{
    uint16_t Value_Percent16;   /* Brake value applied while pin is active, [0:65535] */
    bool  IsEnabled;    /* NVM runtime enable (SKU has pin but unused) */
}
SwitchBrake_Config_T;


typedef const struct
{
    const UserDIn_T DIN;
}
SwitchBrake_T;

/* Max-fuse helper for callers holding an analog brake value. */
static inline uint16_t SwitchBrake_FuseBrake(const SwitchBrake_T * p_sb, uint16_t analogBrake)
{
    uint16_t floor = SwitchBrake_GetValue(p_sb);
    return (floor > analogBrake) ? floor : analogBrake;
}