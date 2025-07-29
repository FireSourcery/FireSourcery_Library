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
    @file   MotVarId.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>


/******************************************************************************/
/*!
*/
/******************************************************************************/


/******************************************************************************/
/*!
*/
/******************************************************************************/
typedef union MotVarId
{
    struct
    {
        uint16_t Base           : 4U; /* Name - corresponds with enum index value */
        uint16_t InnerType      : 4U; /* Accessor. Corresponds with Base enum type, maybe n:1 handlers to enum type literal. */
        uint16_t OuterType      : 4U; /* Handler. InnerType's Type */
        uint16_t Instance       : 2U; /* Instance - Upto 8 Instances for each combination. */
        uint16_t Resv           : 2U; /* Motor/MotorController */
    };
    uint16_t Value;
}
MotVarId_T;


/*
    Status Response for Read/Write
*/
typedef enum MotVarId_Status
{
    MOT_VAR_STATUS_OK,
    MOT_VAR_STATUS_ERROR,
    MOT_VAR_STATUS_ERROR_INVALID_ID,
    MOT_VAR_STATUS_ERROR_READ_ONLY,
    MOT_VAR_STATUS_ERROR_WRITE_ONLY,
    MOT_VAR_STATUS_ERROR_ACCESS_DISABLED,
    MOT_VAR_STATUS_ERROR_NOT_CONFIG_STATE,
    MOT_VAR_STATUS_ERROR_NOT_RUNNING_STATE,
    MOT_VAR_STATUS_RESERVED = 0xFFU,
}
MotVarId_Status_T;

