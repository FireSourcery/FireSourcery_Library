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
    @file   CAN_Frame.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

/* Part of HAL_CAN */

/*
    SocketCAN convention
*/
/* Flag bits in upper 3 bits of Id, following SocketCAN convention */
#define CAN_ID_FLAG_EXT     0x80000000U
#define CAN_ID_FLAG_RTR     0x40000000U
#define CAN_ID_FLAG_ERR     0x20000000U
#define CAN_ID_MASK_EXT     0x1FFFFFFFU  /* 29-bit */
#define CAN_ID_MASK_STD     0x000007FFU  /* 11-bit */

typedef union
{
    uint32_t Id32;
    struct
    {
        uint32_t Id  : 29;
        uint32_t Err : 1;
        uint32_t Rtr : 1;
        uint32_t Eff : 1;
    };
}
can_id_t;

typedef struct __attribute__((packed))
{
    can_id_t CanId;
    uint8_t DataLength;
    uint8_t Opt;
    uint8_t Resv0;
    uint8_t Resv1;
    uint8_t Data[8];
}
CAN_Frame_T;