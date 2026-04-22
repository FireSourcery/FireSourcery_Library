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
    @file   _CAN.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>

/*
    CAN Protocol (ISO 11898)
    ArbitrationID   | 11 (standard) or 29 (extended) The actual identifier value
    IDE 1           | Distinguishes standard vs extended
    RTR 1           | Remote request
    DLC 4           | 0-8 for CAN 2.0
    Data 0-8 bytes  | Payload

    Flex Data Rate (CAN FD) adds:
    FDF/EDL 1   | FD format indicator
    BRS 1       | Bit rate switch
    ESI 1       | Error state indicator
    DLC 9-15    | 12/16/20/24/32/48/64 bytes
*/
typedef struct
{
    uint32_t Id  : 11;
    uint32_t Rtr : 1;
    uint32_t Ide : 1;
    uint32_t R0 : 1;
}
CAN_StandardWireId_T;

typedef struct
{
    uint32_t Id_A   : 11;
    uint32_t Srr   : 1;
    uint32_t Ide   : 1;
    uint32_t Id_B   : 18;
    uint32_t Rtr   : 1;
    uint32_t R0    : 1;
    uint32_t R1    : 1;
}
CAN_ExtendedWireId_T;

/* parsable */
typedef enum
{
    CAN_FRAME_TYPE_DATA,        //    Data frame: a frame containing node data for transmission
    CAN_FRAME_TYPE_REMOTE,      //    Remote frame: a frame requesting the transmission of a specific identifier
    CAN_FRAME_TYPE_ERROR,       //    Error frame: a frame transmitted by any node detecting an error
    CAN_FRAME_TYPE_OVERLOAD,    //    Overload frame: a frame to inject a delay between data or remote frame
}
CAN_FrameType_T;
