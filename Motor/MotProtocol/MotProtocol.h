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
    @file   MotProtocol.h
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#ifndef MOT_PROTOCOL_H
#define MOT_PROTOCOL_H

#include "MotPacket.h"
#include "Framework/Protocol/Extension/Protocol_FlashLoader.h"
#include "Framework/Protocol/Protocol.h"
#include "Peripheral/NvMemory/Flash/Flash.h"

// #define MOT_PROTOCOL_BAUD_RATE_DEFAULT  (19200U)
// #define MOT_PROTOCOL_TIMEOUT_RX         (2000U)     /* Timeout Rx Packet */
// #define MOT_PROTOCOL_TIMEOUT_REQ        (5000U)     /* Timeout Req */


#define MOT_DATA_MODE_CHUNK_MAX ((packet_size_t)(MOT_PACKET_LENGTH_MAX - sizeof(MotPacket_Header_T)))

#define MOT_PROTOCOL_FLASH_LOADER(p_Flash) PROTOCOL_FLASH_LOADER(p_Flash, MOT_PACKET_DATA_MODE_DATA, MOT_DATA_MODE_CHUNK_MAX)


#endif

