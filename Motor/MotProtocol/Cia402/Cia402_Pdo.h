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
    @file   Cia402_Pdo.h
    @author FireSourcery
    @brief  This drive's PDO set — CiA 402 defaults over [PDO_Channel_T]
*/
/******************************************************************************/
#include "Motor/MotProtocol/CANopen/PDO.h"


/******************************************************************************/
/*
    This node's PDOs — one set, four each way as in the predefined connection set. RAM starts at the defaults;
    [Cia402_Pdo_InitFrom] loads a stored copy over them, and the app's NVM map saves them back.
*/
/******************************************************************************/
#define CIA402_PDO_COUNT                (4U)        /* per direction */

typedef struct Cia402_PdoConfig
{
    PDO_Channel_T Rx[CIA402_PDO_COUNT];   /* RPDO1..4 */
    PDO_Channel_T Tx[CIA402_PDO_COUNT];   /* TPDO1..4 */
}
Cia402_PdoConfig_T;


extern Cia402_PdoConfig_T Cia402_PdoConfig;     /* RAM, named for the app's NVM map */
extern PDO_Tables_T CIA402_PDO_TABLES;          /* over Cia402_PdoConfig — what the handlers and parameter objects take */

extern void Cia402_Pdo_InitFrom(const Cia402_PdoConfig_T * p_config);
