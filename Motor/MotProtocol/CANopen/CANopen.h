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
    @file   CANopen.h
    @author FireSourcery
    @brief  CANopen communication objects — the COB-ID scheme
*/
/******************************************************************************/

/******************************************************************************/
/*
    COB-ID — the 11-bit CAN ID of a CANopen message: a 4-bit function code over a 7-bit node ID.
    Bases of the predefined connection set; + node addresses one node.
*/
/******************************************************************************/
#define COB_SDO_REQ_BASE     (0x600U) /* (master → slave) */
#define COB_SDO_RSP_BASE     (0x580U) /* (slave → master) */
#define COB_RXPDO1_BASE      (0x200U)
#define COB_RXPDO2_BASE      (0x300U)
#define COB_RXPDO3_BASE      (0x400U)
#define COB_RXPDO4_BASE      (0x500U)
#define COB_TXPDO1_BASE      (0x180U)
#define COB_TXPDO2_BASE      (0x280U)
#define COB_TXPDO3_BASE      (0x380U)
#define COB_TXPDO4_BASE      (0x480U)
#define COB_EMCY_BASE        (0x080U)

#define COB_FUNCTION_MASK    (0x780U) /* upper 4 bits */
#define COB_NODE_MASK        (0x07FU) /* lower 7 bits */
#define COB_MASK             (COB_FUNCTION_MASK | COB_NODE_MASK)

#define COB_FUNCTION(cob)    ((cob) & COB_FUNCTION_MASK)
#define COB_NODE(cob)        ((cob) & COB_NODE_MASK)

typedef enum Cob_FunctionCode
{
    COB_SDO_REQ   = 0x600U,
    COB_SDO_RSP   = 0x580U,
    COB_RXPDO1    = 0x200U,
    COB_RXPDO2    = 0x300U,
    COB_RXPDO3    = 0x400U,
    COB_RXPDO4    = 0x500U,
    COB_TXPDO1    = 0x180U,
    COB_TXPDO2    = 0x280U,
    COB_TXPDO3    = 0x380U,
    COB_TXPDO4    = 0x480U,
    COB_EMCY      = 0x080U,
}
Cob_FunctionCode_T;

typedef struct Cob
{
    uint16_t Node     : 7; /* lower 7 bits of COB-ID (node ID) */
    uint16_t Function : 4; /* upper 4 bits of COB-ID (function code) */
    uint16_t Reserved : 5; /* upper bits reserved, always 0 */
}
Cob_T;

