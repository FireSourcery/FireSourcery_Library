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
    @file   Cia402_Pdo.c
    @author FireSourcery
    @brief  This node's PDO channels
*/
/******************************************************************************/
#include "Cia402_Pdo.h"
#include "Cia402.h"


/*
    Power-on channels: 1 and 2 carry [Cia402_RxPdo_Control_T], [Cia402_RxPdo_ControlVelocity_T], [Cia402_TxPdo_Status_T],
    [Cia402_TxPdo_StatusVelocity_T]; 3 and 4 are left for a master to map. What [Cia402_Pdo_InitFrom] takes for a stored channel
    that is not well-formed.
*/
static constexpr Cia402_PdoConfig_T CIA402_PDO_CONFIG_DEFAULT =
{
    .Rx =
    {
        { .CobId = { .CanId = COB_RXPDO1_BASE }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE, .MapCount = 1U,
          .Map = { PDO_MAP_ENTRY(CIA402_OD_CONTROLWORD, 0U, uint16_t) } },
        { .CobId = { .CanId = COB_RXPDO2_BASE }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE, .MapCount = 2U,
          .Map = { PDO_MAP_ENTRY(CIA402_OD_CONTROLWORD, 0U, uint16_t), PDO_MAP_ENTRY(CIA402_OD_TARGET_VELOCITY, 0U, int32_t) } },
        { .CobId = { .CanId = COB_RXPDO3_BASE, .Invalid = 1U }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE },
        { .CobId = { .CanId = COB_RXPDO4_BASE, .Invalid = 1U }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE },
    },
    .Tx =
    {
        { .CobId = { .CanId = COB_TXPDO1_BASE }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE, .EventTimer = 10U, .MapCount = 1U,
          .Map = { PDO_MAP_ENTRY(CIA402_OD_STATUSWORD, 0U, uint16_t) } },
        { .CobId = { .CanId = COB_TXPDO2_BASE }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE, .EventTimer = 100U, .MapCount = 2U,
          .Map = { PDO_MAP_ENTRY(CIA402_OD_STATUSWORD, 0U, uint16_t), PDO_MAP_ENTRY(CIA402_OD_VELOCITY_ACTUAL, 0U, int32_t) } },
        { .CobId = { .CanId = COB_TXPDO3_BASE, .Invalid = 1U }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE },
        { .CobId = { .CanId = COB_TXPDO4_BASE, .Invalid = 1U }, .Transmission = PDO_TRANSMISSION_EVENT_PROFILE },
    },
};

#if defined(CIA402_PDO_USE_DEFAULT_CONFIG)
#define _CIA402_PDO_CONFIG_DEFAULT CIA402_PDO_CONFIG_DEFAULT
#else
#define _CIA402_PDO_CONFIG_DEFAULT {}
#endif

Cia402_PdoConfig_T Cia402_PdoConfig = _CIA402_PDO_CONFIG_DEFAULT;

/*
    Constant Table
*/
PDO_Tables_T CIA402_PDO_TABLES =
{
    .RX = { .P_CHANNELS = Cia402_PdoConfig.Rx, .COUNT = CIA402_PDO_COUNT },
    .TX = { .P_CHANNELS = Cia402_PdoConfig.Tx, .COUNT = CIA402_PDO_COUNT },
};

/* Each stored channel if well-formed, else its default — erased flash, an image from another layout. */
void Cia402_Pdo_InitFrom(const Cia402_PdoConfig_T * p_config)
{
    for (uint8_t n = 0U; n < CIA402_PDO_COUNT; n++)
    {
        Cia402_PdoConfig.Rx[n] = PDO_Channel_IsWellFormed(&p_config->Rx[n]) ? p_config->Rx[n] : CIA402_PDO_CONFIG_DEFAULT.Rx[n];
        Cia402_PdoConfig.Tx[n] = PDO_Channel_IsWellFormed(&p_config->Tx[n]) ? p_config->Tx[n] : CIA402_PDO_CONFIG_DEFAULT.Tx[n];
    }
}
