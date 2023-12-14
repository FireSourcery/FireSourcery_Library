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
    @file
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HAL_FLASH_H
#define HAL_FLASH_H

#include "Config.h"

#include "Peripheral/HAL/HAL_Peripheral.h"
#include HAL_PERIPHERAL_PATH(HAL_Flash.h)

// extern bool HAL_Flash_ReadCompleteFlag(const HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
// extern bool HAL_Flash_ReadErrorFlags(const HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
// extern void HAL_Flash_ClearErrorFlags(HAL_Flash_T * p_regs);
// extern bool HAL_Flash_ReadErrorVerifyFlag(const HAL_Flash_T * p_regs);
// extern bool HAL_Flash_ReadErrorProtectionFlag(const HAL_Flash_T * p_regs);
// extern bool HAL_Flash_ReadSecurityFlag(const HAL_Flash_T * p_regs);

// extern void HAL_Flash_StartCmdWritePage(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data);
// extern void HAL_Flash_StartCmdEraseSector(HAL_Flash_T * p_regs, const uint8_t * p_dest);
// extern void HAL_Flash_StartCmdEraseAll(HAL_Flash_T * p_regs);
// extern void HAL_Flash_StartCmdVerifyWriteUnit(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data);
// extern void HAL_Flash_StartCmdVerifyEraseUnit(HAL_Flash_T * p_regs, const uint8_t * p_dest);
// extern void HAL_Flash_StartCmdVerifyEraseUnits(HAL_Flash_T * p_regs, const uint8_t * p_dest, uint16_t units);
// extern void HAL_Flash_StartCmdWriteOnce(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data);
// extern void HAL_Flash_StartCmdReadOnce(HAL_Flash_T * p_regs, const uint8_t * p_dest);

// extern void HAL_Flash_ReadOnceData(HAL_Flash_T * p_regs, uint8_t * p_result);
// extern void HAL_Flash_UnlockSecurity(HAL_Flash_T * p_regs, uint8_t * p_key);
// extern void HAL_Flash_Init(HAL_Flash_T * p_regs);

#endif /* HAL_FLASH_H */
