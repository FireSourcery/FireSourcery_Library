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
#ifndef HAL_EEPROM_PLATFORM_H
#define HAL_EEPROM_PLATFORM_H

#include "HAL_Flash.h" /* Includes common defs */
#include "Peripheral/NvMemory/NvMemory/Config.h"

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

/* flash_flexRam_function_control_code */
#define EEE_ENABLE                              (0x00U)    /*!< Make FlexRAM available for emulated EEPROM */
#define EEE_QUICK_WRITE                         (0x55U)    /*!< Make FlexRAM available for EEPROM quick writes */
#define EEE_STATUS_QUERY                        (0x77U)    /*!< EEPROM quick write status query */
#define EEE_COMPLETE_INTERRUPT_QUICK_WRITE      (0xAAU)    /*!< Complete interrupted EEPROM quick write process */
#define EEE_DISABLE                             (0xFFU)    /*!< Make FlexRAM available as RAM */

#define S32K_EEERAMSIZE_CODE                     (0x02U)     /* Only option for S32K142 */
#define CONFIG_HAL_EEPROM_S32K_DEPART_CODE         (0x08U)     /* Recommenced for max endurance, set once */

#define HAL_EEPROM_START            S32K_FLEX_RAM_START
#define HAL_EEPROM_END                S32K_FLEX_RAM_END
#define HAL_EEPROM_SIZE                S32K_FLEX_RAM_SIZE
#define HAL_EEPROM_UNIT_WRITE_SIZE    4U

typedef FTFC_Type HAL_EEPROM_T;     /* Flash/EEPROM use same controller */

static inline bool HAL_EEPROM_ReadCompleteFlag(HAL_EEPROM_T * p_hal)
{
    (void)p_hal;
    return ((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK) != 0U) ? true : false;
}

static inline bool HAL_EEPROM_ReadErrorFlags(HAL_EEPROM_T * p_hal)
{
    return HAL_Flash_ReadErrorFlags(p_hal);
}

static inline void HAL_EEPROM_ClearErrorFlags(HAL_EEPROM_T * p_hal)
{
    return HAL_Flash_ClearErrorFlags(p_hal);
}

static inline bool HAL_EEPROM_ReadErrorProtectionFlag(HAL_EEPROM_T * p_hal)
{
    return HAL_Flash_ReadErrorProtectionFlag(p_hal);
}

static inline void HAL_EEPROM_StartCmdWriteUnit(HAL_EEPROM_T * p_hal, const uint8_t * p_dest, const uint8_t * p_data)
{
    (void)p_hal;
#if (HAL_EEPROM_UNIT_WRITE_SIZE == 4U)
    (*(uint32_t *)p_dest) = (*(uint32_t *)p_data);     /*  cast away const for eeprom case, it is in FlexRam */
#elif (HAL_EEPROM_UNIT_WRITE_SIZE == 2U)
    (*(uint16_t*)p_dest) = (*(uint16_t*)p_data);
#elif (HAL_EEPROM_UNIT_WRITE_SIZE == 1U)
    (*(uint8_t*)p_dest) = (*(uint8_t*)p_data);
#endif
}

static inline bool HAL_EEPROM_ReadIsFirstTime(HAL_EEPROM_T * p_hal)
{
    (void)p_hal;
    uint32_t regDEPartitionCode = ((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK) >> SIM_FCFG1_DEPART_SHIFT);
    return (regDEPartitionCode != CONFIG_HAL_EEPROM_S32K_DEPART_CODE);
}

/*
    EEE SRAM data size.
    0000b - Reserved
    0001b - Reserved
    0010b - 4 KB
    0011b - 2 KB
    0100b - 1 KB
    0101b - 512 Bytes
    0110b - 256 Bytes
    0111b - 128 Bytes
    1000b - 64 Bytes
    1001b - 32 Bytes
    1111b - 0 Bytes

    In order to achieve specified w/e cycle endurance,
    the emulated EEPROM backup size must be at least 16 times the
    emulated EEPROM partition size in FlexRAM.
*/
/*
    Launch once if not programmed
*/
//static inline void HAL_EEPROM_ProgramPartition(HAL_EEPROM_T * p_hal)     CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static inline void HalEepromProgramPartition(void)
{
    uint32_t regDEPartitionCode = ((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK) >> SIM_FCFG1_DEPART_SHIFT);

    if(regDEPartitionCode != CONFIG_HAL_EEPROM_S32K_DEPART_CODE)
    {
        if(HAL_Flash_ReadCompleteFlag(0) == true)
        {
            HAL_Flash_ClearErrorFlags(0);

            FTFx_FCCOB0 = FTFx_PROGRAM_PARTITION;
            FTFx_FCCOB1 = 0x00U; //    CSEcKeySize;
            FTFx_FCCOB2 = 0x00U; //(uint8_t)(SFE ? 1U : 0U);
            FTFx_FCCOB3 = 0x01U; //(uint8_t)(flexRamEnableLoadEEEData ? 0U : 1U);
            FTFx_FCCOB4 = S32K_EEERAMSIZE_CODE;                    //EEEDataSizeCode = 0x02u: EEPROM size = 4 Kbytes
            FTFx_FCCOB5 = CONFIG_HAL_EEPROM_S32K_DEPART_CODE;     //DEPartitionCode = 0x08u: EEPROM backup size = 64 Kbytes */
            _HAL_Flash_LaunchCmd(0);

            while(HAL_Flash_ReadCompleteFlag(0) == false)
            {
                if(HAL_Flash_ReadErrorFlags(0) == false)
                {
                    break;
                }
            }
            //auto reboot?
        }
    }
}

static void HalEepromInitBlocking(void) CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION; //do not inline to force RAM copy
static void HalEepromInitBlocking(void)
{
#ifdef CONFIG_EEPROM_ONE_TIME_PROGRAM_PARTITION
    /* one time code may not be needed if operation is support by flash tool */
    HalEepromProgramPartition();
#endif

    /* Config FlexRam as emulated EEPROM */
    if(HAL_EEPROM_ReadCompleteFlag(0) == false)
    {
        if(HAL_Flash_ReadCompleteFlag(0) == true)
        {
            HAL_Flash_ClearErrorFlags(0);

            FTFx_FCCOB0 = FTFx_SET_EERAM;
            FTFx_FCCOB1 = (uint8_t)EEE_ENABLE;
            FTFx_FCCOB4 = (uint8_t)(0x00U);
            FTFx_FCCOB5 = (uint8_t)(0x00U);
            _HAL_Flash_LaunchCmd(0);
        }

        while(HAL_Flash_ReadCompleteFlag(0) == false)
        {
            if(HAL_Flash_ReadErrorFlags(0) == false)
            {
                break;
            }
        }
    }
}

//static inline void HAL_EEPROM_Init_Blocking(HAL_EEPROM_T * p_hal)         CONFIG_NV_MEMORY_ATTRIBUTE_RAM_SECTION;
static inline void HAL_EEPROM_Init_Blocking(HAL_EEPROM_T * p_hal)
{
    (void)p_hal;
    HalEepromInitBlocking();
}

#endif
