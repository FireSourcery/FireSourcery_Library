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
#ifndef HAL_FLASH_PLATFORM_H
#define HAL_FLASH_PLATFORM_H

#include "KE0x.h"

/*
    Use CONFIG_FLASH_ATTRIBUTE_RAM_SECTION incase functions are not inlined (and stored in ram with calling function)
*/
#include "Peripheral/NvMemory/Flash/Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/*
    BUSCLK frequency(MHz) FDIV[5:0]
    20.6 21.6 0x14
    21.6 22.6 0x15
    22.6 23.6 0x16
    23.6 24.6 0x17
    24.6 25.6 0x18

    Overwrite when CPU_FREQ is not BUSCLK Freq
*/
#ifndef HAL_FLASH_CLOCK_SOURCE_FREQ
#define HAL_FLASH_CLOCK_SOURCE_FREQ (CPU_FREQ / 2UL)
#endif


#define KE0x_FLASH_RESERVED_START                   0x0000UL    /* */
#define KE0x_FLASH_RESERVED_END                     0x040FUL    /* */
#define KE0x_FLASH_RESERVED_IFR_START               0x0000UL    /* */
#define KE0x_FLASH_RESERVED_IFR_END                 0x03FFUL    /* */
#define KE0x_FLASH_RESERVED_PROGRAM_ONCE_START      0x03C0UL
#define KE0x_FLASH_RESERVED_PROGRAM_ONCE_END        0x03FFUL
#define KE0x_FLASH_RESERVED_FLEX_NVM_CODE           0x03FCUL
#define KE0x_FLASH_RESERVED_EEPROM_SIZE             0x03FDUL
#define KE0x_FLASH_RESERVED_CONFIG_START            0x0400UL    /* */
#define KE0x_FLASH_RESERVED_CONFIG_END              0x040FUL    /* */
#define KE0x_FLASH_RESERVED_BACKDOOR_KEY_START      0x0400UL    /* 8 bytes */
#define KE0x_FLASH_RESERVED_BACKDOOR_KEY_END        0x0407UL    /* 8 bytes */
#define KE0x_FLASH_RESERVED_FPROT3                  0x0408UL
#define KE0x_FLASH_RESERVED_FPROT2                  0x0409UL
#define KE0x_FLASH_RESERVED_FPROT1                  0x040AUL
#define KE0x_FLASH_RESERVED_FPROT0                  0x040BUL
#define KE0x_FLASH_RESERVED_FDPROT                  0x040FUL
#define KE0x_FLASH_RESERVED_FEPROT                  0x040EUL
#define KE0x_FLASH_RESERVED_FOPT                    0x040DUL
#define KE0x_FLASH_RESERVED_FSEC                    0x040CUL

#define KE0x_FLASH_PROGRAM_ONCE_START               0x0000UL /* Index access */
#define KE0x_FLASH_PROGRAM_ONCE_END                 0x003FUL
#define KE0x_FLASH_PROGRAM_ONCE_SIZE                0x40U

#define KE0x_FLASH_SECTOR_SIZE                      0x0200U     /* Erase, 512 Bytes */
#define KE0x_FLASH_PHRASE_SIZE                      0x08U       /* Write, 8 Bytes */

#if     defined(KE06Z4_SERIES)
    #define KE0x_FLASH_EEPROM_UNIT_ERASE_SIZE   0x00U
    #define KE0x_FLASH_EEPROM_UNIT_WRITE_SIZE   0x00U
#elif   defined(KE02Z4_SERIES)
    #define KE0x_FLASH_EEPROM_UNIT_ERASE_SIZE   0x01U
    #define KE0x_FLASH_EEPROM_UNIT_WRITE_SIZE   0x01U
#endif


/*!
    @name Flash controller command numbers
    @{
*/
#define FTMRx_ERASE_VERIFY_ALL_BLOCK          0x01U /*!< ERSVERALL*/
#define FTMRx_ERASE_VERIFY_BLOCK              0x02U /*!< ERSVERBLK*/
#define FTMRx_ERASE_VERIFY_SECTION            0x03U /*!< ERSVERSECT*/
#define FTMRx_READ_ONCE                       0x04U /*!< RDONCE or RDINDEX*/
#define FTMRx_PROGRAM                         0x06U /*!< PGM*/
#define FTMRx_PROGRAM_ONCE                    0x07U /*!< PGMONCE or PGMINDEX*/
#define FTMRx_ERASE_ALL_BLOCK                 0x08U /*!< ERSALL*/
#define FTMRx_ERASE_BLOCK                     0x09U /*!< ERSBLK*/
#define FTMRx_ERASE_SECTOR                    0x0AU /*!< ERSSCR*/
#define FTMRx_ERASE_ALL_BLOCK_UNSECURE        0x0BU /*!< ERSALLU*/
#define FTMRx_SECURITY_BY_PASS                0x0CU /*!< VFYKEY*/
#define FTMRx_SET_USER_MARGIN_LEVEL           0x0DU /*!< SETUSERLVL*/
#define FTMRx_SET_FACTORY_MARGIN_LEVEL        0x0EU /*!< SETFTYLVL*/
#define FTMRx_CONFIGURE_NVM                   0x0FU /*!< CONNVM*/
#define FTMRx_ERASE_VERIFY_EEPROM_SECTION     0x10U /*!< ERSVES*/
#define FTMRx_PROGRAM_EEPROM                  0x11U /*!< PGME*/
#define FTMRx_ERASE_EEPROM_SECTOR             0x12U /*!< ERSESCR*/
/* @} */

/*!
    @name Common flash register info defines
    @{
*/
#if defined(FTMRE)
#define FTMRx                      FTMRE
#define FTMRx_BASE                 FTMRE_BASE
#define FTMRx_FCCOBIX_CCOBIX(x)    FTMRE_FCCOBIX_CCOBIX(x)
#define FTMRx_FCCOBLO_CCOB(x)      FTMRE_FCCOBLO_CCOB(x)
#define FTMRx_FCCOBHI_CCOB(x)      FTMRE_FCCOBHI_CCOB(x)
#define FTMRx_FCLKDIV_FDIVLCK_MASK FTMRE_FCLKDIV_FDIVLCK_MASK
#define FTMRx_FCLKDIV_FDIVLD_MASK  FTMRE_FCLKDIV_FDIVLD_MASK
#define FTMRx_FCLKDIV_FDIV_MASK    FTMRE_FCLKDIV_FDIV_MASK
#define FTMRx_FSTAT_CCIF_MASK      FTMRE_FSTAT_CCIF_MASK
#define FTMRx_FCLKDIV_FDIV(x)      FTMRE_FCLKDIV_FDIV(x)
#define FTMRx_FSTAT_ACCERR_MASK    FTMRE_FSTAT_ACCERR_MASK
#define FTMRx_FSTAT_FPVIOL_MASK    FTMRE_FSTAT_FPVIOL_MASK
#define FTMRx_FSTAT_MGBUSY_MASK    FTMRE_FSTAT_MGBUSY_MASK
#define FTMRx_FSTAT_MGSTAT_MASK    FTMRE_FSTAT_MGSTAT_MASK
#define FTMRx_FSTAT_MGSTAT0_MASK   FTMRE_FSTAT_MGSTAT(1)
#define FTMRx_FSTAT_MGSTAT1_MASK   FTMRE_FSTAT_MGSTAT(2)
#define FTMRx_FSEC_SEC_MASK        FTMRE_FSEC_SEC_MASK
#define FTMRx_FSEC_KEYEN_MASK      FTMRE_FSEC_KEYEN_MASK
#elif defined(FTMRH)
#define FTMRx                      FTMRH
#define FTMRx_BASE                 FTMRH_BASE
#define FTMRx_FCCOBIX_CCOBIX(x)    FTMRH_FCCOBIX_CCOBIX(x)
#define FTMRx_FCCOBLO_CCOB(x)      FTMRH_FCCOBLO_CCOB(x)
#define FTMRx_FCCOBHI_CCOB(x)      FTMRH_FCCOBHI_CCOB(x)
#define FTMRx_FCLKDIV_FDIVLCK_MASK FTMRH_FCLKDIV_FDIVLCK_MASK
#define FTMRx_FCLKDIV_FDIVLD_MASK  FTMRH_FCLKDIV_FDIVLD_MASK
#define FTMRx_FCLKDIV_FDIV_MASK    FTMRH_FCLKDIV_FDIV_MASK
#define FTMRx_FCLKDIV_FDIV(x)      FTMRH_FCLKDIV_FDIV(x)
#define FTMRx_FSTAT_CCIF_MASK      FTMRH_FSTAT_CCIF_MASK
#define FTMRx_FSTAT_ACCERR_MASK    FTMRH_FSTAT_ACCERR_MASK
#define FTMRx_FSTAT_FPVIOL_MASK    FTMRH_FSTAT_FPVIOL_MASK
#define FTMRx_FSTAT_MGBUSY_MASK    FTMRH_FSTAT_MGBUSY_MASK
#define FTMRx_FSTAT_MGSTAT_MASK    FTMRH_FSTAT_MGSTAT_MASK
#define FTMRx_FSTAT_MGSTAT0_MASK   FTMRH_FSTAT_MGSTAT(1U)
#define FTMRx_FSTAT_MGSTAT1_MASK   FTMRH_FSTAT_MGSTAT(2U)
#define FTMRx_FSEC_SEC_MASK        FTMRH_FSEC_SEC_MASK
#define FTMRx_FSEC_KEYEN_MASK      FTMRH_FSEC_KEYEN_MASK
#endif
/* @} */

#define KE0x_FLASH_CLK_DIVIER ((uint8_t)(HAL_FLASH_CLOCK_SOURCE_FREQ / 1000000UL - 1UL))

/*
    Map to upper software layer
*/
#define HAL_FLASH_UNIT_ERASE_SIZE           KE0x_FLASH_SECTOR_SIZE
#define HAL_FLASH_UNIT_WRITE_SIZE           KE0x_FLASH_PHRASE_SIZE /* 8 byte aligned */
#define HAL_FLASH_UNIT_VERIFY_ERASE_SIZE    4U
#define HAL_FLASH_UNIT_VERIFY_WRITE_SIZE    0U
#define HAL_FLASH_UNIT_WRITE_ONCE_SIZE      KE0x_FLASH_PHRASE_SIZE
#define HAL_FLASH_UNIT_READ_ONCE_SIZE       KE0x_FLASH_PHRASE_SIZE
#define HAL_FLASH_UNIT_ERASE_PATTERN        (0xFFU)

#if     defined(KE06Z4_SERIES)
typedef FTMRE_Type HAL_Flash_T;
#elif   defined(KE02Z4_SERIES)
typedef FTMRH_Type HAL_Flash_T;
#endif


/*
    Module Helper
*/

/*!
    @brief Flash Set Command

    This function is used to write the command sequence to the flash reg.
*/
static void _flash_set_command(uint32_t index, uint32_t fValue, uint32_t sValue)
{
    FTMRx->FCCOBIX = FTMRx_FCCOBIX_CCOBIX(index);
    FTMRx->FCCOBLO = FTMRx_FCCOBLO_CCOB(fValue);
    FTMRx->FCCOBHI = FTMRx_FCCOBHI_CCOB(sValue);
}

static void _HAL_Flash_WriteCmdDest(HAL_Flash_T * p_regs, uintptr_t destAddress, const uint8_t cmd)
{
    (void)p_regs;
    /* Write index to specify the command code to be loaded */
    _flash_set_command(0UL, (destAddress) >> 16UL, cmd);
    _flash_set_command(1UL, (destAddress), (destAddress) >> 8UL);
}

static void _HAL_Flash_WriteCmdData(HAL_Flash_T * p_regs, const uint8_t * p_data)
{
    (void)p_regs;
    /* Little endian by default. */
    _flash_set_command(2U, p_data[0U], p_data[1U]); /* Write index to specify the word0 (MSB word) to be programmed */
    _flash_set_command(3U, p_data[2U], p_data[3U]); /* Write index to specify the word1 (LSB word) to be programmed */
    _flash_set_command(4U, p_data[4U], p_data[5U]); /* Write index to specify the word (MSB word) to be programmed */
    _flash_set_command(5U, p_data[6U], p_data[7U]); /* Write index to specify the word (LSB word) to be programmed */
}

static inline bool _HAL_Flash_ReadErrorFlagShared(const HAL_Flash_T * p_regs) { (void)p_regs; return ((FTMRx->FSTAT & (FTMRx_FSTAT_MGSTAT_MASK)) != 0U); }

/* Cmd Functions - when inline only _HAL_Flash_LaunchCmd need to reside in RAM */
static void _HAL_Flash_LaunchCmd(HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static void _HAL_Flash_LaunchCmd(HAL_Flash_T * p_regs) { (void)p_regs; FTMRx->FSTAT |= FTMRx_FSTAT_CCIF_MASK; }

/*
    API Common
*/
static inline bool HAL_Flash_ReadCompleteFlag(const HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline bool HAL_Flash_ReadCompleteFlag(const HAL_Flash_T * p_regs) { (void)p_regs; return ((FTMRx->FSTAT & FTMRx_FSTAT_CCIF_MASK) != 0U); }

static inline bool HAL_Flash_ReadErrorFlags(const HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline bool HAL_Flash_ReadErrorFlags(const HAL_Flash_T * p_regs) { (void)p_regs; return ((FTMRx->FSTAT & (FTMRx_FSTAT_MGSTAT_MASK | FTMRx_FSTAT_FPVIOL_MASK | FTMRx_FSTAT_ACCERR_MASK)) != 0U); }
static inline void HAL_Flash_ClearErrorFlags(HAL_Flash_T * p_regs)      { (void)p_regs; FTMRx->FSTAT = (uint8_t)(FTMRx_FSTAT_ACCERR_MASK | FTMRx_FSTAT_FPVIOL_MASK); }

static inline bool HAL_Flash_ReadErrorVerifyFlag(const HAL_Flash_T * p_regs)        { (void)p_regs; return _HAL_Flash_ReadErrorFlagShared(p_regs); }
// static inline bool HAL_Flash_ReadErrorFlagsVerify(const HAL_Flash_T * p_regs)    { (void)p_regs; return _HAL_Flash_ReadErrorFlagShared(p_regs); }
static inline bool HAL_Flash_ReadErrorProtectionFlag(const HAL_Flash_T * p_regs)    { (void)p_regs; return ((FTMRx->FSTAT & FTMRx_FSTAT_FPVIOL_MASK) != 0U); }

static inline bool HAL_Flash_ReadSecurityFlag(const HAL_Flash_T * p_regs)           { (void)p_regs; return ((FTMRx->FSEC & FTMRx_FSEC_SEC_MASK) >> FTMRE_FSEC_SEC_SHIFT != 0x02U); }

static inline void HAL_Flash_StartCmdWritePage(HAL_Flash_T * p_regs, uintptr_t destAddress, const uint8_t * p_data)
{
    _HAL_Flash_WriteCmdDest(p_regs, destAddress, FTMRx_PROGRAM);
    _HAL_Flash_WriteCmdData(p_regs, p_data);
    _HAL_Flash_LaunchCmd(p_regs);
}

static inline void HAL_Flash_StartCmdEraseSector(HAL_Flash_T * p_regs, uintptr_t destAddress)
{
    _HAL_Flash_WriteCmdDest(p_regs, destAddress, FTMRx_ERASE_SECTOR);
    _HAL_Flash_LaunchCmd(p_regs);
}

static inline void HAL_Flash_StartCmdEraseAll(HAL_Flash_T * p_regs)
{
    _flash_set_command(0U, 0U, FTMRx_ERASE_ALL_BLOCK);
    _HAL_Flash_LaunchCmd(p_regs);
}

/* Not supported on KE0x */
static inline void HAL_Flash_StartCmdVerifyWriteUnit(HAL_Flash_T * p_regs, uintptr_t destAddress, const uint8_t * p_data)
{
    (void)p_regs; (void)destAddress; (void)p_data;
}

static inline void HAL_Flash_StartCmdVerifyEraseUnit(HAL_Flash_T * p_regs, uintptr_t destAddress)
{
    _HAL_Flash_WriteCmdDest(p_regs, destAddress, FTMRx_ERASE_VERIFY_SECTION);
    _flash_set_command(2UL, 1U, 0U);
    _HAL_Flash_LaunchCmd(p_regs);
}

static inline void HAL_Flash_StartCmdVerifyEraseUnits(HAL_Flash_T * p_regs, uintptr_t destAddress, uint16_t units)
{
    _HAL_Flash_WriteCmdDest(p_regs, destAddress, FTMRx_ERASE_VERIFY_SECTION);
    _flash_set_command(2UL, units, units >> 16U);
    _HAL_Flash_LaunchCmd(p_regs);
}

/*
    Calling module ensures alignment

    Index
    0x0000 => 0
    0x0008 => 1
    0x0038 => 7
*/
static inline void HAL_Flash_StartCmdWriteOnce(HAL_Flash_T * p_regs, uintptr_t destAddress, const uint8_t * p_data)
{
    (void)p_regs;
    // assert(destAddress < 64U);
    uint8_t recordIndex = (destAddress - KE0x_FLASH_PROGRAM_ONCE_START) / KE0x_FLASH_PHRASE_SIZE;
    _flash_set_command(0UL, 0UL, FTMRx_PROGRAM_ONCE);
    _flash_set_command(1UL, recordIndex, 0U);
    _HAL_Flash_WriteCmdData(p_regs, p_data);
    _HAL_Flash_LaunchCmd(p_regs);
}

static inline void HAL_Flash_StartCmdReadOnce(HAL_Flash_T * p_regs, uintptr_t destAddress)
{
    (void)p_regs;
    // assert(destAddress < 64U);
    uint8_t recordIndex = (destAddress - KE0x_FLASH_PROGRAM_ONCE_START) / KE0x_FLASH_PHRASE_SIZE;
    _flash_set_command(0UL, 0UL, FTMRx_READ_ONCE);
    _flash_set_command(1UL, recordIndex, 0U);
    _HAL_Flash_LaunchCmd(p_regs);
}

static inline void HAL_Flash_ReadOnceData(HAL_Flash_T * p_regs, uint8_t * p_result)
{
    (void)p_regs;
    for(uint8_t iByte = 0U; iByte < HAL_FLASH_UNIT_READ_ONCE_SIZE; iByte += 2U)
    {
        FTMRx->FCCOBIX = FTMRx_FCCOBIX_CCOBIX(iByte / 2U + 2U);
        p_result[iByte]         = p_regs->FCCOBLO;
        p_result[iByte + 1U]    = p_regs->FCCOBHI;
    }
}

static inline void HAL_Flash_UnlockSecurity(HAL_Flash_T * p_regs, uint8_t * p_key)
{
    (void)p_regs;
    _flash_set_command(0UL, 0UL, FTMRx_SECURITY_BY_PASS);
    _flash_set_command(1U, p_key[0U], p_key[1U]);
    _flash_set_command(2U, p_key[2U], p_key[3U]);
    _flash_set_command(3U, p_key[4U], p_key[5U]);
    _flash_set_command(4U, p_key[6U], p_key[7U]);
}

// static inline void HAL_Flash_ConfigClock(HAL_Flash_T * p_regs)
// {

// }

static inline void HAL_Flash_Init(HAL_Flash_T * p_regs)
{
    (void)p_regs;

    if (((FTMRx->FCLKDIV & FTMRx_FCLKDIV_FDIVLCK_MASK) == 0U) && ((FTMRx->FSTAT & FTMRx_FSTAT_CCIF_MASK) != 0U))
    {
        /* FCLKDIV register is not locked.*/
        FTMRx->FCLKDIV = (uint8_t)(FTMRx->FCLKDIV & (~FTMRx_FCLKDIV_FDIV_MASK)) | FTMRx_FCLKDIV_FDIV(KE0x_FLASH_CLK_DIVIER);
    }

    assert((FTMRx->FCLKDIV & FTMRx_FCLKDIV_FDIV_MASK) == FTMRx_FCLKDIV_FDIV(KE0x_FLASH_CLK_DIVIER));
}

#endif /* HAL_FLASH_H */
