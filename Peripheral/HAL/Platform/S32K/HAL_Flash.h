/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef HAL_FLASH_PLATFORM_H
#define HAL_FLASH_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

/*
	S32K142 Chip Boundaries
	0000_0000	0003_FFFF	 	Program / code flash
	1000_0000	1000_FFFF	 	FlexNVM / code flash
	1400_0000	1400_0FFF		FlexRAM
*/

#define S32K_PROGRAM_FLASH_START	0x00000000	/* */
#define S32K_PROGRAM_FLASH_END		0x0003FFFF	/* */
#define S32K_PROGRAM_FLASH_SIZE		0x00040000

#define S32K_FLEX_NVM_START		0x10000000	/* */
#define S32K_FLEX_NVM_END		0x1000FFFF	/* */
#define S32K_FLEX_NVM_SIZE		0x00010000

#define S32K_FLEX_NVM_CMD_OFFSET ((int32_t)0x00800000 - 0x10000000)

#define S32K_FLEX_RAM_START		0x14000000	/* */
#define S32K_FLEX_RAM_END		0x14000FFF	/* */
#define S32K_FLEX_RAM_SIZE		0x00001000

/* Program Flash IFR */
#define S32K_FLASH_PROGRAM_ONCE_START	0x03C0
#define S32K_FLASH_PROGRAM_ONCE_END		0x03FF
#define S32K_FLASH_PROGRAM_ONCE_SIZE	0x0040	/* 64 Bytes. */

#define S32K_FLASH_SECTOR_SIZE				0x00000800U	/* 2KB */
#define S32K_FLASH_PHRASE_SIZE				0x00000008U	/* 64-bits */

#define S32K_FLASH_EEPROM_UNIT_ERASE_SIZE	0x01
#define S32K_FLASH_EEPROM_UNIT_WRITE_SIZE	0x01

/* Program Flash */
#define S32K_FLASH_RESERVED_START	0x0000	/* */
#define S32K_FLASH_RESERVED_END		0x040F	/* */

/* Program Flash IFR */
#define S32K_FLASH_RESERVED_IFR_START			0x0000	/* */
#define S32K_FLASH_RESERVED_IFR_END				0x03FF	/* */
#define S32K_FLASH_RESERVED_CONFIG_START		0x0400	/* */
#define S32K_FLASH_RESERVED_CONFIG_END			0x040F	/* */
#define S32K_FLASH_RESERVED_BACKDOOR_KEY_START	0x0400	//8 bytes
#define S32K_FLASH_RESERVED_BACKDOOR_KEY_END	0x0407	//8 bytes
#define S32K_FLASH_RESERVED_FPROT3				0x0408
#define S32K_FLASH_RESERVED_FPROT2				0x0409
#define S32K_FLASH_RESERVED_FPROT1				0x040A
#define S32K_FLASH_RESERVED_FPROT0				0x040B
#define S32K_FLASH_RESERVED_FDPROT				0x040F
#define S32K_FLASH_RESERVED_FEPROT				0x040E
#define S32K_FLASH_RESERVED_FOPT				0x040D
#define S32K_FLASH_RESERVED_FSEC				0x040C

/* Data(FlexNvM) Flash */
// #define S32K_FLASH_RESERVED_START	0x0000	/* */
// #define S32K_FLASH_RESERVED_END		0x0400	/* */

/* Data(FlexNvM) Flash IFR */
#define S32K_FLASH_RESERVED_EEPROM_SIZE			0x03FD
#define S32K_FLASH_RESERVED_FLEX_NVM_CODE		0x03FC
/*
	Map to upper software layer
*/
#define HAL_FLASH_UNIT_ERASE_SIZE			S32K_FLASH_SECTOR_SIZE
#define HAL_FLASH_UNIT_WRITE_SIZE			S32K_FLASH_PHRASE_SIZE /* 8 byte aligned */
#define HAL_FLASH_ERASE_SECTOR_SIZE			HAL_FLASH_UNIT_ERASE_SIZE /* alias */
#define HAL_FLASH_WRITE_PAGE_SIZE			HAL_FLASH_UNIT_WRITE_SIZE
#define HAL_FLASH_UNIT_VERIFY_ERASE_SIZE	S32K_FLASH_SECTOR_SIZE
#define HAL_FLASH_UNIT_VERIFY_WRITE_SIZE	4U
#define HAL_FLASH_UNIT_WRITE_ONCE_SIZE		S32K_FLASH_PHRASE_SIZE
#define HAL_FLASH_UNIT_READ_ONCE_SIZE		S32K_FLASH_PHRASE_SIZE

/*
	S32K  FLASH/EEPROM use same controller, same HAL
	only 1 flash controller, unused
*/
typedef FTFC_Type HAL_Flash_T;

/* Register rename */
#define FTFx_BASE      FTFC_BASE
#define FTFC_FSTAT     FTFC->FSTAT
#define FTFx_FCNFG     FTFC->FCNFG
#define FTFx_FSEC      FTFC->FSEC
#define FTFx_FOPT      FTFC->FOPT
#define FTFx_FCCOB3    FTFC->FCCOB[0]
#define FTFx_FCCOB2    FTFC->FCCOB[1]
#define FTFx_FCCOB1    FTFC->FCCOB[2]
#define FTFx_FCCOB0    FTFC->FCCOB[3]
#define FTFx_FCCOB7    FTFC->FCCOB[4]
#define FTFx_FCCOB6    FTFC->FCCOB[5]
#define FTFx_FCCOB5    FTFC->FCCOB[6]
#define FTFx_FCCOB4    FTFC->FCCOB[7]
#define FTFx_FCCOBB    FTFC->FCCOB[8]
#define FTFx_FCCOBA    FTFC->FCCOB[9]
#define FTFx_FCCOB9    FTFC->FCCOB[10]
#define FTFx_FCCOB8    FTFC->FCCOB[11]
#define FTFx_FPROT3    FTFC->FPROT[0]
#define FTFx_FPROT2    FTFC->FPROT[1]
#define FTFx_FPROT1    FTFC->FPROT[2]
#define FTFx_FPROT0    FTFC->FPROT[3]
#define FTFx_FEPROT    FTFC->FEPROT
#define FTFx_FDPROT    FTFC->FDPROT
#define FTFx_FCSESTAT  FTFC->FCSESTAT

#define FTFx_VERIFY_BLOCK             0x00U
#define FTFx_VERIFY_SECTION           0x01U
#define FTFx_PROGRAM_CHECK            0x02U
#define FTFx_READ_RESOURCE            0x03U
#define FTFx_PROGRAM_LONGWORD         0x06U
#define FTFx_PROGRAM_PHRASE           0x07U
#define FTFx_ERASE_BLOCK              0x08U
#define FTFx_ERASE_SECTOR             0x09U
#define FTFx_PROGRAM_SECTION          0x0BU
#define FTFx_VERIFY_ALL_BLOCK         0x40U
#define FTFx_READ_ONCE                0x41U
#define FTFx_PROGRAM_ONCE             0x43U
#define FTFx_ERASE_ALL_BLOCK          0x44U
#define FTFx_SECURITY_BY_PASS         0x45U
#define FTFx_PFLASH_SWAP              0x46U
#define FTFx_ERASE_ALL_BLOCK_UNSECURE 0x49U
#define FTFx_PROGRAM_PARTITION        0x80U
#define FTFx_SET_EERAM                0x81U

#define FTFx_WORD_SIZE     0x0002U	/* Word size 2 bytes */
#define FTFx_LONGWORD_SIZE 0x0004U	/* Long word size 4 bytes */
#define FTFx_PHRASE_SIZE   0x0008U	/* Phrase size 8 bytes */
#define FTFx_DPHRASE_SIZE  0x0010U	/* Double-phrase size 16 bytes */

#define GET_BIT_0_7(value)   ((uint8_t)(((uint32_t)(value)) & 0xFFU))
#define GET_BIT_8_15(value)  ((uint8_t)((((uint32_t)(value)) >> 8U) & 0xFFU))
#define GET_BIT_16_23(value) ((uint8_t)((((uint32_t)(value)) >> 16U) & 0xFFU))
#define GET_BIT_24_31(value) ((uint8_t)(((uint32_t)(value)) >> 24U))

/*
	Use CONFIG_FLASH_ATTRIBUTE_RAM_SECTION incase functions are not inlined (and stored in ram with calling function)
*/
#include "Peripheral/NvMemory/Flash/Config.h"

/*
	module helper
*/
static inline void _HAL_Flash_WriteCmdDest(HAL_Flash_T * p_regs, const uint8_t * p_dest)
{
	(void)p_regs;
	FTFx_FCCOB1 = GET_BIT_16_23(p_dest);
	FTFx_FCCOB2 = GET_BIT_8_15(p_dest);
	FTFx_FCCOB3 = GET_BIT_0_7(p_dest);
}

static inline void _HAL_Flash_WriteCmdWriteData(HAL_Flash_T * p_regs, const uint8_t * p_data)
{
	for(uint8_t iByte = 0U; iByte < HAL_FLASH_UNIT_WRITE_SIZE; iByte++)
	{
		// ((uint8_t *)FTFx_BASE)[iByte + 0x08U] = p_data[iByte];
		p_regs->FCCOB[iByte + 0x04U] = p_data[iByte];
	}
}

static inline void _HAL_Flash_WriteCmdStart(HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline void _HAL_Flash_WriteCmdStart(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	FTFC_FSTAT |= FTFC_FSTAT_CCIF_MASK;
}

static inline bool _HAL_Flash_ReadErrorFlagShared(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	return ((FTFC_FSTAT & (FTFC_FSTAT_MGSTAT0_MASK)) != 0U) ? true : false;
}

/*
	API Common
*/
static inline bool HAL_Flash_ReadCompleteFlag(HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline bool HAL_Flash_ReadCompleteFlag(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	return ((FTFC_FSTAT & FTFC_FSTAT_CCIF_MASK) != 0U) ? true : false;
}

static inline bool HAL_Flash_ReadErrorFlags(HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline bool HAL_Flash_ReadErrorFlags(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	return ((FTFC_FSTAT & (FTFC_FSTAT_MGSTAT0_MASK | FTFC_FSTAT_FPVIOL_MASK | FTFC_FSTAT_ACCERR_MASK | FTFC_FSTAT_RDCOLERR_MASK)) != 0U) ? true : false;
}

//static inline void HAL_Flash_ClearErrorFlags(HAL_Flash_T * p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline void HAL_Flash_ClearErrorFlags(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	FTFC_FSTAT = (uint8_t)(FTFC_FSTAT_FPVIOL_MASK | FTFC_FSTAT_ACCERR_MASK | FTFC_FSTAT_RDCOLERR_MASK);
}

//static inline bool HAL_Flash_ReadErrorVerifyFlag(HAL_Flash_T *p_regs) CONFIG_FLASH_ATTRIBUTE_RAM_SECTION;
static inline bool HAL_Flash_ReadErrorVerifyFlag(HAL_Flash_T * p_regs)
{
	return _HAL_Flash_ReadErrorFlagShared(p_regs);
}

static inline bool HAL_Flash_ReadErrorProtectionFlag(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	return (FTFC_FSTAT & (FTFC_FSTAT_MGSTAT0_MASK | FTFC_FSTAT_FPVIOL_MASK));
}

static inline bool HAL_Flash_ReadSecurityFlag(HAL_Flash_T * p_regs)
{
	(void)p_regs;
	return ((FTFx_FSEC & FTFC_FSEC_SEC_MASK) != FTFC_FSEC_SEC(0x02U)) ? true : false;
}

/*
	Cmd Functions
*/
static inline void HAL_Flash_StartCmdWritePage(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data)
{
	FTFx_FCCOB0 = FTFx_PROGRAM_PHRASE;
	_HAL_Flash_WriteCmdDest(p_regs, p_dest);
	_HAL_Flash_WriteCmdWriteData(p_regs, p_data);
	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_StartCmdEraseSector(HAL_Flash_T * p_regs, const uint8_t * p_dest)
{
	FTFx_FCCOB0 = FTFx_ERASE_SECTOR;
	_HAL_Flash_WriteCmdDest(p_regs, p_dest);
	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_StartCmdVerifyWriteUnit(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data)
{
	FTFx_FCCOB0 = FTFx_PROGRAM_CHECK;
	_HAL_Flash_WriteCmdDest(p_regs, p_dest);
	FTFx_FCCOB4 = 1U; //CONFIG_HAL_FLASH_MARGIN_LEVEL

	for(uint8_t iByte = 0U; iByte < HAL_FLASH_UNIT_VERIFY_WRITE_SIZE; iByte++)
	{
		// ((uint8_t *)FTFx_BASE)[iByte + 0x0CU] = p_data[iByte];
		p_regs->FCCOB[iByte + 0x08U] = p_data[iByte];
	}

	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_StartCmdVerifyEraseUnit(HAL_Flash_T * p_regs, const uint8_t * p_dest)
{
	FTFx_FCCOB0 = FTFx_VERIFY_SECTION;
	_HAL_Flash_WriteCmdDest(p_regs, p_dest);
	FTFx_FCCOB4 = GET_BIT_8_15(0U);
	FTFx_FCCOB5 = GET_BIT_0_7(1U);
	FTFx_FCCOB6 = 1U; //CONFIG_HAL_FLASH_MARGIN_LEVEL
	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_StartCmdVerifyEraseUnits(HAL_Flash_T * p_regs, const uint8_t * p_dest, uint16_t units)
{
	FTFx_FCCOB0 = FTFx_VERIFY_SECTION;
	_HAL_Flash_WriteCmdDest(p_regs, p_dest);
	FTFx_FCCOB4 = GET_BIT_8_15(units);
	FTFx_FCCOB5 = GET_BIT_0_7(units);
	FTFx_FCCOB6 = 1U; //CONFIG_HAL_FLASH_MARGIN_LEVEL
	_HAL_Flash_WriteCmdStart(p_regs);
}

/*
	Calling module ensures alignment
	56 Bytes 7 Units available. 8th unit is config

	Index
	0x03C0 => 0
	0x03C8 => 1
	0x03D0 => 2
*/
static inline void HAL_Flash_StartCmdWriteOnce(HAL_Flash_T * p_regs, const uint8_t * p_dest, const uint8_t * p_data)
{
	uint8_t recordIndex = (p_dest - (uint8_t *)S32K_FLASH_PROGRAM_ONCE_START) / S32K_FLASH_PHRASE_SIZE;
	FTFx_FCCOB0 = FTFx_PROGRAM_ONCE;
	FTFx_FCCOB1 = recordIndex;
	_HAL_Flash_WriteCmdWriteData(p_regs, p_data);
	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_StartCmdReadOnce(HAL_Flash_T * p_regs, const uint8_t * p_dest)
{
	uint8_t recordIndex = (p_dest - (uint8_t *)S32K_FLASH_PROGRAM_ONCE_START) / S32K_FLASH_PHRASE_SIZE;
	FTFx_FCCOB0 = FTFx_READ_ONCE;
	FTFx_FCCOB1 = recordIndex;
	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_ReadOnceData(HAL_Flash_T * p_regs, uint8_t * p_result)
{
	for(uint8_t iByte = 0U; iByte < HAL_FLASH_UNIT_READ_ONCE_SIZE; iByte++)
	{
		// p_result[iByte] = ((uint8_t *)FTFx_BASE)[iByte + 0x08U];
		p_result[iByte] = p_regs->FCCOB[iByte + 0x04U];
	}
}

static inline void HAL_Flash_StartCmdEraseAll(HAL_Flash_T * p_regs)
{
	FTFx_FCCOB0 = FTFx_ERASE_ALL_BLOCK;
	_HAL_Flash_WriteCmdStart(p_regs);
}

static inline void HAL_Flash_Init(HAL_Flash_T * p_regs)
{
	(void)p_regs;
}

#endif /* HAL_FLASH_H */
