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

#define S32K_FLEX_RAM_START		0x14000000	/* */
#define S32K_FLEX_RAM_END		0x14000FFF	/* */
#define S32K_FLEX_RAM_SIZE		0x00001000

#define S32K_FLASH_RESERVED_START	0x0000	/* */
#define S32K_FLASH_RESERVED_END		0x040F	/* */

#define S32K_FLASH_RESERVED_CONFIG_START		0x0400	/* */
#define S32K_FLASH_RESERVED_CONFIG_END			0x040F	/* */
#define S32K_FLASH_RESERVED_BACKDOOR_KEY_START	0x0400	//8 bytes
#define S32K_FLASH_RESERVED_BACKDOOR_KEY_END	0x0407	//8 bytes
#define S32K_FLASH_RESERVED_FPROT0				0x0408
#define S32K_FLASH_RESERVED_FPROT1				0x0409
#define S32K_FLASH_RESERVED_FPROT2				0x040A
#define S32K_FLASH_RESERVED_FPROT3				0x040B
#define S32K_FLASH_RESERVED_FDPROT				0x040F
#define S32K_FLASH_RESERVED_FEPROT				0x040E
#define S32K_FLASH_RESERVED_FOPT				0x040D
#define S32K_FLASH_RESERVED_FSEC				0x040C

#define S32K_FLASH_RESERVED_IFR_START			0x0000	/* */
#define S32K_FLASH_RESERVED_IFR_END				0x03FF	/* */

#define S32K_FLASH_RESERVED_PROGRAM_ONCE_START	0x03C0
#define S32K_FLASH_RESERVED_PROGRAM_ONCE_END	0x03FF
#define S32K_FLASH_RESERVED_EEPROM_SIZE			0x03FD
#define S32K_FLASH_RESERVED_FLEX_NVM_CODE		0x03FC

#define S32K_FLASH_SECTOR_SIZE				(0x00000800U)	/* 2KB */
#define S32K_FLASH_PHRASE_SIZE				0x00000008U	/* 64-bits */

#define S32K_FLASH_EEPROM_UNIT_SIZE_ERASE	0x01
#define S32K_FLASH_EEPROM_UNIT_SIZE_WRITE	0x01

/*
 * Bounds
 */
#define HAL_FLASH_UNIT_ERASE_SIZE	S32K_FLASH_SECTOR_SIZE
#define HAL_FLASH_UNIT_WRITE_SIZE	S32K_FLASH_PHRASE_SIZE /* 8 byte aligned */
#define HAL_FLASH_ERASE_BLOCK_SIZE	HAL_FLASH_UNIT_ERASE_SIZE /* alias */
#define HAL_FLASH_WRITE_PAGE_SIZE	HAL_FLASH_UNIT_WRITE_SIZE

#define HAL_FLASH_START				S32K_PROGRAM_FLASH_START	/* */
#define HAL_FLASH_END				S32K_PROGRAM_FLASH_END	/* */
#define HAL_FLASH_SIZE				S32K_PROGRAM_FLASH_SIZE	/* */

#define HAL_FLASH_DATA_START			S32K_FLEX_NVM_START			/* First 32KB allocated to EEPROM */
#define HAL_FLASH_DATA_END				S32K_FLEX_NVM_END	/* */
#define HAL_FLASH_DATA_SIZE				S32K_FLEX_NVM_END			/* 32KB remaining,  */

#define HAL_FLASH_ONCE_START			S32K_FLASH_RESERVED_PROGRAM_ONCE_START	/* */
#define HAL_FLASH_ONCE_END				0x03FB	/* */
#define HAL_FLASH_ONCE_SIZE				60U	/* 60 Bytes, last 4 reserved for config field */
#define HAL_FLASH_ONCE_SIZE_ERASE		S32K_FLASH_SECTOR_SIZE
#define HAL_FLASH_ONCE_SIZE_WRITE		S32K_FLASH_PHRASE_SIZE

#define FTFx_BASE                               FTFC_BASE
#define FTFx_FSTAT                              FTFC->FSTAT
#define FTFx_FCNFG                              FTFC->FCNFG
#define FTFx_FSEC                               FTFC->FSEC
#define FTFx_FOPT                               FTFC->FOPT
#define FTFx_FCCOB3                             FTFC->FCCOB[0]
#define FTFx_FCCOB2                             FTFC->FCCOB[1]
#define FTFx_FCCOB1                             FTFC->FCCOB[2]
#define FTFx_FCCOB0                             FTFC->FCCOB[3]
#define FTFx_FCCOB7                             FTFC->FCCOB[4]
#define FTFx_FCCOB6                             FTFC->FCCOB[5]
#define FTFx_FCCOB5                             FTFC->FCCOB[6]
#define FTFx_FCCOB4                             FTFC->FCCOB[7]
#define FTFx_FCCOBB                             FTFC->FCCOB[8]
#define FTFx_FCCOBA                             FTFC->FCCOB[9]
#define FTFx_FCCOB9                             FTFC->FCCOB[10]
#define FTFx_FCCOB8                             FTFC->FCCOB[11]
#define FTFx_FPROT3                             FTFC->FPROT[0]
#define FTFx_FPROT2                             FTFC->FPROT[1]
#define FTFx_FPROT1                             FTFC->FPROT[2]
#define FTFx_FPROT0                             FTFC->FPROT[3]
#define FTFx_FEPROT                             FTFC->FEPROT
#define FTFx_FDPROT                             FTFC->FDPROT
#define FTFx_FCSESTAT                           FTFC->FCSESTAT

#define FTFx_FSTAT_MGSTAT0_MASK                 FTFC_FSTAT_MGSTAT0_MASK
#define FTFx_FSTAT_MGSTAT0_SHIFT                FTFC_FSTAT_MGSTAT0_SHIFT
#define FTFx_FSTAT_MGSTAT0_WIDTH                FTFC_FSTAT_MGSTAT0_WIDTH
#define FTFx_FSTAT_MGSTAT0(x)                   FTFC_FSTAT_MGSTAT0(x)
#define FTFx_FSTAT_FPVIOL_MASK                  FTFC_FSTAT_FPVIOL_MASK
#define FTFx_FSTAT_FPVIOL_SHIFT                 FTFC_FSTAT_FPVIOL_SHIFT
#define FTFx_FSTAT_FPVIOL_WIDTH                 FTFC_FSTAT_FPVIOL_WIDTH
#define FTFx_FSTAT_FPVIOL(x)                    FTFC_FSTAT_FPVIOL(x)
#define FTFx_FSTAT_ACCERR_MASK                  FTFC_FSTAT_ACCERR_MASK
#define FTFx_FSTAT_ACCERR_SHIFT                 FTFC_FSTAT_ACCERR_SHIFT
#define FTFx_FSTAT_ACCERR_WIDTH                 FTFC_FSTAT_ACCERR_WIDTH
#define FTFx_FSTAT_ACCERR(x)                    FTFC_FSTAT_ACCERR(x)
#define FTFx_FSTAT_RDCOLERR_MASK                FTFC_FSTAT_RDCOLERR_MASK
#define FTFx_FSTAT_RDCOLERR_SHIFT               FTFC_FSTAT_RDCOLERR_SHIFT
#define FTFx_FSTAT_RDCOLERR_WIDTH               FTFC_FSTAT_RDCOLERR_WIDTH
#define FTFx_FSTAT_RDCOLERR(x)                  FTFC_FSTAT_RDCOLERR(x)
#define FTFx_FSTAT_CCIF_MASK                    FTFC_FSTAT_CCIF_MASK
#define FTFx_FSTAT_CCIF_SHIFT                   FTFC_FSTAT_CCIF_SHIFT
#define FTFx_FSTAT_CCIF_WIDTH                   FTFC_FSTAT_CCIF_WIDTH
#define FTFx_FSTAT_CCIF(x)                      FTFC_FSTAT_CCIF(x)

#define FTFx_FCNFG_EEERDY_MASK                  FTFC_FCNFG_EEERDY_MASK
#define FTFx_FCNFG_EEERDY_SHIFT                 FTFC_FCNFG_EEERDY_SHIFT
#define FTFx_FCNFG_EEERDY_WIDTH                 FTFC_FCNFG_EEERDY_WIDTH
#define FTFx_FCNFG_EEERDY(x)                    FTFC_FCNFG_EEERDY(x)
#define FTFx_FCNFG_RAMRDY_MASK                  FTFC_FCNFG_RAMRDY_MASK
#define FTFx_FCNFG_RAMRDY_SHIFT                 FTFC_FCNFG_RAMRDY_SHIFT
#define FTFx_FCNFG_RAMRDY_WIDTH                 FTFC_FCNFG_RAMRDY_WIDTH
#define FTFx_FCNFG_RAMRDY(x)                    FTFC_FCNFG_RAMRDY(x)
#define FTFx_FCNFG_ERSSUSP_MASK                 FTFC_FCNFG_ERSSUSP_MASK
#define FTFx_FCNFG_ERSSUSP_SHIFT                FTFC_FCNFG_ERSSUSP_SHIFT
#define FTFx_FCNFG_ERSSUSP_WIDTH                FTFC_FCNFG_ERSSUSP_WIDTH
#define FTFx_FCNFG_ERSSUSP(x)                   FTFC_FCNFG_ERSSUSP(x)
#define FTFx_FCNFG_ERSAREQ_MASK                 FTFC_FCNFG_ERSAREQ_MASK
#define FTFx_FCNFG_ERSAREQ_SHIFT                FTFC_FCNFG_ERSAREQ_SHIFT
#define FTFx_FCNFG_ERSAREQ_WIDTH                FTFC_FCNFG_ERSAREQ_WIDTH
#define FTFx_FCNFG_ERSAREQ(x)                   FTFC_FCNFG_ERSAREQ(x)
#define FTFx_FCNFG_RDCOLLIE_MASK                FTFC_FCNFG_RDCOLLIE_MASK
#define FTFx_FCNFG_RDCOLLIE_SHIFT               FTFC_FCNFG_RDCOLLIE_SHIFT
#define FTFx_FCNFG_RDCOLLIE_WIDTH               FTFC_FCNFG_RDCOLLIE_WIDTH
#define FTFx_FCNFG_RDCOLLIE(x)                  FTFC_FCNFG_RDCOLLIE(x)
#define FTFx_FCNFG_CCIE_MASK                    FTFC_FCNFG_CCIE_MASK
#define FTFx_FCNFG_CCIE_SHIFT                   FTFC_FCNFG_CCIE_SHIFT
#define FTFx_FCNFG_CCIE_WIDTH                   FTFC_FCNFG_CCIE_WIDTH
#define FTFx_FCNFG_CCIE(x)                      FTFC_FCNFG_CCIE(x)

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
 * S32K  FLASH/EEPROM use same controller, same HAL
 */
typedef FTFC_Type HAL_Flash_T;


/*
 *   module helper
 */
static inline void HAL_Flash_WriteCmdDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest)
{
    FTFx_FCCOB1 = GET_BIT_16_23(p_dest);
    FTFx_FCCOB2 = GET_BIT_8_15(p_dest);
    FTFx_FCCOB3 = GET_BIT_0_7(p_dest);
}

static inline void HAL_Flash_WriteCmdWriteData(HAL_Flash_T * p_hal_flash, const uint8_t * p_data)
{
	for (uint8_t i = 0U; i < FTFx_PHRASE_SIZE; i++)
	{
		((uint8_t*)FTFx_BASE)[i + 0x08U] = p_data[i];
	}
}

static inline void HAL_Flash_WriteCmdStart(HAL_Flash_T * p_hal_flash)
{
	 FTFx_FSTAT |= FTFx_FSTAT_CCIF_MASK;
}

/*
 * API
 */
/*
 *   Common
 */
static inline bool HAL_Flash_ReadErrorFlags(HAL_Flash_T *p_hal_flash)
{
	return	(FTFx_FSTAT & (FTFx_FSTAT_MGSTAT0_MASK | FTFx_FSTAT_FPVIOL_MASK | FTFx_FSTAT_ACCERR_MASK | FTFx_FSTAT_RDCOLERR_MASK));
}

static inline void HAL_Flash_ClearErrorFlags(HAL_Flash_T *p_hal_flash)
{
	FTFx_FSTAT = (uint8_t)(FTFx_FSTAT_FPVIOL_MASK | FTFx_FSTAT_ACCERR_MASK | FTFx_FSTAT_RDCOLERR_MASK);
}

static inline bool HAL_Flash_ReadCompleteFlags(HAL_Flash_T * p_hal_flash)
{
	return (bool)(FTFx_FSTAT & FTFx_FSTAT_CCIF_MASK);
}

static inline bool HAL_Flash_ReadErrorVerifyFlag(HAL_Flash_T *p_hal_flash)
{
	return	(FTFx_FSTAT & (FTFx_FSTAT_MGSTAT0_MASK));
}

/*
 * Functions
 */
static inline void HAL_Flash_StartCmdWritePage(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data)
{
    FTFx_FCCOB0 = FTFx_PROGRAM_PHRASE;
    HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);
    HAL_Flash_WriteCmdWriteData(p_hal_flash, p_data);
	HAL_Flash_WriteCmdStart(p_hal_flash);
}

static inline void HAL_Flash_StartCmdEraseSector(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest)
{
	FTFx_FCCOB0 = FTFx_ERASE_SECTOR;
    HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);
	HAL_Flash_WriteCmdStart(p_hal_flash);
}

static inline void HAL_Flash_StartCmdVerifyWriteUnit(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data)
{
	FTFx_FCCOB0 = FTFx_PROGRAM_CHECK;
    HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);
    //writedata
	HAL_Flash_WriteCmdStart(p_hal_flash);
}

//static inline void HAL_Flash_StartCmdVerifyEraseUnit(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest)
//{
//	FTFx_FCCOB0 = FTFx_VERIFY_SECTION;
//}

static inline void HAL_Flash_StartCmdVerifyEraseUnits(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, uint8_t units)
{
	FTFx_FCCOB0 = FTFx_VERIFY_SECTION;
    HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);
    //write units
	HAL_Flash_WriteCmdStart(p_hal_flash);
}

/*
 * Calling module ensures alignment
 * 0x03C0 => 0
 * 0x03C8 => 1
 */
static inline void HAL_Flash_StartCmdWriteOnce(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data)
{
	uint8_t recordIndex = p_dest - (uint8_t *)HAL_FLASH_ONCE_START;

	FTFx_FCCOB0 = FTFx_PROGRAM_ONCE;
	FTFx_FCCOB1 = recordIndex;

	HAL_Flash_WriteCmdWriteData(p_hal_flash, p_data);
	HAL_Flash_WriteCmdStart(p_hal_flash);
}

static inline void HAL_Flash_StartCmdReadOnce(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest)
{
	uint8_t recordIndex = p_dest - (uint8_t *)HAL_FLASH_ONCE_START;

	FTFx_FCCOB0 = FTFx_READ_ONCE;
	FTFx_FCCOB1 = recordIndex;

	HAL_Flash_WriteCmdStart(p_hal_flash);
}

static inline void HAL_Flash_ReadOnceData(HAL_Flash_T * p_hal_flash, uint8_t * p_result)
{
	for (uint8_t i = 0U; i < FTFx_PHRASE_SIZE; i++)
	{
        p_result[i] = ((uint8_t *)FTFx_BASE)[i + 0x08U];
    }
}

static inline void HAL_Flash_Init(HAL_Flash_T * p_hal_flash)
{

}


/*
 * Mapped
 */
//static inline void HAL_Flash_WriteCmdWritePage(HAL_Flash_T * p_hal_flash)
//{
//	FTFx_FCCOB0 = FTFx_PROGRAM_PHRASE;
//}
//
//static inline void HAL_Flash_WriteCmdWriteData(HAL_Flash_T * p_hal_flash, const uint8_t * p_data)
//{
//	for (uint8_t i = 0U; i < FTFx_PHRASE_SIZE; i++)
//	{
//		((uint8_t*)FTFx_BASE)[i + 0x08U] = p_data[i];
//	}
//}
//
//static inline void HAL_Flash_WriteCmdEraseSector(HAL_Flash_T * p_hal_flash)
//{
//	FTFx_FCCOB0 = FTFx_ERASE_SECTOR;
//}
//
//static inline void HAL_Flash_WriteCmdVerifyEraseSector(HAL_Flash_T * p_hal_flash)
//{
//	FTFx_FCCOB0 = FTFx_VERIFY_SECTION;
//}
//
//static inline void HAL_Flash_WriteCmdVerifyWritePage(HAL_Flash_T * p_hal_flash)
//{
//	FTFx_FCCOB0 = FTFx_PROGRAM_CHECK;
//}
//
//static inline void HAL_Flash_WriteCmdWriteOnce(HAL_Flash_T * p_hal_flash)
//{
//	FTFx_FCCOB0 = FTFx_PROGRAM_ONCE;
//}
//
//static inline void HAL_Flash_WriteCmdReadOnce(HAL_Flash_T * p_hal_flash)
//{
//	FTFx_FCCOB0 = FTFx_READ_ONCE;
//}
//
//static inline void HAL_Flash_WriteCmdWriteDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);}
//static inline void HAL_Flash_WriteCmdWriteStart(HAL_Flash_T * p_hal_flash){HAL_Flash_WriteCmdStart(p_hal_flash);}
//static inline bool HAL_Flash_ReadCompleteWriteFlag(HAL_Flash_T * p_hal_flash){return HAL_Flash_ReadCompleteFlags(p_hal_flash);}
//static inline void HAL_Flash_PrepCmdWrite(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data){}
//
//static inline void HAL_Flash_WriteCmdEraseDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);}
//static inline void HAL_Flash_WriteCmdEraseStart(HAL_Flash_T * p_hal_flash){HAL_Flash_WriteCmdStart(p_hal_flash);}
//static inline bool HAL_Flash_ReadCompleteEraseFlag(HAL_Flash_T * p_hal_flash){return HAL_Flash_ReadCompleteFlags(p_hal_flash);}
//static inline void HAL_Flash_PrepCmdErase(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//
//static inline void HAL_Flash_WriteCmdVerifyWriteDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);}
//static inline void HAL_Flash_WriteCmdVerifyWriteStart(HAL_Flash_T * p_hal_flash){HAL_Flash_WriteCmdStart(p_hal_flash);}
//static inline bool HAL_Flash_ReadCompleteVerifyWriteFlag(HAL_Flash_T * p_hal_flash){return HAL_Flash_ReadCompleteFlags(p_hal_flash);}
//static inline void HAL_Flash_PrepCmdVerifyWrite(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}
//
//static inline void HAL_Flash_WriteCmdVerifyEraseDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){HAL_Flash_WriteCmdDest(p_hal_flash, p_dest);}
//static inline void HAL_Flash_WriteCmdVerifyEraseStart(HAL_Flash_T * p_hal_flash){HAL_Flash_WriteCmdStart(p_hal_flash);}
//static inline bool HAL_Flash_ReadCompleteVerifyEraseFlag(HAL_Flash_T * p_hal_flash){return HAL_Flash_ReadCompleteFlags(p_hal_flash);}
//static inline void HAL_Flash_PrepCmdVerifyErase(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}

/*
 * Calling module ensures alignment
 * 0x03C0 => 0
 * 0x03C8 => 1
 */
//static inline void HAL_Flash_WriteCmdWriteOnceDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest)
//{
//	uint8_t recordIndex = p_dest - (uint8_t *)HAL_FLASH_ONCE_START;
//
//	FTFx_FCCOB1 = recordIndex;
//}
//static inline void HAL_Flash_WriteCmdWriteOnceData(HAL_Flash_T *p_hal_flash, const uint8_t * p_data){HAL_Flash_WriteCmdWriteData(p_hal_flash, p_data);}
//static inline void HAL_Flash_WriteCmdWriteOnceStart(HAL_Flash_T * p_hal_flash){HAL_Flash_WriteCmdStart(p_hal_flash);}
//static inline bool HAL_Flash_ReadCompleteWriteOnceFlag(HAL_Flash_T * p_hal_flash){return HAL_Flash_ReadCompleteFlags(p_hal_flash);}
//static inline void HAL_Flash_PrepCmdWriteOnce(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest, const uint8_t * p_data){}
//
//static inline void HAL_Flash_WriteCmdReadOnceDest(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest)
//{
//	HAL_Flash_WriteCmdWriteOnceDest(p_hal_flash, p_dest);
//}
//static inline void HAL_Flash_WriteCmdReadOnceStart(HAL_Flash_T * p_hal_flash){HAL_Flash_WriteCmdStart(p_hal_flash);}
//static inline bool HAL_Flash_ReadCompleteReadOnceFlag(HAL_Flash_T * p_hal_flash){return HAL_Flash_ReadCompleteFlags(p_hal_flash);}
//static inline void HAL_Flash_ReadOnceData(HAL_Flash_T *p_hal_flash, uint8_t * p_result)
//{
//	for (uint8_t i = 0U; i < FTFx_PHRASE_SIZE; i++)
//	{
//        p_result[i] = ((uint8_t *)FTFx_BASE)[i + 0x08U];
//    }
//}
//static inline void HAL_Flash_PrepCmdReadOnce(HAL_Flash_T * p_hal_flash, const uint8_t * p_dest){}

#endif /* HAL_FLASH_H */


