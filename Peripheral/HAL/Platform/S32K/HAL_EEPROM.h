#ifndef HAL_EEPROM_PLATFORM_H
#define HAL_EEPROM_PLATFORM_H

#include "HAL_Flash.h"

//In order to achieve specified w/e cycle
//endurance, the emulated EEPROM backup size must be at least 16 times the
//emulated EEPROM partition size in FlexRAM.

#define HAL_EEPROM_START			S32K_FLEX_RAM_START
#define HAL_EEPROM_END				S32K_FLEX_RAM_END
#define HAL_EEPROM_SIZE				S32K_FLEX_RAM_SIZE
#define HAL_EEPROM_UNIT_SIZE_ERASE	S32K_FLASH_EEPROM_UNIT_SIZE_ERASE
#define HAL_EEPROM_UNIT_SIZE_WRITE	S32K_FLASH_EEPROM_UNIT_SIZE_WRITE

// flash_flexRam_function_control_code
#define EEE_ENABLE                              (0x00U)    /*!< Make FlexRAM available for emulated EEPROM */
#define EEE_QUICK_WRITE                         (0x55U)    /*!< Make FlexRAM available for EEPROM quick writes */
#define EEE_STATUS_QUERY                        (0x77U)    /*!< EEPROM quick write status query */
#define EEE_COMPLETE_INTERRUPT_QUICK_WRITE      (0xAAU)    /*!< Complete interrupted EEPROM quick write process */
#define EEE_DISABLE                             (0xFFU)    /*!< Make FlexRAM available as RAM */

typedef FTFC_Type HAL_EEPROM_T;	//Flash/EEPROM use same controller

static inline bool HAL_EEPROM_ReadCompleteFlag(HAL_EEPROM_T * p_hal)
{
	return (bool)(FTFx_FCNFG & FTFx_FCNFG_EEERDY_MASK);
}

static inline bool HAL_EEPROM_ReadErrorFlags(HAL_EEPROM_T * p_hal)
{
	return HAL_Flash_ReadErrorFlags(p_hal);
}

static inline void HAL_EEPROM_ClearErrorFlags(HAL_EEPROM_T * p_hal)
{
	return HAL_Flash_ClearErrorFlags(p_hal);
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
*/

static inline void HAL_EEPROM_Init(HAL_EEPROM_T * p_hal)
{
#define S32K_EEERAMSIZE_CODE 	(0x02U)		//only option
#define CONFIG_HAL_EEPROM_S32K_DEPART_CODE 		(0x08U)	 //recommened for max endurance

	uint8_t regDEPartitionCode = (uint8_t)((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK) >> SIM_FCFG1_DEPART_SHIFT);

	if (regDEPartitionCode != CONFIG_HAL_EEPROM_S32K_DEPART_CODE)
	{
		if (HAL_Flash_ReadCompleteFlag(p_hal) == true)
		{
			HAL_EEPROM_ClearErrorFlags(p_hal);

			FTFx_FCCOB0 = FTFx_PROGRAM_PARTITION;
			FTFx_FCCOB1 = 0x00U; //	CSEcKeySize;
			FTFx_FCCOB2 = 0x00U; //(uint8_t)(SFE ? 1U : 0U);
			FTFx_FCCOB3 = 0x01U; //(uint8_t)(flexRamEnableLoadEEEData ? 0U : 1U);
			FTFx_FCCOB4 = S32K_EEERAMSIZE_CODE;					//EEEDataSizeCode = 0x02u: EEPROM size = 4 Kbytes
			FTFx_FCCOB5 = CONFIG_HAL_EEPROM_S32K_DEPART_CODE; 	//DEPartitionCode = 0x08u: EEPROM backup size = 64 Kbytes */
			HAL_Flash_WriteCmdStart(p_hal);

			while (HAL_Flash_ReadCompleteFlag(p_hal) == false)
			{
				if (HAL_EEPROM_ReadErrorFlags(p_hal) == false)
				{
					break;
				}
			}
		}
	}


    if (HAL_Flash_ReadCompleteFlag(p_hal) == true)
	{
    	HAL_EEPROM_ClearErrorFlags(p_hal);

        FTFx_FCCOB0 = FTFx_SET_EERAM;
        FTFx_FCCOB1 = (uint8_t)EEE_QUICK_WRITE;
        FTFx_FCCOB4 = (uint8_t)(0x00U);	//bytes selected for quick writes
        FTFx_FCCOB5 = (uint8_t)(0x00U);
        HAL_Flash_WriteCmdStart(p_hal);

		while (HAL_Flash_ReadCompleteFlag(p_hal) == false)
		{
			if (HAL_EEPROM_ReadErrorFlags(p_hal) == false)
			{
				break;
			}
		}
	}
}


#endif
