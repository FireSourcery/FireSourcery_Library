/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/
#include "MotorFlash.h"


#include "Peripheral/Flash/Flash_EEPROM.h"
#include "Peripheral/Flash/Flash.h"

Flash_T MotorFlashMain;

void MotorFlash_Init //per app
(
	HAL_Flash_T * p_hal_flash
)
{
	Flash_Init
	(
		&MotorFlashMain,
		p_hal_flash,
		MotorFlash_OnBlock,
		0U
	);
}

// void MotorFlash_InitParition //per motor
//(
//	Motor_T * p_motorDest,
//	const Flash_Partition_T * p_flashParition
//)
//{
////	p_motor->p_MotorFlashParition = p_flashParition;
//}

