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
	@file 	Xcvr.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Xcvr.h"

void Xcvr_Init(Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
	if(Xcvr_SetXcvr(p_xcvr, xcvrIndex) != true)
	{
		Xcvr_SetXcvr(p_xcvr, 0U);
	}

//	switch(p_xcvr->Type)
//	{
//		case XCVR_TYPE_SERIAL:	Serial_Init(p_xcvr->p_Xcvr);	break;
//		case XCVR_TYPE_I2C: 		break;
//		case XCVR_TYPE_SPI: 		break;
//		case XCVR_TYPE_VIRTUAL: 	break;
//		default: break;
//	}
}

bool Xcvr_SetXcvr(Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
	bool status;

	if (xcvrIndex < p_xcvr->CONFIG.XCVR_COUNT)
	{
		p_xcvr->p_Xcvr = p_xcvr->CONFIG.P_XCVR_TABLE[xcvrIndex].P_XCVR;
		p_xcvr->Type = p_xcvr->CONFIG.P_XCVR_TABLE[xcvrIndex].TYPE;
		status = true;
	}
	else
	{
		status = false;
	}

	return status;
}

bool Xcvr_SetXcvr_Ptr(Xcvr_T * p_xcvr, void * p_xcvrStruct)
{
	bool status;

//	if(Xcvr_CheckValid(p_xcvr, *p_xcvrStruct) != 0xFF) //return id
//	{
//		p_xcvr->p_Xcvr = p_xcvrStruct;
//		p_xcvr->Type = p_xcvr->CONFIG.P_XCVR_TABLE[xcvrIndex].TYPE;
//	}

	return status;
}

bool Xcvr_CheckIsSet(const Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
	return ((xcvrIndex < p_xcvr->CONFIG.XCVR_COUNT) && (p_xcvr->p_Xcvr == p_xcvr->CONFIG.P_XCVR_TABLE[xcvrIndex].P_XCVR));
}

bool Xcvr_CheckValid(const Xcvr_T * p_xcvr, void * p_target)
{
	bool isValid = false;

	for (uint8_t iXcvr = 0; iXcvr < p_xcvr->CONFIG.XCVR_COUNT; iXcvr++)
	{
		if (p_target == p_xcvr->CONFIG.P_XCVR_TABLE[iXcvr].P_XCVR)
		{
			isValid = true;
			break;
		}
	}

	return isValid;
}

void Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate)
{
	switch(p_xcvr->Type)
	{
		case XCVR_TYPE_SERIAL:	Serial_ConfigBaudRate(p_xcvr->p_Xcvr, baudRate);	break;
		case XCVR_TYPE_I2C: 		break;
		case XCVR_TYPE_SPI: 		break;
		case XCVR_TYPE_VIRTUAL: 	break;
		default: break;
	}
}

bool Xcvr_Tx(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t length)
{
	bool status;

	switch(p_xcvr->Type)
	{
		case XCVR_TYPE_SERIAL:	status = Serial_Send(p_xcvr->p_Xcvr, p_srcBuffer, length);	break;
		case XCVR_TYPE_I2C: 		break;
		case XCVR_TYPE_SPI: 		break;
		case XCVR_TYPE_VIRTUAL: 	break;
		default: break;
	}

	return status;
}

uint32_t Xcvr_Rx(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t length)
{
	uint32_t rxCount;

	switch(p_xcvr->Type)
	{
		case XCVR_TYPE_SERIAL:	rxCount = Serial_Recv(p_xcvr->p_Xcvr, p_destBuffer, length);	break;
		case XCVR_TYPE_I2C: 		break;
		case XCVR_TYPE_SPI: 		break;
		case XCVR_TYPE_VIRTUAL: 	break;
		default: break;
	}

	return rxCount;
}

uint32_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr)
{
	uint32_t count;

	switch(p_xcvr->Type)
	{
		case XCVR_TYPE_SERIAL: 	count = Serial_GetRxFullCount(p_xcvr->p_Xcvr);	break;
		case XCVR_TYPE_I2C: 		break;
		case XCVR_TYPE_SPI: 		break;
		case XCVR_TYPE_VIRTUAL: 	break;
		default: break;
	}

	return count;
}

uint32_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr)
{
	uint32_t count;

	switch(p_xcvr->Type)
	{
		case XCVR_TYPE_SERIAL: 	count = Serial_GetTxEmptyCount(p_xcvr->p_Xcvr);	break;
		case XCVR_TYPE_I2C: 		break;
		case XCVR_TYPE_SPI: 		break;
		case XCVR_TYPE_VIRTUAL: 	break;
		default: break;
	}

	return count;
}


