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
	@file 	Datagram.c
	@author FireSoucery
	@brief	Datagram mode, continuous transmission
	@version V0
*/
/******************************************************************************/
#include "Datagram.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//void Datagram_Build(Datagram_T * p_datagram, uint8_t * p_packet, uint8_t * p_size)
void Datagram_Build(Datagram_T * p_datagram)
{
	size_t packetOffset = p_datagram->HeaderSize; //header should always be constant size

	size_t fullCount = 0U;

//	for (uint8_t iVar = p_datagram->ActiveVar; iVar < p_datagram->DatagramVarCount; iVar++)
//	{
//		if(p_datagram->P_VAR_TABLE[iVar].VarSize + fullCount > p_datagram->TxDataSize)
//		{
//			memcpy(p_datagram->P_TX_BUFFER + packetOffset, p_datagram->P_VAR_TABLE[iVar].p_Var, p_datagram->TxDataSize - fullCount);
//			p_datagram->ActiveVar = iVar;
//			p_datagram->ActiveVarByte = p_datagram->TxDataSize - fullCount;
//			fullCount = p_datagram->TxDataSize;
//			packetOffset = fullCount + p_datagram->HeaderSize;
//			break;
//		}
//		else
//		{
//			memcpy(p_datagram->P_TX_BUFFER  + packetOffset, p_datagram->P_VAR_TABLE[iVar].p_Var, p_datagram->P_VAR_TABLE[iVar].VarSize);
//			fullCount += p_datagram->P_VAR_TABLE[iVar].VarSize;
//			packetOffset += p_datagram->P_VAR_TABLE[iVar].VarSize;
//
//
//
//			if(iVar == p_datagram->DatagramVarCount - 1U)
//			{
//				//flag on cycle
//			}
//		}
//	}
//
//	 p_datagram->TxDataSizeCurrent = fullCount;
//
//	if (p_datagram->BUILD_HEADER != 0U)
//	{
//		p_datagram->BUILD_HEADER(p_datagram->P_TX_BUFFER , packetOffset - p_datagram->DATAGRAM_HEADER_SIZE);
//	}


//	*p_size = packetOffset;

//	DataLinkTxPacket(p_datagram);
}

void Datagram_SetVar(Datagram_T * p_datagram, uint8_t varIndex, void * p_var, size_t varSize)
{
	if ((varSize < p_datagram->VAR_SIZE_MAX) && (varIndex < p_datagram->VAR_TABLE_LENGTH))
	{
		p_datagram->P_VAR_TABLE[varIndex].p_Var = p_var;
		p_datagram->P_VAR_TABLE[varIndex].VarSize = varSize;
	}
}

void Datagram_SetVarTable(Datagram_T * p_datagram, Datagram_Entry_T * p_varTable, uint8_t varCount)
{
	uint8_t iVar;

	for (iVar = 0U; (iVar < varCount) && (iVar < p_datagram->VAR_TABLE_LENGTH); iVar++)
	{
		Datagram_SetVar(p_datagram, iVar, p_varTable[iVar].p_Var, p_varTable[iVar].VarSize);
	}

	p_datagram->DatagramVarCount = iVar;
}




uint8_t * Datagram_GetPtr(Datagram_T * p_datagram)
{
	return  p_datagram->P_TX_BUFFER;
}

void Datagram_SetCmdData(Datagram_T * p_datagram, uint8_t * p_source, uint8_t size)
{
	p_datagram->DatagramModeEnable = true;
}


void Datagram_Enable(Datagram_T * p_datagram)
{
	p_datagram->DatagramModeEnable = true;
}


void Datagram_Signal(Datagram_T * p_datagram)
{
//	p_datagram->Signal++;
}

/*
 full packet with header
 */
size_t Datagram_GetPacketSize(Datagram_T * p_datagram)
{
	return  p_datagram->HeaderSize + p_datagram->TxDataSizeActive;
}

/*
 * Protocol Side call
 */
bool Datagram_Server_Proc(Datagram_T * p_datagram)
{
//	bool isComplete = false;
//
//	switch (p_datagram->State)
//	{
//		case DATAGRAM_STATE_ACTIVE:
//			if (p_datagram->Signal > 0U)
//			{
//				if (*p_datagram->P_TIMER - p_datagram->TxTime > p_datagram->TX_PERIOD_MIN)
//				{
//					p_datagram->TxTime = *p_datagram->P_TIMER;
//					p_datagram->Signal--;
//
//					Datagram_Build(p_datagram);
//
//
//					isComplete = true;
//				}
//			}
//			break;
//
//		case DATAGRAM_STATE_INACTIVE:
//			break;
//
//		default: break;
//	}
//
//	return isComplete;
}


bool Datagram_Client_Proc(Datagram_T * p_datagram)
{

}

//bool Datagram_Client_CaptureVar(Datagram_T * p_datagram, uint8_t varIndex, void * p_var, size_t varSize)
//{
//
//}
