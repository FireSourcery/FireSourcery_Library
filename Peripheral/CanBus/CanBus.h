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
	@file 	HAL_CanBus.h
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "HAL_CanBus.h"
#include "CanMessage.h"

#include <stdint.h>
#include <stdbool.h>


//typedef enum
//{
//	CAN_SERVICE_CODE_WRITE_DATA,
//	CAN_SERVICE_CODE_WRITE_CONTROL,
//}
//Protocol_SerivceCode_T;

typedef void (*CanBus_ServiceInit_T)(CanMessage_T * p_messageBuffers);
typedef void (*CanBus_BroadcastFunction_T)(void * p_context, CanMessage_T * p_MessageBufferArray);
typedef void (*CanBus_RxReqFunction_T)(void * p_context, CanMessage_T * p_rxMessageBuffer);

typedef void (*CanBus_RemoteUpdate_T)(void * p_context, CanMessage_T * p_MessageBufferArray);

//typedef const struct
//{
//	CanBus_ServiceFunction_T FUNCTION;
//	uint32_t PERIOD_MS;
//	CanBus_ServiceInit_T INIT;
//}
//CanBus_Service_T;

typedef const struct
{
	CanBus_BroadcastFunction_T BROADCAST;
//	uint8_t BROADCASTS_COUNT;
	uint32_t BROADCAST_PERIOD_MS;
	CanBus_RxReqFunction_T RX_REQ;
//	uint8_t RX_REQS_COUNT;
//	CanBus_ServiceFunction_T SERVICE_FUNCTION; //single service function for now
	CanBus_ServiceInit_T INIT;
	//MESSAGE_CONTROL_TABLE[]
	//BIT_RATE
}
CanBus_Services_T;

typedef const struct
{
	HAL_CanBus_T * const P_HAL_CAN_BUS;
	void * P_APP_CONTEXT; //service and call back context
	const volatile uint32_t * const P_TIMER;
//	uint32_t * const P_TIMER_BUFFERS;
}
CanBus_Config_T;

typedef struct CanBus_Tag
{
	//buffer id map
	const CanBus_Config_T CONFIG;

#define	CONFIG_CAN_BUS_TX_RX_SHARED
#define CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT 4

#ifdef CONFIG_CAN_BUS_TX_RX_SHARED
	CanMessage_T Buffers[CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT];
#elif defined(CONFIG_CAN_BUS_TX_RX_INDIVIDUAL)
	CanMessage_T TxBuffers[CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT/2]; //match per hw buffer
	CanMessage_T RxBuffers[CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT/2];
#endif

//	void (*RxCompleteCallback)(struct CanBus_Tag * p_this);
//	void (*TxCompleteCallback)(struct CanBus_Tag * p_this);
//	void (*ErrorCallback)(struct CanBus_Tag * p_this);

#if CONFIG_CAN_BUS_DMA_ENABLE
    uint8_t RxFifoDmaChannel;
#endif
//    CanBus_TransferMode_T RxTransferMode;

    CanBus_Services_T * p_Services;
    uint32_t BroadcastTimeSaved;
//    uint32_t BroadcastPeriod;//if configuratble is needed

} CanBus_T;

#define CAN_BUS_CONFIG(p_Hal, p_App, p_Timer)		\
{													\
	.CONFIG = 										\
	{												\
		.P_HAL_CAN_BUS = p_Hal, 					\
		.P_APP_CONTEXT = p_App, 					\
		.P_TIMER = p_Timer, 						\
	},												\
}




/*
 * Abstraction layer must use to implementation for shared and independent rxTx buffers,
 * due to remote txrx behavior
 */
static inline void CanBus_TxRx_ISR(CanBus_T * p_can)
{
	uint8_t hwBufferIndex;
	uint8_t bufferId = -1;

	for(uint8_t iBuffer = 0; iBuffer < CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT; iBuffer++)
	{
		hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, iBuffer);
		if(HAL_CanBus_ReadIsBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex) == true)
		{
			bufferId = iBuffer;
			break;
		}
	}

	if(bufferId != -1)
	{
		if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_RX_WAIT_DATA)
		{
			if(HAL_CanBus_LockRxBuffer(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex) == true)
			{
				HAL_CanBus_ReadRxMessageBuffer(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, &p_can->Buffers[bufferId]);
				HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
				HAL_CanBus_UnlockRxBuffer(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);

				p_can->Buffers[bufferId].Status = CAN_MESSAGE_RX_WAIT_SERVICE;
				//rx into buffer then wait for protocol service, double buffering? or call handler directly?
				//if recv  once,  disable
			}
			else
			{
				//check repeat error
			}

		}
		else if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_RX_WAIT_REMOTE)
		{
//			if(HAL_CanBus_ReadIsBufferrRxRemoteTxEmpty(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex) == true)
//			{
// 				//remote recived
//			}
//			else
//			{
//
//			}
		}
		else if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_REMOTE)
		{
			if(HAL_CanBus_ReadIsBufferTxRemoteRxEmpty(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex) == true) //or wait tx remote, rx empty
			{
				//interrupt is from tx complete
				HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
				p_can->Buffers[bufferId].Status = CAN_MESSAGE_RX_WAIT_DATA;
				//ensure rx enabled if not autoset
//				HAL_CanBus_EnableRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
			}
			else //rx full
			{
				//interrupt is from tx remote response received
				p_can->Buffers[bufferId].Status = CAN_MESSAGE_RX_WAIT_DATA;
			}
		}
		else //if(p_can->Buffers[bufferId].Status != CAN_MESSAGE_RX_WAIT)
		{
			HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
			HAL_CanBus_DisableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
			p_can->Buffers[bufferId].Status = CAN_MESSAGE_IDLE;
		}
//		else if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE) //or init
//		{
//			//error  or ISR without message set interrupt
//			HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
//			HAL_CanBus_DisableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
//		}
	}
	else
	{
		//error ISR without flag set,
	}
}

static inline void CanBus_Tx_ISR(CanBus_T * p_can)
{
//	if(p_can->TxBuffers[bufferId].Status == CAN_MESSAGE_TX_REMOTE)
//	{
//		if(HAL_CanBus_ReadTxBufferWaitRemote(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
//		{
//			HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//			//ensure rx enabled
//			HAL_CanBus_EnableRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//		}
//	}
//	else if (p_can->TxBuffers[bufferId].Status == CAN_MESSAGE_TX_DATA)
//	{
//		HAL_CanBus_ClearTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//	}
//	else if (p_can->TxBuffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
//		//clear error state
//
//	}
//
//	p_can->TxBuffers[bufferId].Status = CAN_MESSAGE_IDLE;
//	HAL_CanBus_DisableTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//	if (p_can->TxCompleteCallback != 0U) p_can->TxCompleteCallback(p_can->p_CallbackContext);
}


static inline void CanBus_Rx_ISR(CanBus_T * p_can)
{
//	if(p_can->RxBuffers[bufferId].Status == CAN_MESSAGE_RX_BUSY)
//	{
//		if(HAL_CanBus_LockRxBuffer(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
//		{
//			HAL_CanBus_ReadRxMessageBuffer(p_can->CONFIG.P_HAL_CAN_BUS, &p_can->RxBuffers[bufferId], bufferId);
//			HAL_CanBus_ClearRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//			HAL_CanBus_UnlockRxBuffer(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//		}
//	}
//	else if (p_can->RxBuffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
//		//clear error state
//		HAL_CanBus_ClearRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//	}
//	CanBus_TxRx_Buffer_ISR(p_can, rxBufferId);
//	if (p_can->RxCompleteCallback != 0U) p_can->RxCompleteCallback(p_can->p_CallbackContext);
}





#endif
