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
//	CAN_BUS_MESSAGE_BUFFER_IDLE,
//	CAN_BUS_MESSAGE_BUFFER_RX_BUSY,
//	CAN_BUS_MESSAGE_BUFFER_TX_BUSY,
//	CAN_BUS_MESSAGE_BUFFER_RX_FIFO_BUSY,
//	CAN_BUS_MESSAGE_BUFFER_COMPLETE,
//	CAN_BUS_MESSAGE_BUFFER_TX_REMOTE,
//	CAN_BUS_MESSAGE_BUFFER_RX_REMOTE,
//#if CONFIG_CAN_BUS_DMA_ENABLE
//	CAN_BUS_MESSAGE_BUFFER_DMA_ERROR
//#endif
//} CanBus_MessageBufferStatus_T;

typedef const struct
{
	HAL_CanBus_T * const P_HAL_CAN_BUS;
}
CanBus_Config_T;

#define CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT 4

typedef struct CanBus_Tag
{
	//buffer id map
	const CanBus_Config_T CONFIG;
	union
	{
		struct
		{
			CanMessage_T TxBuffers[CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT/2]; //match per hw buffer
			CanMessage_T RxBuffers[CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT/2];
		};
		CanMessage_T Buffers[CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT];
	};

	//	CanBus_MessageBufferStatus_T MessageBufferStatus[4]; //CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT
//	CanBus_MessageBufferStatus_T TxBufferStatus;
//	CanBus_MessageBufferStatus_T RxBufferStatus;
	void (*RxCompleteCallback)(struct CanBus_Tag * p_this);
	void (*TxCompleteCallback)(struct CanBus_Tag * p_this);
	void (*ErrorCallback)(struct CanBus_Tag * p_this);

#if CONFIG_CAN_BUS_DMA_ENABLE
    uint8_t RxFifoDmaChannel;
#endif
//    CanBus_TransferMode_T RxTransferMode;
} CanBus_T;

#define CAN_BUS_CONFIG(p_Hal )	\
{																\
	.CONFIG = {.P_HAL_CAN_BUS = p_Hal, },						\
}

static inline void CanBus_TxRx_Buffer_ISR(CanBus_T * p_can, uint8_t bufferId)
{
#ifdef CONFIG_CAN_BUS_HW_BUFFER_INDEX_ENABLE
	uint8_t hwBufferIndex = HAL_CanBus_CalcRxMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
#endif

	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_RX_BUSY)
	{
		if(HAL_CanBus_LockRxBuffer(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
		{
			HAL_CanBus_ReadRxMessageBuffer(p_can->CONFIG.P_HAL_CAN_BUS, &p_can->RxBuffers[bufferId], bufferId);
			HAL_CanBus_ClearRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
			HAL_CanBus_UnlockRxBuffer(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
		}
	}
	else if (p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
	{
		//clear error state
		HAL_CanBus_ClearRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
	}
   //if recv  once,
   //diable

	//was tx interrupt
	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_REMOTE)
	{
//		if(HAL_CanBus_ReadTxBufferWaitRemote(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
		{
			HAL_CanBus_ClearTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
			//ensure rx enabled
			HAL_CanBus_EnableRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
		}
	}
	else if (p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_DATA)
	{
		HAL_CanBus_ClearTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
	}
	else if (p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
	{
		//clear error state
	}

	p_can->Buffers[bufferId].Status = CAN_MESSAGE_IDLE;
	HAL_CanBus_DisableTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);

}

static inline uint8_t GetCanBusInterruptBufferId(CanBus_T * p_can)
{
	uint8_t bufferId = -1;
	uint8_t hwBufferIndex;
#define CONFIG_CAN_BUS_INTERRUPT_SHARED

#ifdef CONFIG_CAN_BUS_INTERRUPT_SHARED
	for(uint8_t iBuffer = 0; iBuffer < CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT; iBuffer++)
	{
		hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, iBuffer);
		if(HAL_CanBus_ReadIsBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex) == true)
		{
			bufferId = iBuffer;
			break;
		}
	}
#elif defined(CONFIG_CAN_BUS_INTERRUPT_INDIVIDUAL)

#endif

	return bufferId;
}

static inline void CanBus_TxRx_ISR(CanBus_T * p_can)
{
	uint8_t bufferId = GetCanBusInterruptBufferId(p_can);

	if(bufferId != -1)
	{
		CanBus_TxRx_Buffer_ISR(p_can, bufferId);
	}
}

static inline void CanBus_Tx_ISR(CanBus_T * p_can)
{
//	if(p_can->TxBuffers[bufferId].Status == CAN_MESSAGE_TX_REMOTE)
//	{
//		if(HAL_CanBus_ReadTxBufferWaitRemote(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
//		{
//			HAL_CanBus_ClearTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
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

//static inline bool CanBus_GetIsRxComplete(CanBus_T * p_can)
//{
//    return (p_can->IsRxComplete);
//}
//
//static inline bool CanBus_GetIsTxComplete(CanBus_T * p_can)
//{
//    return (p_can->IsTxComplete);
//}

//void ProcessCANRx(void)
//{
//	if (CANController_IsRxComplete(&CANController1))
//	{
//		ParseRxPacket(CANController_GetRxFrame(&CANController1));
//		MotorPWM = GetControllerThrottle(); //PacketThrottle * 512 / 1000;
//
//		//UpdatePWM();
//		CANController_StartRxFrame(&CANController1); //start next frame
//	}
//}
//void CanBus_ProcRxPoll(CanBus_T * p_can)
//{
//	 uint32_t * p_id;
//	 uint8_t * p_data;
//
//	 if (CanBus_IsRxComplete(p_can))
//	 {
//		 CanBus_GetRxFrame(p_can, p_id, p_data);
//		 //parse;
//		 //table function;
//		 CanBus_ReceiveFrame(p_can); //next frame
//	 }
//}
#endif
