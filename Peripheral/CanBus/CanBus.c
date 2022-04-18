#include "CanBus.h"
#include "CanMessage.h"

#include <stdint.h>
#include <stdbool.h>

void CanBus_Init(CanBus_T * p_can, const CanBus_Services_T * p_services)
{
	p_can->p_Services = p_services;
	CanBus_InitServices(p_can);
}

void CanBus_InitServices(CanBus_T * p_can)
{
	uint8_t hwBufferIndex;
	HAL_CanBus_DisableInterrupts(p_can->CONFIG.P_HAL_CAN_BUS);
	if(p_can->p_Services->INIT != 0U)
	{
		p_can->p_Services->INIT(&p_can->Buffers[0U]);

		for(uint8_t iBufferId = 0U; iBufferId < CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT; iBufferId++)
		{
			if((p_can->Buffers[iBufferId].Status == CAN_MESSAGE_RX_INIT) || (p_can->Buffers[iBufferId].Status == CAN_MESSAGE_TX_INIT))
			{
				hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, iBufferId);

				HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
				HAL_CanBus_WriteMessageBufferControl(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, &p_can->Buffers[iBufferId]);

				if(p_can->Buffers[iBufferId].Status == CAN_MESSAGE_RX_INIT)
				{

					p_can->Buffers[iBufferId].Status == CAN_MESSAGE_RX_WAIT;
					HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
				}
				else if(p_can->Buffers[iBufferId].Status == CAN_MESSAGE_TX_INIT)
				{
					p_can->Buffers[iBufferId].Status == CAN_MESSAGE_IDLE;
				}
			}
		}
	}
	HAL_CanBus_EnableInterrupts(p_can->CONFIG.P_HAL_CAN_BUS);
}

void CanBus_ProcServices(CanBus_T * p_can)
{
	uint8_t hwBufferIndex;

	HAL_CanBus_DisableInterrupts(p_can->CONFIG.P_HAL_CAN_BUS);

	if(*p_can->CONFIG.P_TIMER > p_can->BroadcastTimeSaved + p_can->p_Services->BROADCAST_PERIOD_MS)
	{
		p_can->BroadcastTimeSaved = *p_can->CONFIG.P_TIMER;

//		for(uint8_t iBroad = 0U; i < p_can->p_Services->BROADCASTS_COUNT; i++)
//		{
		p_can->p_Services->BROADCAST(p_can->CONFIG.P_APP_CONTEXT, &p_can->Buffers[0U]);
//		}
	}

	for(uint8_t iBufferId = 0U; iBufferId < CONFIG_CAN_BUS_MESSAGE_BUFFER_COUNT; iBufferId++)
	{
		if(p_can->Buffers[iBufferId].Status == CAN_MESSAGE_RX_COMPLETE)
		{
			p_can->p_Services->RX_REQ(p_can->CONFIG.P_APP_CONTEXT, &p_can->Buffers[iBufferId]);
		}

		//set from broadcast
		else if(p_can->Buffers[iBufferId].Status == CAN_MESSAGE_TX_DATA)
		{
			hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, iBufferId);
			HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
//			HAL_CanBus_WriteTxMessageBufferData(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, &p_can->Buffers[iBufferId].Data[0U], p_can->Buffers[iBufferId].DataLength);
//			HAL_CanBus_WriteTxMessageBufferStart(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);

			HAL_CanBus_WriteTxMessageBuffer(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, &p_can->Buffers[iBufferId]);
			HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
		}
//		//todo common fucntion
//		else if(p_can->Buffers[iBufferId].Status == CAN_MESSAGE_RX_INIT)
//		{
//			p_can->Buffers[iBufferId].Status == CAN_MESSAGE_RX_WAIT;
//			HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
//		}
//		else if(p_can->Buffers[iBufferId].Status == CAN_MESSAGE_TX_INIT)
//		{
//			p_can->Buffers[iBufferId].Status == CAN_MESSAGE_IDLE;
//		}
	}

	HAL_CanBus_EnableInterrupts(p_can->CONFIG.P_HAL_CAN_BUS);
}

void CanBus_ConfigBitRate(CanBus_T * p_can, uint32_t bitRate)
{
	HAL_CanBus_ConfigBitRate(p_can->CONFIG.P_HAL_CAN_BUS, bitRate);
}

/*
 *
 */
//void CanBus_TxMessage(CanBus_T * p_can, uint8_t bufferId)
//{
//	uint8_t hwBufferIndex;
//
//	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
////		HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//		if (p_can->Buffers[bufferId].Type == CAN_FRAME_TYPE_REMOTE)
//		{
//			p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_REMOTE;
//		}
//		else
//		{
//			p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_DATA;
//		}
//		hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, iBufferId);
//		HAL_CanBus_WriteTxMessageBuffer(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, &p_can->Buffers[bufferId]);
//		HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
////	    p_can->IsTxComplete = false;
//	}
//
////	return status;
//}
//
//
//void CanBus_WriteTxMessageControl(CanBus_T * p_can, uint8_t bufferId)
//{
//	uint8_t hwBufferIndex;
//
//	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
////		HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//		if (p_can->Buffers[bufferId].Type == CAN_FRAME_TYPE_REMOTE)
//		{
//			p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_REMOTE;
//		}
//		else
//		{
//			p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_DATA;
//		}
//		hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS, iBufferId);
//		HAL_CanBus_WriteTxMessageBufferControl(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, &p_can->Buffers[bufferId]);
//		HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
////	    p_can->IsTxComplete = false;
//	}
//}
//
//void CanBus_StartTxData(CanBus_T * p_can, uint8_t bufferId, const uint8_t * p_data, uint8_t dataLength)
//{
////	CanMessage_Status_T status;
//	uint8_t hwBufferIndex;
//	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
////		HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//		p_can->Buffers[bufferId].Status == CAN_MESSAGE_TX_DATA;
//		hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS,  bufferId);
//		HAL_CanBus_WriteTxMessageBufferData(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex, p_data, dataLength);
//		HAL_CanBus_WriteTxMessageBufferStart(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
//		HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
////	    p_can->IsTxComplete = false;
//	}
//
////	return  p_can->Buffers[bufferId].Status;
//}
//
//void CanBus_SetRxMessageControl(CanBus_T * p_can, uint8_t bufferId )
//{
//	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
//
//	}
//	//deactiveate, config
////	HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//
//}
//
///*
// * config and rx
// */
//void CanBus_WriteRxMessage(CanBus_T * p_can, uint8_t bufferId )
//{
//	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
//
//	}
//	//deactiveate, config
////	HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//
//}
//
//void CanBus_StartRx(CanBus_T * p_can, uint8_t bufferId)
//{
//	uint8_t hwBufferIndex;
//
//	if(p_can->Buffers[bufferId].Status == CAN_MESSAGE_IDLE)
//	{
//		p_can->Buffers[bufferId].Status == CAN_MESSAGE_RX_WAIT;
//		hwBufferIndex = HAL_CanBus_CalcMessageBufferIndex(p_can->CONFIG.P_HAL_CAN_BUS,  bufferId);
//		HAL_CanBus_EnableBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, hwBufferIndex);
//	}
//}
//
//
//void CanBus_GetRxData(CanBus_T * p_can, uint8_t bufferId)
//{
//	//deactiveate, config
//
//	HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//
//}
//
//void CanBus_GetPtrRxData(CanBus_T * p_can,   uint8_t bufferId)
//{
//	//deactiveate, config
//
//	HAL_CanBus_ClearBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//
//}


//const uint8_t * CanBus_GetRxMessageBufferData(const CanBus_T * p_can, uint8_t bufferId)
//{
//	return p_can->RxBuffers[bufferId].Data;
//}
//
//void CanBus_GetRxMessageId(CanBus_T * p_can )
//{
//
//}





