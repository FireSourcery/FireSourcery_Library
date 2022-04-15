#include "CanBus.h"
#include "CanMessage.h"

#include <stdint.h>
#include <stdbool.h>


void CanBus_Init(CanBus_T * p_can)
{

}

void CanBus_ConfigBaudRate(CanBus_T * p_can, uint32_t baudRate)
{
	HAL_CanBus_ConfigBaudRate(p_can->CONFIG.P_HAL_CAN_BUS, baudRate);
}

/*
 *
 */
void CanBus_SendMessage(CanBus_T * p_can, const CanMessage_T * p_message)
{
//	status_t status;
	//if (p_can->TxState != CAN_STATE_IDLE) return;
	uint8_t bufferId = 0U; //get buffer id

//	 HAL_CanBus_CheckTxMessageBufferId(HAL_CanBus_T * p_hal, uint8_t bufferId)
//	        /* Check if the Payload Size is smaller than the payload configured */
//	        DEV_ASSERT((uint8_t)cs->dataLen <= FLEXCAN_GetPayloadSize(base));

	if(HAL_CanBus_ReadTxBufferComplete(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
	{
		HAL_CanBus_ClearTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);

	//	if (HAL_CanBus_ReadTxBufferAvailable(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)

//		if (p_message.Type = CAN_FRAME_TYPE_REMOTE)
//		{
//			p_can->TxState = CAN_STATE_TX_REMOTE;
//		}
//		else
//		{
//			p_can->TxState = CAN_STATE_TX_DATA;
//		}
		HAL_CanBus_WriteTxMessageBuffer(p_can->CONFIG.P_HAL_CAN_BUS, p_message, bufferId);
		HAL_CanBus_EnableTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);

//	    p_can->IsTxComplete = false;
	}
//	return status;
}

//void CanBus_SendData(CanBus_T * p_can, const uint8_t * p_data, uint8_t dataLength, uint8_t bufferId)
//{
////	status_t status;
//
//    //if (p_can->TxState != CAN_STATE_IDLE) return;
//	uint8_t bufferId = 0U;
//
//	if (HAL_CanBus_ReadTxBufferComplete(p_can->CONFIG.P_HAL_CAN_BUS) == true)
//	{
//		HAL_CanBus_WriteTxMessageBufferData(p_can->CONFIG.P_HAL_CAN_BUS, p_data, dataLength, bufferId);
//		HAL_CanBus_WriteTxMessageBufferStart(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
//		HAL_CanBus_EnableTxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS);
////	    p_can->IsTxComplete = false;
//	}
////	return status;
//}


void CanBus_SetRecvMessage(CanBus_T * p_can, uint8_t messageId, uint8_t bufferId)
{
	//deactiveate, config

		HAL_CanBus_ClearRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);

}

void CanBus_StartRecvMessage(CanBus_T * p_can, uint8_t bufferId)
{
	if(HAL_CanBus_ReadRxBufferComplete(p_can->CONFIG.P_HAL_CAN_BUS, bufferId) == true)
	{
		HAL_CanBus_EnableRxBufferInterrupt(p_can->CONFIG.P_HAL_CAN_BUS, bufferId);
	}
}

const uint8_t * CanBus_GetRxMessageBufferData(const CanBus_T * p_can, uint8_t bufferId)
{
	return p_can->RxBuffers[bufferId].Data;
}

void CanBus_GetRxMessageId(CanBus_T * p_can )
{

}



//void CanBus_StartRxFrame(CanBus_T * p_can)
//{
//    if (p_can->RxState != CAN_STATE_IDLE) return;
//
//	p_can->RxState = CAN_STATE_RX_DATA;
//	HAL_CanBus_EnableRxBufferFullInterrupt();
//	p_can->IsRxComplete = false;
//}


