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
#ifndef HAL_CAN_BUS_PLATFORM_H

#define HAL_CAN_BUS_PLATFORM_H

#include "../../../../../[Workspace]/CanBus/CanMessage.h"
#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

typedef CAN_Type HAL_CanBus_T;



static const volatile uint32_t * GetPtrMessageBuffer(HAL_CanBus_T * p_hal, uint32_t msgBuffIdx)
{
    uint8_t payload_size = 8U;
    uint8_t arbitration_field_size = 8U;
    uint32_t ramBlockSize = 512U;
    uint32_t ramBlockOffset;

    uint8_t mb_size = (uint8_t)(payload_size + arbitration_field_size);
    uint8_t maxMbNum = (uint8_t)(ramBlockSize / mb_size);

    ramBlockOffset = 128U * (msgBuffIdx / (uint32_t)maxMbNum);

    /* Multiply the MB index by the MB size (in words) */
    uint32_t mb_index = ramBlockOffset + ((msgBuffIdx % (uint32_t)maxMbNum) * ((uint32_t)mb_size >> 2U));

    return &p_hal->RAMn[mb_index];
}

#define CAN_DLC_VALUE_12_BYTES                   9U
#define CAN_DLC_VALUE_16_BYTES                   10U
#define CAN_DLC_VALUE_20_BYTES                   11U
#define CAN_DLC_VALUE_24_BYTES                   12U
#define CAN_DLC_VALUE_32_BYTES                   13U
#define CAN_DLC_VALUE_48_BYTES                   14U
#define CAN_DLC_VALUE_64_BYTES                   15U

static uint8_t GetDataLengthMax(uint8_t dlcValue)
{
	uint8_t ret = 0U;

	if(dlcValue <= 8U)
	{
		ret = dlcValue;
	}
	else
	{
		switch(dlcValue)
		{
			case CAN_DLC_VALUE_12_BYTES: ret = 12U; break;
			case CAN_DLC_VALUE_16_BYTES: ret = 16U; break;
			case CAN_DLC_VALUE_20_BYTES: ret = 20U; break;
			case CAN_DLC_VALUE_24_BYTES: ret = 24U; break;
			case CAN_DLC_VALUE_32_BYTES: ret = 32U; break;
			case CAN_DLC_VALUE_48_BYTES: ret = 48U; break;
			case CAN_DLC_VALUE_64_BYTES: ret = 64U; break;
			default: break;	/* The argument is not a valid DLC size */
		}
	}

    return ret;
}

static uint8_t GetDataLengthCode(uint8_t dataLength)
{
    static const uint8_t DLC[65U] = {
		0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U,
			/* 9 to 12 payload have DLC Code 12 Bytes */
		CAN_DLC_VALUE_12_BYTES, CAN_DLC_VALUE_12_BYTES, CAN_DLC_VALUE_12_BYTES, CAN_DLC_VALUE_12_BYTES,
			/* 13 to 16 payload have DLC Code 16 Bytes */
		CAN_DLC_VALUE_16_BYTES, CAN_DLC_VALUE_16_BYTES, CAN_DLC_VALUE_16_BYTES, CAN_DLC_VALUE_16_BYTES,
			/* 17 to 20 payload have DLC Code 20 Bytes */
		CAN_DLC_VALUE_20_BYTES, CAN_DLC_VALUE_20_BYTES, CAN_DLC_VALUE_20_BYTES, CAN_DLC_VALUE_20_BYTES,
			/* 21 to 24 payload have DLC Code 24 Bytes */
		CAN_DLC_VALUE_24_BYTES, CAN_DLC_VALUE_24_BYTES, CAN_DLC_VALUE_24_BYTES, CAN_DLC_VALUE_24_BYTES,
			/* 25 to 32 payload have DLC Code 32 Bytes */
		CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES,
		CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES, CAN_DLC_VALUE_32_BYTES,
			/* 33 to 48 payload have DLC Code 48 Bytes */
		CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
		CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
		CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
		CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES, CAN_DLC_VALUE_48_BYTES,
			/* 49 to 64 payload have DLC Code 64 Bytes */
		CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES,
		CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES,
		CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES,
		CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES, CAN_DLC_VALUE_64_BYTES };
    uint32_t ret;

    if (dataLength < 65U)
    {
        ret = DLC[dataLength];
    }
    else
    {
    	ret = 0xFFU;   /* The argument is not a valid payload size will return 0xFF*/
    }

    return (uint8_t)ret;
}

static inline bool HAL_CanBus_CheckTxMessageBufferId(HAL_CanBus_T * p_hal, uint8_t bufferId)
{
	//	uint32_t val1, val2 = 1;

	//	    if (msgBuffIdx > (((p_canBus->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT) )
	//	    {
	//	        stat = STATUS_CAN_BUFF_OUT_OF_RANGE;
	//	    }


	/* Check if RX FIFO is enabled*/
//	if(((p_canBus->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U)
//	{
//		/* Get the number of RX FIFO Filters*/
//		val1 = (((p_canBus->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
//		/* Get the number if MBs occupied by RX FIFO and ID filter table*/
//		/* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
//		/* Every number of RFFN means 8 number of RX FIFO filters*/
//		/* and every 4 number of RX FIFO filters occupied one MB*/
//		val2 = RxFifoOcuppiedLastMsgBuff(val1);
//
////		if(msgBuffIdx <= val2)
////		{
////			stat = STATUS_CAN_BUFF_OUT_OF_RANGE;
////		}
//	}
}

static inline bool HAL_CanBus_ReadTxMessageBufferSize(HAL_CanBus_T * p_hal, uint8_t bufferId)
{
}


#define CAN_ID_EXT_MASK                          0x3FFFFu
#define CAN_ID_EXT_SHIFT                         0
#define CAN_ID_EXT_WIDTH                         18
#define CAN_ID_STD_MASK                          0x1FFC0000u
#define CAN_ID_STD_SHIFT                         18
#define CAN_ID_STD_WIDTH                         11
#define CAN_ID_PRIO_MASK                         0xE0000000u
#define CAN_ID_PRIO_SHIFT                        29
#define CAN_ID_PRIO_WIDTH                        3
/* CS Bit Fields */
#define CAN_CS_TIME_STAMP_MASK                   0xFFFFu
#define CAN_CS_TIME_STAMP_SHIFT                  0
#define CAN_CS_TIME_STAMP_WIDTH                  16
#define CAN_CS_DLC_MASK                          0xF0000u
#define CAN_CS_DLC_SHIFT                         16
#define CAN_CS_DLC_WIDTH                         4
#define CAN_CS_RTR_MASK                          0x100000u
#define CAN_CS_RTR_SHIFT                         20
#define CAN_CS_RTR_WIDTH                         1
#define CAN_CS_IDE_MASK                          0x200000u
#define CAN_CS_IDE_SHIFT                         21
#define CAN_CS_IDE_WIDTH                         1
#define CAN_CS_SRR_MASK                          0x400000u
#define CAN_CS_SRR_SHIFT                         22
#define CAN_CS_SRR_WIDTH                         1
#define CAN_CS_CODE_MASK                         0xF000000u
#define CAN_CS_CODE_SHIFT                        24
#define CAN_CS_CODE_WIDTH                        4
#define CAN_MB_EDL_MASK                          0x80000000u
#define CAN_MB_BRS_MASK                          0x40000000u

enum
{
	FLEXCAN_RX_INACTIVE = 0x0, /*!< MB is not active.*/
	FLEXCAN_RX_BUSY = 0x1, /*!< FlexCAN is updating the contents of the MB. The CPU must not access the MB.*/
	FLEXCAN_RX_FULL = 0x2, /*!< MB is full.*/
	FLEXCAN_RX_EMPTY = 0x4, /*!< MB is active and empty.*/
	FLEXCAN_RX_OVERRUN = 0x6, /*!< MB is overwritten into a full buffer.*/
	FLEXCAN_RX_RANSWER = 0xA, /*!< A frame was configured to recognize a Remote Request Frame and transmit a Response Frame in return.*/
	FLEXCAN_RX_NOT_USED = 0xF /*!< Not used*/
};

enum
{
	FLEXCAN_TX_INACTIVE = 0x08, /*!< MB is not active.*/
	FLEXCAN_TX_ABORT = 0x09, /*!< MB is aborted.*/
	FLEXCAN_TX_DATA = 0x0C, /*!< MB is a TX Data Frame(MB RTR must be 0).*/
	FLEXCAN_TX_REMOTE = 0x1C, /*!< MB is a TX Remote Request Frame (MB RTR must be 1).*/
	FLEXCAN_TX_TANSWER = 0x0E, /*!< MB is a TX Response Request Frame from. an incoming Remote Request Frame.*/
	FLEXCAN_TX_NOT_USED = 0xF /*!< Not used*/
};

static inline void HAL_CanBus_WriteTxMessageBufferHeader(HAL_CanBus_T * p_hal, const CanMessage_T * p_txMessage, uint8_t bufferId)
{

}

static inline void HAL_CanBus_WriteTxMessageBufferData(HAL_CanBus_T * p_hal, const uint8_t * p_data, uint8_t dataLength, uint8_t bufferId)
{

}


static inline void HAL_CanBus_WriteTxMessageBuffer(HAL_CanBus_T * p_hal, const CanMessage_T * p_txMessage, uint8_t bufferId)
{
	volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, bufferId);
	volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
	volatile uint32_t * p_bufferId = &p_messageBuffer[1U];
	volatile uint8_t * p_bufferData = (volatile uint8_t*)(&p_messageBuffer[2U]);
	volatile uint32_t * p_bufferData32 = &p_messageBuffer[2U];
//	const uint32_t * p_messageData = (const uint32_t*)&p_txMessage->Data;
	uint8_t dlc = GetDataLengthCode(p_txMessage->DataLength);
	uint32_t messageHeader = 0U;
//	status_t stat = STATUS_SUCCESS;

	uint32_t txCode;

	{
	        /* Make sure the BRS bit will not be ignored */
//	        if (FLEXCAN_IsFDEnabled(base) && cs->enable_brs)
//	        {
//	            p_canBus->FDCTRL = (p_canBus->FDCTRL & ~CAN_FDCTRL_FDRATE_MASK) | CAN_FDCTRL_FDRATE(1U);
//	        }
//	        /* Check if the Payload Size is smaller than the payload configured */
//	        DEV_ASSERT((uint8_t)cs->dataLen <= FLEXCAN_GetPayloadSize(base));

		if(p_txMessage->Data != 0U)
		{
			//todo 32 bit write
			//account for endianess during write to buffer
			for(uint8_t iByte = 0; iByte < p_txMessage->DataLength; iByte++)
			{
				p_bufferData[iByte] = p_txMessage->Data[iByte];
			}

			for(uint8_t iByte = p_txMessage->DataLength; iByte < GetDataLengthMax(dlc); iByte++)
			{
				p_bufferData[iByte] = p_txMessage->FlexDataPadding;
			}
		}

		/* Clean up the arbitration field area */
		*p_bufferHeader = 0U;
		*p_bufferId 	= 0U;

		/* Set the ID according the format structure */
		if(p_txMessage->msgIdType == FLEXCAN_MSG_ID_EXT)
		{
			/* ID [28-0] */
			*p_bufferId &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
			*p_bufferId |= (p_txMessage->msgId & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

			/* Set IDE */
			messageHeader |= CAN_CS_IDE_MASK;

			/* Clear SRR bit */
			messageHeader &= ~CAN_CS_SRR_MASK;
		}
		if(p_txMessage->msgIdType == FLEXCAN_MSG_ID_STD)
		{
			/* ID[28-18] */
			*p_bufferId &= ~CAN_ID_STD_MASK;
			*p_bufferId |= (p_txMessage->msgId << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

			/* make sure IDE and SRR are not set */
			messageHeader &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);
		}

		/* Set the length of data in bytes */
		messageHeader &= ~CAN_CS_DLC_MASK;
		messageHeader |= ((uint32_t)dlc << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK;

//		txCode = ParseTxCode(p_txMessage);

		/* Set MB CODE */
//		if(txCode != (uint32_t)FLEXCAN_TX_NOT_USED)
//		{
//			if(p_txMessage->FrameType == (uint32_t)FLEXCAN_TX_REMOTE)
//			{
//				messageHeader |= CAN_CS_RTR_MASK;
//			}
//
//			messageHeader &= ~CAN_CS_CODE_MASK;
//
//			if(p_txMessage->fd_enable)
//			{
//				messageHeader |= ((txCode << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK) | CAN_MB_EDL_MASK;
//			}
//			else
//			{
//				messageHeader |= (txCode << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
//			}
//
//			if(p_txMessage->enable_brs)
//			{
//				messageHeader |= CAN_MB_BRS_MASK;
//			}

			*p_bufferHeader |= messageHeader;
//		}
	}

}

static inline void HAL_CanBus_ReadRxMessageBuffer(HAL_CanBus_T * p_hal, CanMessage_T * p_rxFrame, uint8_t bufferId)
{

}

static inline bool HAL_CanBus_ReadTxBufferEmptyFlag(HAL_CanBus_T * p_hal)
{

}

static inline bool HAL_CanBus_GetRxBufferFullFlag(HAL_CanBus_T * p_hal)
{

}

static inline void HAL_CanBus_EnableTxBufferEmptyInterrupt(HAL_CanBus_T * p_hal, uint8_t bufferId)
{
//    if (bufferId  < 32U)
//    {
	(p_hal->IMASK1) = ((p_hal->IMASK1) | (1UL << (bufferId % 32U)));
//    }
}

static inline void HAL_CanBus_EnableRxBufferFullInterrupt(HAL_CanBus_T * p_hal)
{

}

static inline void HAL_CanBus_DisableTxBufferEmptyInterrupt(HAL_CanBus_T * p_hal, uint8_t bufferId)
{
	(p_hal->IMASK1) = ((p_hal->IMASK1) & ~(1UL << (bufferId % 32U)));
}

static inline void HAL_CanBus_DisableRxBufferFullInterrupt(HAL_CanBus_T * p_hal)
{

}

static inline void HAL_CanBus_ClearRxBufferFullFlag(HAL_CanBus_T * p_hal)
{
}


static inline void HAL_CanBus_SetBaudRate(HAL_CanBus_T * p_hal, uint32_t baudRate)
{
}




static inline void HAL_CanBus_Init(uint32_t busFreq)
{

}

#endif
