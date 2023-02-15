/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
    @file     HAL_CanBus.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HAL_CAN_BUS_PLATFORM_H
#define HAL_CAN_BUS_PLATFORM_H

#include "Peripheral/CanBus/CanMessage.h"
#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef CAN_Type HAL_CanBus_T;

//static inline bool HAL_CanBus_CheckMessageBufferId(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    //    uint32_t val1, val2 = 1;
//
//    //        if (msgBuffIdx > (((p_canBus->MCR) & CAN_MCR_MAXMB_MASK) >> CAN_MCR_MAXMB_SHIFT) )
//    //        {
//    //            stat = STATUS_CAN_BUFF_OUT_OF_RANGE;
//    //        }
//
//    /* Check if RX FIFO is enabled*/
////    if(((p_canBus->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U)
////    {
////        /* Get the number of RX FIFO Filters*/
////        val1 = (((p_canBus->CTRL2) & CAN_CTRL2_RFFN_MASK) >> CAN_CTRL2_RFFN_SHIFT);
////        /* Get the number if MBs occupied by RX FIFO and ID filter table*/
////        /* the Rx FIFO occupies the memory space originally reserved for MB0-5*/
////        /* Every number of RFFN means 8 number of RX FIFO filters*/
////        /* and every 4 number of RX FIFO filters occupied one MB*/
////        val2 = RxFifoOcuppiedLastMsgBuff(val1);
////
//////        if(msgBuffIdx <= val2)
//////        {
//////            stat = STATUS_CAN_BUFF_OUT_OF_RANGE;
//////        }
////    }
//}

static volatile uint32_t * GetPtrMessageBuffer(HAL_CanBus_T * p_hal, uint32_t msgBuffIdx)
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

static uint8_t CalcDataFieldLength(uint8_t dlcValue)
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
            default: break;    /* The argument is not a valid DLC size */
        }
    }

    return ret;
}

static uint8_t CalcDataLengthCode(uint8_t dataLength)
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

/*!< MB code for TX or RX buffers. Defined by flexcan_mb_code_rx_t and flexcan_mb_code_tx_t */
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


static inline void HAL_CanBus_WriteMessageBufferControl(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_message)
{
    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
    volatile uint32_t * p_bufferId = &p_messageBuffer[1U];
    uint32_t messageHeader = 0U;
    uint32_t code;
    {
        if((p_message->EnableBitRateSwitch == true)) //reg_IsFdEnabled() &&
        {
            p_hal->FDCTRL = (p_hal->FDCTRL & ~CAN_FDCTRL_FDRATE_MASK) | CAN_FDCTRL_FDRATE(1U);
        }

        /* Clean up the arbitration field area */
        *p_bufferHeader = 0U;
        *p_bufferId     = 0U;

        /* Set the ID according the format structure */
        if(p_message->Format == CAN_FRAME_FORMAT_EXTEND)
        {
            /* ID [28-0] */
            *p_bufferId &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
            *p_bufferId |= (p_message->Id.Id & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

            /* Set IDE */
            messageHeader |= CAN_CS_IDE_MASK;

            /* Clear SRR bit */
            messageHeader &= ~CAN_CS_SRR_MASK;
        }
        else if(p_message->Format == CAN_FRAME_FORMAT_STANDARD)
        {
            /* ID[28-18] */
            *p_bufferId &= ~CAN_ID_STD_MASK;
            *p_bufferId |= (p_message->Id.Id << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

            /* make sure IDE and SRR are not set */
            messageHeader &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);
        }

        /* Set the length of data in bytes */
//        messageHeader &= ~CAN_CS_DLC_MASK;
//        messageHeader |= ((uint32_t)dlc << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK;

        /* Set MB CODE */
//        if(txCode != (uint32_t)FLEXCAN_TX_NOT_USED)
        {

            switch(p_message->Status)
            {
                case CAN_MESSAGE_RX_WAIT_REMOTE:     code = FLEXCAN_RX_EMPTY;    break;
                case CAN_MESSAGE_RX_WAIT_DATA:         code = FLEXCAN_RX_EMPTY;     break;
                case CAN_MESSAGE_TX_DATA:             code = FLEXCAN_TX_DATA;        break;
                case CAN_MESSAGE_TX_REMOTE:         code = FLEXCAN_TX_REMOTE;    break;
                default: code =  0U; break;
            }

            if(p_message->Type == CAN_FRAME_TYPE_REMOTE)
            {
                messageHeader |= CAN_CS_RTR_MASK;
            }

            messageHeader &= ~CAN_CS_CODE_MASK;
            if(p_message->EnableFlexData)            {messageHeader |= CAN_MB_EDL_MASK;}
            if(p_message->EnableBitRateSwitch)        {messageHeader |= CAN_MB_BRS_MASK;}
            messageHeader |= (code << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
            *p_bufferHeader |= messageHeader;
        }
    }
}

// static inline void HAL_CanBus_WriteTxMessageBufferControl(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_txMessage)
// {


// }

// static inline void HAL_CanBus_WriteTxMessageBufferData(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const uint8_t * p_data, uint8_t dataLength)
// {

// }

// static inline void HAL_CanBus_WriteTxMessageBufferStart(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {

// }

static inline void HAL_CanBus_WriteTxMessageBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_txMessage)
{
    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
    volatile uint32_t * p_bufferId = &p_messageBuffer[1U];
    volatile uint8_t * p_bufferData = (  uint8_t*)(&p_messageBuffer[2U]);
//      uint32_t * p_bufferData32 = &p_messageBuffer[2U];
//    const uint32_t * p_messageData = (const uint32_t*)&p_txMessage->Data;
    uint8_t dlc = CalcDataLengthCode(p_txMessage->DataLength);
    uint8_t dataLength = CalcDataFieldLength(dlc);
    uint32_t messageHeader = 0U;
    uint32_t txCode;

    {
        if((p_txMessage->EnableBitRateSwitch == true)) //reg_IsFdEnabled() &&
        {
            p_hal->FDCTRL = (p_hal->FDCTRL & ~CAN_FDCTRL_FDRATE_MASK) | CAN_FDCTRL_FDRATE(1U);
        }

        if(p_txMessage->Data != 0U)
        {
            //todo 32 bit write
            //account for endianess during write to sw struct
            memcpy((uint8_t *)&p_bufferData[0U], &p_txMessage->Data[0U], p_txMessage->DataLength);
//            for(uint8_t iByte = 0; iByte < p_txMessage->DataLength; iByte++)
//            {
//                p_bufferData[iByte] = p_txMessage->Data[iByte];
//            }

            for(uint8_t iByte = p_txMessage->DataLength; iByte < dataLength; iByte++)
            {
                p_bufferData[iByte] = p_txMessage->FlexDataPadding;
            }
        }

        /* Clean up the arbitration field area */
        *p_bufferHeader = 0U;
        *p_bufferId     = 0U;

        /* Set the ID according the format structure */
        if(p_txMessage->Format == CAN_FRAME_FORMAT_EXTEND)
        {
            /* ID [28-0] */
            *p_bufferId &= ~(CAN_ID_STD_MASK | CAN_ID_EXT_MASK);
            *p_bufferId |= (p_txMessage->Id.Id & (CAN_ID_STD_MASK | CAN_ID_EXT_MASK));

            /* Set IDE */
            messageHeader |= CAN_CS_IDE_MASK;

            /* Clear SRR bit */
            messageHeader &= ~CAN_CS_SRR_MASK;
        }
        else if(p_txMessage->Format == CAN_FRAME_FORMAT_STANDARD)
        {
            /* ID[28-18] */
            *p_bufferId &= ~CAN_ID_STD_MASK;
            *p_bufferId |= (p_txMessage->Id.Id << CAN_ID_STD_SHIFT) & CAN_ID_STD_MASK;

            /* make sure IDE and SRR are not set */
            messageHeader &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);
        }

        /* Set the length of data in bytes */
        messageHeader &= ~CAN_CS_DLC_MASK;
        messageHeader |= ((uint32_t)dlc << CAN_CS_DLC_SHIFT) & CAN_CS_DLC_MASK;

        /* Set MB CODE */
//        if(txCode != (uint32_t)FLEXCAN_TX_NOT_USED)
        {
            if(p_txMessage->Type == CAN_FRAME_TYPE_REMOTE)
            {
                txCode = FLEXCAN_TX_REMOTE;
                messageHeader |= CAN_CS_RTR_MASK;
            }
            else
            {
                txCode = FLEXCAN_TX_DATA;
            }
            messageHeader &= ~CAN_CS_CODE_MASK;
            if(p_txMessage->EnableFlexData)            {messageHeader |= CAN_MB_EDL_MASK;}
            if(p_txMessage->EnableBitRateSwitch)    {messageHeader |= CAN_MB_BRS_MASK;}
            messageHeader |= (txCode << CAN_CS_CODE_SHIFT) & CAN_CS_CODE_MASK;
            *p_bufferHeader |= messageHeader;
        }
    }
}

/*
 * code field should read full
 */
static inline void HAL_CanBus_ReadRxMessageBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, CanMessage_T * p_rxMessage)
{
    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
    volatile uint32_t * p_bufferId = &p_messageBuffer[1U];
    volatile uint8_t * p_bufferData = (volatile uint8_t*)(&p_messageBuffer[2U]);
//    volatile uint32_t * p_bufferData32 = &p_messageBuffer[2U];
//    const uint32_t * p_messageData = (const uint32_t*)&p_txMessage->Data;
    uint8_t dlc = (uint8_t)(((*p_messageBuffer) & CAN_CS_DLC_MASK) >> 16);
    uint8_t dataLength = CalcDataFieldLength(dlc);
//    uint32_t messageHeader = 0U;

//#if FEATURE_CAN_HAS_FD
//    if (payload_size > FLEXCAN_GetPayloadSize(base))
//    {
//        payload_size = FLEXCAN_GetPayloadSize(base);
//    }
//#endif /* FEATURE_CAN_HAS_FD */

    p_rxMessage->DataLength = dataLength;
    p_rxMessage->Id.Id = ((*p_bufferHeader & CAN_CS_IDE_MASK) != 0U) ?    (*p_bufferId) : (*p_bufferId) >> CAN_ID_STD_SHIFT;
    memcpy(&p_rxMessage->Data[0U], (uint8_t *)&p_bufferData[0U], dataLength);
}

//static inline CanMessage_Status_T HAL_CanBus_ReadTxBufferStatus(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
//    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
//    CanMessage_Status_T status;
//
//    switch((*p_bufferHeader & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT)
//    {
////        case FLEXCAN_TX_INACTIVE :     status = CAN_MESSAGE_IDLE; break;
////        case FLEXCAN_TX_ABORT :        status = CAN_MESSAGE_IDLE; break;
//        case FLEXCAN_TX_DATA :        status = CAN_MESSAGE_TX_DATA; break;
//        case FLEXCAN_TX_REMOTE :    status = CAN_MESSAGE_TX_REMOTE; break;
////        case FLEXCAN_TX_TANSWER :    status = CAN_MESSAGE_IDLE; break;
////        case FLEXCAN_TX_NOT_USED :    status = CAN_MESSAGE_IDLE; break;
//        default: status = CAN_MESSAGE_IDLE; break;
//    }
//
//    return status;
//}
//
//static inline CanMessage_Status_T HAL_CanBus_ReadRxBufferStatus(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
//    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
//    CanMessage_Status_T status;
//
//    switch((*p_bufferHeader & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT)
//    {
//        case FLEXCAN_RX_INACTIVE :     status = CAN_MESSAGE_IDLE; break;
//        case FLEXCAN_RX_BUSY :        status = CAN_MESSAGE_RX_BUSY; break;
//        case FLEXCAN_RX_FULL :        status = CAN_MESSAGE_RX_BUSY; break;
//        case FLEXCAN_RX_EMPTY :        status = CAN_MESSAGE_IDLE; break;
//        case FLEXCAN_RX_OVERRUN :    status = CAN_MESSAGE_IDLE; break;
//        case FLEXCAN_RX_RANSWER :    status = CAN_MESSAGE_IDLE; break;
//        case FLEXCAN_RX_NOT_USED :    status = CAN_MESSAGE_IDLE; break;
//        default: status = CAN_MESSAGE_IDLE; break;
//    }
//
//    return status;
//}

static inline uint8_t HAL_CanBus_CalcMessageBufferIndex(HAL_CanBus_T * p_hal, uint8_t userId)
{
    return (((p_hal->MCR & CAN_MCR_RFEN_MASK) >> CAN_MCR_RFEN_SHIFT) != 0U) ? (5U + ((userId + 1U) * 8U / 4U)) : userId;
}

//static inline bool HAL_CanBus_ReadBufferComplete(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    return (p_hal->IFLAG1 | (1UL << (hwBufferIndex % 32U))); //interrupt is set,
//}

static inline bool HAL_CanBus_ReadIsBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    return (((p_hal->IFLAG1 & p_hal->IMASK1) >> (hwBufferIndex % 32U)) & 1U);
}

/*
 * It must be guaranteed that the CPU clears only the bit causing
the current interrupt. For this reason, bit manipulation
instructions (BSET) must not be used to clear interrupt flags.
These instructions may cause accidental clearing of interrupt
flags which are set after entering the current interrupt service
routine.
 */
static inline void HAL_CanBus_ClearBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    p_hal->IFLAG1 = 1UL << (hwBufferIndex % 32U);
}

static inline void HAL_CanBus_EnableBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    p_hal->IMASK1 = p_hal->IMASK1 | (1UL << (hwBufferIndex % 32U));
}

static inline void HAL_CanBus_DisableBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    p_hal->IMASK1 = p_hal->IMASK1 & ~(1UL << (hwBufferIndex % 32U));
}

static inline bool HAL_CanBus_ReadIsBufferTxRemoteRxEmpty(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    return (((GetPtrMessageBuffer(p_hal, hwBufferIndex)[0U] & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT) == FLEXCAN_RX_EMPTY);
}

// static inline bool HAL_CanBus_ReadIsBufferTxRemoteRxFull(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {

// }



//static inline uint8_t HAL_CanBus_CalcRxMessageBufferIndex(HAL_CanBus_T * p_hal, uint8_t userId)
//{
//    return HAL_CanBus_CalcTxMessageBufferIndex(p_hal, userId);
//}
//static inline bool HAL_CanBus_ReadRxBufferComplete(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
////    return HAL_CanBus_ReadTxBufferComplete(p_hal, hwBufferIndex);
//}
//
//static inline bool HAL_CanBus_ReadRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    return HAL_CanBus_ReadBufferInterrupt(p_hal, hwBufferIndex);
//}
//
//static inline void HAL_CanBus_ClearRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    HAL_CanBus_ClearBufferInterrupt(p_hal, hwBufferIndex);
//}
//
//static inline void HAL_CanBus_EnableRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    HAL_CanBus_EnableBufferInterrupt(p_hal, hwBufferIndex);
//}
//
//static inline void HAL_CanBus_DisableRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
//{
//    HAL_CanBus_DisableBufferInterrupt(p_hal, hwBufferIndex);
//}

static inline bool HAL_CanBus_LockRxBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    (void)*GetPtrMessageBuffer(p_hal, hwBufferIndex); /* Lock the mailbox by reading it */
    return true;
}

static inline void HAL_CanBus_UnlockRxBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
{
    (void)hwBufferIndex;
    (void)p_hal->TIMER; /* Unlock the mailbox by reading the free running timer */
}

// static inline bool HAL_CanBus_LockRxFifoBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
//     (void)p_hal;
//     return true;
// }

// static inline void HAL_CanBus_UnlockRxFifoBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
//     (void)p_hal;
// }

#define CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER 88U
#define CONFIG_HAL_CAN_BUS_0_IRQ_NUMBER 81U

static inline void HAL_CanBus_DisableInterrupts(HAL_CanBus_T * p_hal)
{
    if(p_hal == CAN1)
    {
        S32_NVIC->ICER[(uint32_t)(CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER) >> 5U] = (uint32_t)(1UL << ((uint32_t)(CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER) & (uint32_t)0x1FU));
    }
    else if(p_hal == CAN0)
    {
        S32_NVIC->ICER[(uint32_t)(CONFIG_HAL_CAN_BUS_0_IRQ_NUMBER) >> 5U] = (uint32_t)(1UL << ((uint32_t)(CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER) & (uint32_t)0x1FU));
    }
}

static inline void HAL_CanBus_EnableInterrupts(HAL_CanBus_T * p_hal)
{
    if(p_hal == CAN1)
    {
        S32_NVIC->ISER[(uint32_t)(CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER) >> 5U] = (uint32_t)(1UL << ((uint32_t)(CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER) & (uint32_t)0x1FU));
    }
    else if(p_hal == CAN0)
    {
        S32_NVIC->ISER[(uint32_t)(CONFIG_HAL_CAN_BUS_0_IRQ_NUMBER) >> 5U] = (uint32_t)(1UL << ((uint32_t)(CONFIG_HAL_CAN_BUS_0_IRQ_NUMBER) & (uint32_t)0x1FU));
    }
}


static inline void HAL_CanBus_ConfigBitRate(HAL_CanBus_T * p_hal, uint32_t bitRate)
{
    (void)p_hal; (void)bitRate;
}

static inline void HAL_CanBus_Init(HAL_CanBus_T * p_hal, uint32_t busFreq)
{
    (void)p_hal; (void)busFreq;
}

#endif
