/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   CanType.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef CAN_TYPE_H
#define CAN_TYPE_H

#include <stdint.h>
#include <stdbool.h>

/*! Extended frame format: with 29 identifier bits */
typedef struct
{
    uint32_t Id_6_0 :7; /*!< ID[0:6] */
    uint32_t Id_14_7 :8; /*!< ID[14:7] */
    uint32_t Id_17_15 :3; /*!< ID[17:15] */
    uint32_t Id_20_18 :3; /*!< ID[20:18] */
    uint32_t Id_28_21 :8; /*!< ID[28:21] */
    uint32_t Rsvd :3;
} CanMessage_StandardId_T;

/*! Base frame format: with 11 identifier bits */
typedef struct
{
    uint32_t Id_2_0 :3; /*!< ID[0:2] */
    uint32_t Id_10_3 :8; /*!< ID[10:3] */
    uint32_t Rsvd :21;
} CanMessage_ExtendId_T;

typedef union
{
    CanMessage_StandardId_T StdID;
    CanMessage_ExtendId_T ExtID;
    uint32_t Id;
} CanMessage_Id_T;

typedef enum
{
    CAN_FRAME_FORMAT_STANDARD,
    CAN_FRAME_FORMAT_EXTEND,
//    Base frame format: with 11 identifier bits
//    Extended frame format: with 29 identifier bits
} CanMessage_FrameFormat_T;

typedef enum
{
    CAN_FRAME_TYPE_DATA,
    CAN_FRAME_TYPE_REMOTE,
//    Data frame: a frame containing node data for transmission
//    Remote frame: a frame requesting the transmission of a specific identifier
//    Error frame: a frame transmitted by any node detecting an error
//    Overload frame: a frame to inject a delay between data or remote frame
} CanMessage_FrameType_T;


typedef enum
{
    CAN_MESSAGE_IDLE,
//    CAN_MESSAGE_RX_WAIT,
//    CAN_MESSAGE_RX_COMPLETE,
    CAN_MESSAGE_RX_WAIT_DATA,
    CAN_MESSAGE_RX_WAIT_REMOTE,
    CAN_MESSAGE_RX_WAIT_SERVICE,
//    CAN_MESSAGE_RX_FIFO_BUSY,
//    CAN_MESSAGE_COMPLETE,
    CAN_MESSAGE_TX_DATA,
    CAN_MESSAGE_TX_REMOTE,
//    CAN_MESSAGE_TX_INIT,
//    CAN_MESSAGE_RX_INIT_DATA,
//    CAN_MESSAGE_RX_INIT_REMOTE,
//    CAN_MESSAGE_RX_REMOTE,
} CanMessage_Status_T;

//typedef enum
//{
//    CAN_BUS_MESSAGE_BUFFER_IDLE,
//    CAN_BUS_MESSAGE_BUFFER_RX_BUSY,
//    CAN_BUS_MESSAGE_BUFFER_TX_BUSY,
//    CAN_BUS_MESSAGE_BUFFER_RX_FIFO_BUSY,
//    CAN_BUS_MESSAGE_BUFFER_COMPLETE,
//    CAN_BUS_MESSAGE_BUFFER_TX_REMOTE,
//    CAN_BUS_MESSAGE_BUFFER_RX_REMOTE,
//#if CONFIG_CAN_BUS_DMA_ENABLE
//    CAN_BUS_MESSAGE_BUFFER_DMA_ERROR
//#endif
//} CanBus_MessageBufferStatus_T;

typedef struct
{
    CanMessage_Id_T Id;
    CanMessage_FrameFormat_T Format;         /*!< Standard or extended frame */
    CanMessage_FrameType_T Type;             /*!< Remote or data frame */

    uint8_t Priority;                             /*!< transmit buffer priority */
    uint16_t TimeStamp;
    uint8_t DataLength;
    bool EnableBitRateSwitch;
    bool EnableFlexData;
    uint8_t FlexDataPadding;
//      uint32_t rtr              : 1;
//      uint32_t edl              : 1;
//      uint32_t brs              : 1;
//      uint32_t esi              : 1;
//      uint32_t dlc              : 4;

    union
    {
        uint8_t Data[8U];
        uint32_t Data32[2U];
    };

    uint16_t Crc;
    CanMessage_Status_T Status; //HAL translates to code matching hw
    //hwbufferid

} CanMessage_T;



#endif
