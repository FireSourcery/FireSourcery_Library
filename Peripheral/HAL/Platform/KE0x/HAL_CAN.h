// /******************************************************************************/
// /*!
//     @section LICENSE

//     Copyright (C) 2023 FireSourcery

//     This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.

//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.

//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
// */
// /******************************************************************************/
// /******************************************************************************/
// /*!
//     @file   HAL_CanBus.h
//     @author
//     @brief
//
// */
// /******************************************************************************/
// #ifndef HAL_CAN_BUS_PLATFORM_H
// #define HAL_CAN_BUS_PLATFORM_H

// #include "Peripheral/CanBus/CanMessage.h"

// #include "Include.h"

// #include <stdint.h>
// #include <stdbool.h>
// #include <string.h>

// typedef MSCAN_Type HAL_CanBus_T;

// static inline void HAL_CanBus_WriteMessageBufferControl(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_message)
// {

// }

// // static inline void HAL_CanBus_WriteTxMessageBufferControl(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_txMessage)
// // {


// // }

// // static inline void HAL_CanBus_WriteTxMessageBufferData(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const uint8_t * p_data, uint8_t dataLength)
// // {

// // }

// // static inline void HAL_CanBus_WriteTxMessageBufferStart(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// // {

// // }

// static inline void HAL_CanBus_WriteTxMessageBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, const CanMessage_T * p_txMessage)
// {

// }

// /*
//  * code field should read full
//  */
// static inline void HAL_CanBus_ReadRxMessageBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex, CanMessage_T * p_rxMessage)
// {

// }

// //static inline CanMessage_Status_T HAL_CanBus_ReadTxBufferStatus(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
// //    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
// //    CanMessage_Status_T status;
// //
// //    switch((*p_bufferHeader & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT)
// //    {
// ////        case FLEXCAN_TX_INACTIVE :     status = CAN_MESSAGE_IDLE; break;
// ////        case FLEXCAN_TX_ABORT :        status = CAN_MESSAGE_IDLE; break;
// //        case FLEXCAN_TX_DATA :        status = CAN_MESSAGE_TX_DATA; break;
// //        case FLEXCAN_TX_REMOTE :    status = CAN_MESSAGE_TX_REMOTE; break;
// ////        case FLEXCAN_TX_TANSWER :    status = CAN_MESSAGE_IDLE; break;
// ////        case FLEXCAN_TX_NOT_USED :    status = CAN_MESSAGE_IDLE; break;
// //        default: status = CAN_MESSAGE_IDLE; break;
// //    }
// //
// //    return status;
// //}
// //
// //static inline CanMessage_Status_T HAL_CanBus_ReadRxBufferStatus(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    volatile uint32_t * p_messageBuffer = GetPtrMessageBuffer(p_hal, hwBufferIndex);
// //    volatile uint32_t * p_bufferHeader = &p_messageBuffer[0U];
// //    CanMessage_Status_T status;
// //
// //    switch((*p_bufferHeader & CAN_CS_CODE_MASK) >> CAN_CS_CODE_SHIFT)
// //    {
// //        case FLEXCAN_RX_INACTIVE :     status = CAN_MESSAGE_IDLE; break;
// //        case FLEXCAN_RX_BUSY :        status = CAN_MESSAGE_RX_BUSY; break;
// //        case FLEXCAN_RX_FULL :        status = CAN_MESSAGE_RX_BUSY; break;
// //        case FLEXCAN_RX_EMPTY :        status = CAN_MESSAGE_IDLE; break;
// //        case FLEXCAN_RX_OVERRUN :    status = CAN_MESSAGE_IDLE; break;
// //        case FLEXCAN_RX_RANSWER :    status = CAN_MESSAGE_IDLE; break;
// //        case FLEXCAN_RX_NOT_USED :    status = CAN_MESSAGE_IDLE; break;
// //        default: status = CAN_MESSAGE_IDLE; break;
// //    }
// //
// //    return status;
// //}

// static inline uint8_t HAL_CanBus_CalcMessageBufferIndex(HAL_CanBus_T * p_hal, uint8_t userId)
// {
// }

// //static inline bool HAL_CanBus_ReadBufferComplete(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    return (p_hal->IFLAG1 | (1UL << (hwBufferIndex % 32U))); //interrupt is set,
// //}

// static inline bool HAL_CanBus_ReadIsBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// /*
//  * It must be guaranteed that the CPU clears only the bit causing
// the current interrupt. For this reason, bit manipulation
// instructions (BSET) must not be used to clear interrupt flags.
// These instructions may cause accidental clearing of interrupt
// flags which are set after entering the current interrupt service
// routine.
//  */
// static inline void HAL_CanBus_ClearBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// static inline void HAL_CanBus_EnableBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// static inline void HAL_CanBus_DisableBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// static inline bool HAL_CanBus_ReadIsBufferTxRemoteRxEmpty(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// // static inline bool HAL_CanBus_ReadIsBufferTxRemoteRxFull(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// // {

// // }



// //static inline uint8_t HAL_CanBus_CalcRxMessageBufferIndex(HAL_CanBus_T * p_hal, uint8_t userId)
// //{
// //    return HAL_CanBus_CalcTxMessageBufferIndex(p_hal, userId);
// //}
// //static inline bool HAL_CanBus_ReadRxBufferComplete(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// ////    return HAL_CanBus_ReadTxBufferComplete(p_hal, hwBufferIndex);
// //}
// //
// //static inline bool HAL_CanBus_ReadRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    return HAL_CanBus_ReadBufferInterrupt(p_hal, hwBufferIndex);
// //}
// //
// //static inline void HAL_CanBus_ClearRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    HAL_CanBus_ClearBufferInterrupt(p_hal, hwBufferIndex);
// //}
// //
// //static inline void HAL_CanBus_EnableRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    HAL_CanBus_EnableBufferInterrupt(p_hal, hwBufferIndex);
// //}
// //
// //static inline void HAL_CanBus_DisableRxBufferInterrupt(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// //{
// //    HAL_CanBus_DisableBufferInterrupt(p_hal, hwBufferIndex);
// //}

// static inline bool HAL_CanBus_LockRxBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// static inline void HAL_CanBus_UnlockRxBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// {
// }

// // static inline bool HAL_CanBus_LockRxFifoBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// // {
// //     (void)p_hal;
// //     return true;
// // }

// // static inline void HAL_CanBus_UnlockRxFifoBuffer(HAL_CanBus_T * p_hal, uint8_t hwBufferIndex)
// // {
// //     (void)p_hal;
// // }

// #define CONFIG_HAL_CAN_BUS_1_IRQ_NUMBER 88U
// #define CONFIG_HAL_CAN_BUS_0_IRQ_NUMBER 81U

// static inline void HAL_CanBus_DisableInterrupts(HAL_CanBus_T * p_hal)
// {
// }

// static inline void HAL_CanBus_EnableInterrupts(HAL_CanBus_T * p_hal)
// {
// }


// static inline void HAL_CanBus_ConfigBitRate(HAL_CanBus_T * p_hal, uint32_t bitRate)
// {
//     (void)p_hal; (void)bitRate;
// }

// static inline void HAL_CanBus_Init(HAL_CanBus_T * p_hal, uint32_t busFreq)
// {
//     (void)p_hal; (void)busFreq;
// }


// // #include "External/SDK_2_11_0_MKE06Z64xxx4/devices/MKE06Z4/MKE06Z4.h"
// #include <stdint.h>
// #include <stdbool.h>

// static inline bool HAL_CanBus_GetTxBufferEmptyFlag(HAL_CanBus_T * p_hal)
// {
//     return (bool)(MSCAN->CANTFLG & MSCAN_CANTFLG_TXE_MASK);
// }

// static inline bool HAL_CanBus_GetRxBufferFullFlag(HAL_CanBus_T * p_hal)
// {
//     return (bool)(MSCAN->CANRFLG & MSCAN_CANRFLG_RXF_MASK);
// }

// static inline void HAL_CanBus_EnableTxBufferEmptyInterrupt(HAL_CanBus_T * p_hal)
// {
//     MSCAN->CANTIER |= MSCAN_CANTIER_TXEIE_MASK;
// }

// static inline void HAL_CanBus_EnableRxBufferFullInterrupt(HAL_CanBus_T * p_hal)
// {
//     MSCAN->CANRIER |= MSCAN_CANRIER_RXFIE_MASK;
// }

// static inline void HAL_CanBus_DisableTxBufferEmptyInterrupt(HAL_CanBus_T * p_hal)
// {
//     MSCAN->CANTIER &= ~MSCAN_CANTIER_TXEIE_MASK;
// }

// static inline void HAL_CanBus_DisableRxBufferFullInterrupt(HAL_CanBus_T * p_hal)
// {
//     MSCAN->CANRIER &= ~MSCAN_CANRIER_RXFIE_MASK;
// }

// static inline void HAL_CanBus_ClearRxBufferFullFlag(HAL_CanBus_T * p_hal)
// {
//     MSCAN->CANRFLG |= MSCAN_CANRFLG_RXF_MASK;
// }


// static inline void HAL_CanBus_SetBaudRate(HAL_CanBus_T * p_hal, uint32_t baudRate)
// {
//     #define MSCAN_TIME_QUANTA_NUM (8)

//     //MSCAN_SetBaudRate(MSCAN, FreqClockSource, baudRate);

//     mscan_timing_config_t timingConfig;
//     uint32_t priDiv = baudRate * (uint32_t)MSCAN_TIME_QUANTA_NUM;

//     /* Assertion: Desired baud rate is too high. */
//     assert(baudRate <= 1000000U);
//     /* Assertion: Source clock should greater than baud rate * MSCAN_TIME_QUANTA_NUM. */
//     assert(priDiv <= FreqClockSource);

// #if (defined(FSL_FEATURE_FLEXCAN_HAS_IMPROVED_TIMING_CONFIG) && FSL_FEATURE_FLEXCAN_HAS_IMPROVED_TIMING_CONFIG)
//     /* MsCAN timing setting formula:
//      * MSCAN_TIME_QUANTA_NUM = 1 + (TSEG1 + 1) + (TSEG2 + 1);
//      * We can calculate SEG1 and SEG2 according to the load factor
//      */
//     if (false == MSCAN_CalculateImprovedTimingValues(baudRate_Bps, sourceClock_Hz, &timingConfig))
// #endif
//     {
//         if (0U == priDiv)
//         {
//             priDiv = 1U;
//         }

//         priDiv = (FreqClockSource / priDiv) - 1U;

//         /* Desired baud rate is too low. */
//         if (priDiv > 0x3FU)
//         {
//             priDiv = 0x3FU;
//         }

//         timingConfig.priDiv     = (uint8_t)priDiv;
//         timingConfig.timeSeg1   = 3U;
//         timingConfig.timeSeg2   = 2U;
//         timingConfig.sJumpwidth = 0U;
//     }
//     timingConfig.samp = 0;

//     /* Update actual timing characteristic. */
//     MSCAN_SetTimingConfig(MSCAN, &timingConfig);
// }


// //static inline void HAL_CanBus_WriteTxMessageBuffer(CAN_ID_t id, CAN_FrameFormat_t format, CAN_FrameType_t type, uint8_t length, uint8_t * p_data)
// //{
// //    mscan_frame_t txFrame;
// //
// //    txFrame.ID_Type.ID = id;
// //    txFrame.format     = format;
// //    txFrame.type       = type;
// //    txFrame.DLR        = length;
// //
// //    for (i = 0U; i < length; i++)
// //    {
// //        txFrame.DSR[i] = p_data[i];
// //    }
// //
// //    MSCAN_WriteTxMb(MSCAN, &txFrame);
// //}

// static inline void HAL_CanBus_WriteTxMessageBuffer(HAL_CanBus_T * p_hal, CAN_Frame_t * p_txFrame)
// {
//     uint8_t txEmptyFlag;
//     mscan_mb_t mb = {0};
//     IDR1_3_UNION sIDR1, sIDR3;
//     status_t status;
//     uint8_t i;

//     /* Write IDR. */
//     if (kMSCAN_FrameFormatExtend == p_txFrame->Format)
//     {
//         /* Deal with Extended frame. */
//         sIDR1.IDR1.EID20_18_OR_SID2_0 = p_txFrame->ID.ExtID.EID20_18;
//         sIDR1.IDR1.R_TSRR             = 1U;
//         sIDR1.IDR1.R_TEIDE            = 1U;
//         sIDR1.IDR1.EID17_15           = p_txFrame->ID.ExtID.EID17_15;
//         sIDR3.IDR3.EID6_0             = p_txFrame->ID.ExtID.EID6_0;
//         sIDR3.IDR3.ERTR               = (kMSCAN_FrameTypeRemote == p_txFrame->Type) ? 1U : 0U;
//         /* Write into MB structure. */
//         mb.EIDR0 = p_txFrame->ID.ExtID.EID28_21;
//         mb.EIDR1 = sIDR1.Bytes;
//         mb.EIDR2 = p_txFrame->ID.ExtID.EID14_7;
//         mb.EIDR3 = sIDR3.Bytes;
//     }
//     else
//     {
//         /* Deal with Standard frame. */
//         sIDR1.IDR1.EID20_18_OR_SID2_0 = p_txFrame->ID.StdID.EID2_0;
//         sIDR1.IDR1.R_TSRR             = 0U;
//         sIDR1.IDR1.R_TEIDE            = 0U;
//         sIDR1.IDR1.EID17_15           = 0U; /* Reserved for Standard frame*/
//         /* Write into MB structure. */
//         mb.EIDR0 = p_txFrame->ID.StdID.EID10_3;
//         mb.EIDR1 = sIDR1.Bytes;
//     }
//     /* Write DLR, BPR */
//     mb.DLR = p_txFrame->DLR;
//     mb.BPR = p_txFrame->BPR;

//     /* Write DSR */
//     for (i = 0U; i < mb.DLR; i++)
//     {
//         mb.EDSR[i] = p_txFrame->DSR[i];
//     }

//     /* 1.Read TFLG to get the empty transmitter buffers. */
//     txEmptyFlag = MSCAN_GetTxBufferEmptyFlag(MSCAN);

// //    if ((uint8_t)kMSCAN_TxBufFull != txEmptyFlag)
// //    {
//         /* 2.Write TFLG value back. */
//         MSCAN_TxBufferSelect(MSCAN, txEmptyFlag);
//         /* Push contents of mb structure into hardware register. */
//         MSCAN->TEIDR0 = mb.EIDR0;
//         MSCAN->TEIDR1 = mb.EIDR1;
//         MSCAN->TEIDR2 = mb.EIDR2;
//         MSCAN->TEIDR3 = mb.EIDR3;
//         for (i = 0U; i < mb.DLR; i++)
//         {
//             MSCAN->TEDSR[i] = mb.EDSR[i];
//         }
//         MSCAN->TDLR = mb.DLR;
//         MSCAN->TBPR = mb.BPR;

//         /* 3.Read TBSEL again to get lowest tx buffer, then write 1 to clear
//         the corresponding bit to schedule transmission. */
//         MSCAN_TxBufferLaunch(MSCAN, MSCAN_GetTxBufferSelect(MSCAN));

// //        status = kStatus_Success;
// //    }
// //    else
// //    {
// //        status = kStatus_Fail;
// //    }

//     //return status;
// }

// static inline void HAL_CanBus_ReadRxMessageBuffer(HAL_CanBus_T * p_hal, CAN_Frame_t * p_rxFrame)
// {
//     IDR1_3_UNION sIDR1;
//     IDR1_3_UNION sIDR3;
//     uint8_t i;
//     status_t status;

// //    if (0U != MSCAN_GetRxBufferFullFlag(MSCAN))
// //    {
//         sIDR1.Bytes      = MSCAN_ReadRIDR1(MSCAN); //MSCAN->REIDR1;
//         sIDR3.Bytes      = MSCAN_ReadRIDR3(MSCAN);
//         p_rxFrame->Format = (mscan_frame_format_t)(sIDR1.IDR1.R_TEIDE);

//         if (kMSCAN_FrameFormatExtend == p_rxFrame->Format) /* Extended frame. */
//         {
//             p_rxFrame->Type                   = (mscan_frame_type_t)(sIDR3.IDR3.ERTR);
//             p_rxFrame->ID.ExtID.EID28_21 = MSCAN_ReadRIDR0(MSCAN);
//             p_rxFrame->ID.ExtID.EID20_18 = sIDR1.IDR1.EID20_18_OR_SID2_0;
//             p_rxFrame->ID.ExtID.EID17_15 = sIDR1.IDR1.EID17_15;
//             p_rxFrame->ID.ExtID.EID14_7  = MSCAN_ReadRIDR2(MSCAN);
//             p_rxFrame->ID.ExtID.EID6_0   = sIDR3.IDR3.EID6_0;
//         }
//         else /* Standard frame. */
//         {
//             p_rxFrame->Type                  = (mscan_frame_type_t)(sIDR1.IDR1.R_TSRR);
//             p_rxFrame->ID.StdID.EID10_3 = MSCAN_ReadRIDR0(MSCAN);
//             p_rxFrame->ID.StdID.EID2_0  = sIDR1.IDR1.EID20_18_OR_SID2_0;
//         }

//         p_rxFrame->DLR = MSCAN->RDLR & 0x0FU;
//         for (i = 0; i < p_rxFrame->DLR; i++)
//         {
//             p_rxFrame->DSR[i] = MSCAN->REDSR[i];
//         }

//         p_rxFrame->TSRH = MSCAN->RTSRH;
//         p_rxFrame->TSRL = MSCAN->RTSRL;

// //        status = kStatus_Success;
// //    }
// //    else
// //    {
// //        status = kStatus_Fail;
// //    }
// //
// //    return status;
// }

// /*! @brief Initialize KE06 MSCAN module. Default settings, interrupt driven for send and receive */
// static inline void HAL_CanBus_Init(HAL_CanBus_T * p_hal)
// {
//     mscan_config_t mscanConfig;

//     FreqClockSource = busFreq;

//     mscanConfig.baudRate                = 500000;
//     mscanConfig.enableTimer             = false;
//     mscanConfig.enableWakeup            = false;
//     mscanConfig.clkSrc                  = kMSCAN_ClkSrcBus;
//     mscanConfig.enableLoopBack          = false;
//     mscanConfig.enableListen            = false;
//     mscanConfig.busoffrecMode           = kMSCAN_BusoffrecAuto;
//     mscanConfig.filterConfig.filterMode = kMSCAN_Filter32Bit;

// //    mscanConfig.filterConfig.u32IDAR0 = MSCAN_RX_MB_EXT_MASK(NODE_ID1);
// //    mscanConfig.filterConfig.u32IDAR1 = MSCAN_RX_MB_EXT_MASK(NODE_ID1);
// //    mscanConfig.filterConfig.u32IDMR0 = MSCAN_IDMR0;
// //    mscanConfig.filterConfig.u32IDMR1 = MSCAN_IDMR1;

//     MSCAN_Init(MSCAN, &mscanConfig, busFreq);

//     MSCAN_EnableRxInterrupts(MSCAN, kMSCAN_RxFullInterruptEnable);
//     MSCAN_EnableTxInterrupts(MSCAN, kMSCAN_TxEmptyInterruptEnable);
//     EnableIRQ(MSCAN_1_IRQn);
//     EnableIRQ(MSCAN_2_IRQn);
// }

// #endif
