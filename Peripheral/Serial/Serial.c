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
    @file   Serial.c
    @author FireSourcery
    @brief

*/
/******************************************************************************/
#include "Serial.h"


#ifndef SERIAL_ENTER_CRITICAL
#define SERIAL_ENTER_CRITICAL(p_serial) _Critical_DisableIrq()
#endif
#ifndef SERIAL_EXIT_CRITICAL
#define SERIAL_EXIT_CRITICAL(p_serial) _Critical_EnableIrq()
#endif

#if     defined(CONFIG_SERIAL_SINGLE_THREADED)
    #define _ENTER_CRITICAL(local,...)   local
    #define _EXIT_CRITICAL(local,...)    local
#elif  defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
    #define _ENTER_CRITICAL(local,...)   __VA_ARGS__
    #define _EXIT_CRITICAL(local,...)    __VA_ARGS__
#else
    #define _ENTER_CRITICAL(local,...)
    #define _EXIT_CRITICAL(local,...)
#endif

/*
    Single threaded buffer read/write only need disable channel ISR
    if interrupt occurs after checking sw buffer, it will run to completion,
    sw write occur in between hw read/write
    Multithread disable all interrupts for entire duration
    Enter critical section before checking sw buffer
*/
static inline void EnterCriticalTx(Serial_T * p_serial) { _ENTER_CRITICAL(HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL), SERIAL_ENTER_CRITICAL(p_serial)); }
/* EnableTxInterrupt determine by checking of buffer */
static inline void ExitCriticalTx(Serial_T * p_serial) { _EXIT_CRITICAL((void)p_serial, SERIAL_EXIT_CRITICAL(p_serial)); }
static inline void EnterCriticalRx(Serial_T * p_serial) { _ENTER_CRITICAL(HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL), SERIAL_ENTER_CRITICAL(p_serial)); }
static inline void ExitCriticalRx(Serial_T * p_serial) { _EXIT_CRITICAL(HAL_Serial_EnableRxInterrupt(p_serial->P_HAL_SERIAL), SERIAL_EXIT_CRITICAL(p_serial)); }
static inline bool AcquireCriticalTx(Serial_T * p_serial) { _ENTER_CRITICAL(HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL); return true, SERIAL_ENTER_CRITICAL(p_serial)); }
static inline void ReleaseCriticalTx(Serial_T * p_serial) { _EXIT_CRITICAL((void)p_serial, SERIAL_EXIT_CRITICAL(p_serial)); }
static inline bool AcquireCriticalRx(Serial_T * p_serial) { _ENTER_CRITICAL(HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL); return true, SERIAL_ENTER_CRITICAL(p_serial)); }
static inline void ReleaseCriticalRx(Serial_T * p_serial) { _EXIT_CRITICAL(HAL_Serial_EnableRxInterrupt(p_serial->P_HAL_SERIAL), SERIAL_EXIT_CRITICAL(p_serial)); }

/*

*/
static inline bool Hal_SendChar(Serial_T * p_serial, const uint8_t txchar)
{
    bool isNotFull = (HAL_Serial_ReadTxEmptyCount(p_serial->P_HAL_SERIAL) > 0U);
    if (isNotFull == true) { HAL_Serial_WriteTxChar(p_serial->P_HAL_SERIAL, txchar); }
    return isNotFull;
    // return (HAL_Serial_ReadTxEmptyCount(p_serial->P_HAL_SERIAL) > 0U) ? ({ HAL_Serial_WriteTxChar(p_serial->P_HAL_SERIAL, txchar); true; }) : false;
}

/*

*/
static inline bool Hal_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
    bool isNotEmpty = (HAL_Serial_ReadRxFullCount(p_serial->P_HAL_SERIAL) > 0U);
    if(isNotEmpty == true) { *p_rxChar = HAL_Serial_ReadRxChar(p_serial->P_HAL_SERIAL); }
    return isNotEmpty;
}

static inline uint8_t Hal_GetLoopCount(size_t length)
{
#ifdef CONFIG_SERIAL_HW_FIFO_DISABLE
    (void)length;
    return 1U;
#else
    return length;
#endif
}

static inline size_t Hal_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
    size_t charCount;
    for(charCount = 0U; charCount < Hal_GetLoopCount(length); charCount++)
    {
        if(Hal_SendChar(p_serial, p_srcBuffer[charCount]) == false) { break; }
    }
    return charCount;
}

static inline size_t Hal_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
    size_t charCount;
    for(charCount = 0U; charCount < Hal_GetLoopCount(length); charCount++)
    {
        if(Hal_RecvChar(p_serial, &p_destBuffer[charCount]) == false) { break; }
    }
    return charCount;
}

/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void Serial_Init(Serial_T * p_serial)
{
    HAL_Serial_Init(p_serial->P_HAL_SERIAL);
    HAL_Serial_WriteTxSwitch(p_serial->P_HAL_SERIAL, true);
    HAL_Serial_WriteRxSwitch(p_serial->P_HAL_SERIAL, true);
    Ring_Init(&p_serial->TX_RING);
    Ring_Init(&p_serial->RX_RING);
    Serial_EnableRxIsr(p_serial);
    Serial_EnableTxIsr(p_serial);
}

void Serial_Deinit(Serial_T * p_serial)
{
    HAL_Serial_Deinit(p_serial->P_HAL_SERIAL);
}

bool Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate)
{
    bool isSuccess = true;

    HAL_Serial_WriteTxSwitch(p_serial->P_HAL_SERIAL, false);
    HAL_Serial_WriteRxSwitch(p_serial->P_HAL_SERIAL, false);

    isSuccess = HAL_Serial_ConfigBaudRate(p_serial->P_HAL_SERIAL, baudRate);

    HAL_Serial_WriteTxSwitch(p_serial->P_HAL_SERIAL, true);
    HAL_Serial_WriteRxSwitch(p_serial->P_HAL_SERIAL, true);

    return isSuccess;
}

bool Serial_SendByte(Serial_T * p_serial, uint8_t txChar)
{
    bool isSuccess = false;

    EnterCriticalTx(p_serial);
    //write directly to hw fifo/reg todo
    //    if (Ring_IsEmpty(p_serial->TX_RING.P_STATE) == true) && HAL_Serial_GetIsActive == false
    //    {?Tx need not disable interrupt after checking empty, hw buffer can only decrease.
    //        isSuccess = Hal_SendChar(p_serial, txChar);
    //    }
    //    else
    //    {
    isSuccess = Ring_Enqueue(p_serial->TX_RING.P_STATE, &txChar);
    if(isSuccess == true) { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
    //    }
    ExitCriticalTx(p_serial);

    return isSuccess;
}

bool Serial_RecvByte(Serial_T * p_serial, uint8_t * p_rxChar)
{
    bool isSuccess = false;

    EnterCriticalRx(p_serial);
    //    if (Ring_IsEmpty(p_serial->RX_RING.P_STATE) == true)
    //    {?Rx must prevent interrupt after checking full, hw buffer can increase.
    //        isSuccess = Hal_RecvChar(p_serial, p_rxChar);
    //    }
    //    else
    //    {
    isSuccess = Ring_Dequeue(p_serial->RX_RING.P_STATE, p_rxChar);
    //    }
    ExitCriticalRx(p_serial);

    return isSuccess;
}

/*
    Caller check Serial_GetRxFullCount to avoid meta data collision
*/
uint8_t Serial_GetByte(Serial_T * p_serial)
{
    uint8_t rxChar;
    if(Serial_RecvByte(p_serial, &rxChar) == false) { rxChar = 0xFFU; }
    return rxChar;
}


/*
    Send max available, if > than buffer size. May clip.
*/
size_t Serial_SendMax(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t srcSize)
{
    size_t charCount = 0U;

    if(AcquireCriticalTx(p_serial) == true)
    {
        //send immediate if fits in hardware fifo
        //        if (Ring_IsEmpty(p_serial->TX_RING.P_STATE) == true)
        //        {
        //            charCount += Hal_Send(p_serial, p_srcBuffer, srcSize);
        //        }

        //        if (charCount < srcSize)
        //        {
        charCount += Ring_EnqueueMax(p_serial->RX_RING.P_STATE, p_srcBuffer, srcSize - charCount);
        if(charCount > 0U) { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
        //        }
        ReleaseCriticalTx(p_serial);
    }

    return charCount;
}

size_t Serial_RecvMax(Serial_T * p_serial, uint8_t * p_destBuffer, size_t destSize)
{
    size_t charCount = 0U;

    if(AcquireCriticalRx(p_serial) == true)
    {
        //        if (Ring_IsEmpty(p_serial->RX_RING.P_STATE) == true)
        //        {
        //            EnterCriticalRx(p_serial);
        //            charCount += Hal_Recv(p_serial, p_destBuffer, destSize);
        //            ExitCriticalRx(p_serial);
        //        }
        //        else
        //        {
        charCount += Ring_DequeueMax(p_serial->RX_RING.P_STATE, p_destBuffer, destSize);
        //        }
        ReleaseCriticalRx(p_serial);
    }

    return charCount;
}

bool Serial_SendN(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
    bool status = false;

    if(AcquireCriticalTx(p_serial) == true)
    {
        status = Ring_EnqueueN(p_serial->TX_RING.P_STATE, p_srcBuffer, length);
        if(status == true) { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
        ReleaseCriticalTx(p_serial);
    }

    return status;
}

/*
    Rx only if length had been reached
*/
bool Serial_RecvN(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
    bool status = false;

    if(AcquireCriticalRx(p_serial) == true)
    {
        status = Ring_DequeueN(p_serial->RX_RING.P_STATE, p_destBuffer, length);
        ReleaseCriticalRx(p_serial);
    }

    return status;
}

bool Serial_Send(Serial_T * p_serial, const uint8_t * p_srcBuffer, size_t length)
{
    return Serial_SendN(p_serial, p_srcBuffer, length);
}

size_t Serial_Recv(Serial_T * p_serial, uint8_t * p_destBuffer, size_t length)
{
    return Serial_RecvMax(p_serial, p_destBuffer, length);
}

void Serial_FlushBuffers(Serial_T * p_serial)
{
    Ring_Clear(p_serial->TX_RING.P_STATE);
    Ring_Clear(p_serial->RX_RING.P_STATE);
}

/*
    Experimental
*/


// char Serial_GetChar(Serial_T * p_serial)
// {
//     char rxChar = 0xFFU;
//     if (Serial_RecvByte(p_serial, (uint8_t *)&rxChar) == false) { rxChar = 0xFFU; }
//     return rxChar;
// }

// bool Serial_SendCharString(Serial_T * p_serial, const uint8_t * p_srcBuffer)
// {
//     bool status = false;

//     const uint8_t * p_char = p_srcBuffer;

//     if(AcquireCriticalTx(p_serial) == true)
//     {
//         while(*p_char != '\0')
//         {
//             status = Ring_Enqueue(p_serial->TX_RING.P_STATE, p_char);
//             if(status == false) { break; }
//             p_char++;
//         }
//         if(p_char != p_srcBuffer) { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
//         ReleaseCriticalTx(p_serial);
//     }

//     return status;
// }

// polling via hw fifo buffer
// void Serial_PollRxData(Serial_T * p_serial)
// {
//     HAL_Serial_DisableRxInterrupt(p_serial->P_HAL_SERIAL);
//     Serial_RxData_ISR(p_serial);
//     HAL_Serial_EnableRxInterrupt(p_serial->P_HAL_SERIAL);
// }

// void Serial_PollTxData(Serial_T * p_serial)
// {
//     HAL_Serial_DisableTxInterrupt(p_serial->P_HAL_SERIAL);
//     Serial_TxData_ISR(p_serial);
//     if(Ring_IsEmpty(p_serial->TX_RING.P_STATE) == false) { HAL_Serial_EnableTxInterrupt(p_serial->P_HAL_SERIAL); }
// }

