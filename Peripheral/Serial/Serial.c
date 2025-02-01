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
    @version V0
*/
/******************************************************************************/
#include "Serial.h"

/*
    Single threaded buffer read/write only need disable channel ISR
    if interrupt occurs after checking sw buffer, it will run to completion,
    sw write occur in between hw read/write
*/
/*
    Multithread always disable all interrupts for entire duration
    must enter critical section before checking sw buffer
*/
static inline void EnterCriticalTx(Serial_T * p_serial)
{
#if     defined(CONFIG_SERIAL_SINGLE_THREADED)
    HAL_Serial_DisableTxInterrupt(p_serial->CONST.P_HAL_SERIAL);
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL) || defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    (void)p_serial;
    _Critical_DisableIrq();
#endif
}

static inline void ExitCriticalTx(Serial_T * p_serial)
{
    (void)p_serial;
#if     defined(CONFIG_SERIAL_SINGLE_THREADED)
    /* EnableTxInterrupt determine by checking of buffer */
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL) || defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    _Critical_EnableIrq();
#endif
}

static inline void EnterCriticalRx(Serial_T * p_serial)
{
#if     defined(CONFIG_SERIAL_SINGLE_THREADED)
    HAL_Serial_DisableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL) || defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    (void)p_serial;
    _Critical_DisableIrq();
#endif
}

static inline void ExitCriticalRx(Serial_T * p_serial)
{
#if     defined(CONFIG_SERIAL_SINGLE_THREADED)
    HAL_Serial_EnableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL) || defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    (void)p_serial;
    _Critical_EnableIrq();
#endif
}


/*
    Selection between mutex and critical
*/
static inline bool AcquireCriticalTx(Serial_T * p_serial)
{
#if        defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    return Critical_AcquireSignal(&p_serial->TxMutex);
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
    (void)p_serial;
    _Critical_DisableIrq();
    return true;
#elif     defined(CONFIG_SERIAL_SINGLE_THREADED)
    HAL_Serial_DisableTxInterrupt(p_serial->CONST.P_HAL_SERIAL);
    return true;
#endif
}

static inline void ReleaseCriticalTx(Serial_T * p_serial)
{
#if        defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    Critical_ReleaseSignal(&p_serial->TxMutex);
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
    (void)p_serial;
    _Critical_EnableIrq();
#elif     defined(CONFIG_SERIAL_SINGLE_THREADED)
    (void)p_serial;
    /* EnableTxInterrupt determine by checking of buffer */
#endif
}

static inline bool AcquireCriticalRx(Serial_T * p_serial)
{
#if        defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    return Critical_AcquireSignal(&p_serial->RxMutex);
#elif     defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
    (void)p_serial;
    _Critical_DisableIrq();
    return true;
#elif     defined(CONFIG_SERIAL_SINGLE_THREADED)
    HAL_Serial_DisableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
    return true;
#endif
}

static inline void ReleaseCriticalRx(Serial_T * p_serial)
{
#if        defined(CONFIG_SERIAL_MULTITHREADED_USE_MUTEX)
    Critical_ReleaseSignal(&p_serial->RxMutex);
#elif    defined(CONFIG_SERIAL_MULTITHREADED_USE_CRITICAL)
    (void)p_serial;
    _Critical_EnableIrq();
#elif     defined(CONFIG_SERIAL_SINGLE_THREADED)
    HAL_Serial_EnableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
#endif
}

/*

*/
static inline bool Hal_SendChar(Serial_T * p_serial, const uint8_t txchar)
{
    bool isNotFull = (HAL_Serial_ReadTxEmptyCount(p_serial->CONST.P_HAL_SERIAL) > 0U);
    if (isNotFull == true) { HAL_Serial_WriteTxChar(p_serial->CONST.P_HAL_SERIAL, txchar); }
    return isNotFull;
    // return (HAL_Serial_ReadTxEmptyCount(p_serial->CONST.P_HAL_SERIAL) > 0U) ? ({ HAL_Serial_WriteTxChar(p_serial->CONST.P_HAL_SERIAL, txchar); true; }) : false;
}

/*

*/
static inline bool Hal_RecvChar(Serial_T * p_serial, uint8_t * p_rxChar)
{
    bool isNotEmpty = (HAL_Serial_ReadRxFullCount(p_serial->CONST.P_HAL_SERIAL) > 0U);
    if(isNotEmpty == true) { *p_rxChar = HAL_Serial_ReadRxChar(p_serial->CONST.P_HAL_SERIAL); }
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
/*
    Rx data reg/fifo full ISR, receive from hw to software buffer
*/
void Serial_RxData_ISR(Serial_T * p_serial)
{
    uint8_t rxChar;

    while(HAL_Serial_ReadRxFullCount(p_serial->CONST.P_HAL_SERIAL) > 0U) /* Rx until hw buffer is empty */
    {
        if(Ring_IsFull(&p_serial->RxRing) == true) /* Rx until software buffer is full */
        {
            /* if buffer stays full, disable irq to prevent blocking lower priority threads. user must restart rx irq */
            HAL_Serial_DisableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
            break;
        }
        else
        {
            rxChar = HAL_Serial_ReadRxChar(p_serial->CONST.P_HAL_SERIAL);
            Ring_Enqueue(&p_serial->RxRing, &rxChar);
        }
    }
}

/*
    Tx data reg/fifo empty ISR, transmit from software buffer to hw
    Alternatively, HAL_Serial_ReadTxFullCount < CONFIG_HAL_SERIAL_FIFO_SIZE
*/
void Serial_TxData_ISR(Serial_T * p_serial)
{
    uint8_t txChar;

    while(HAL_Serial_ReadTxEmptyCount(p_serial->CONST.P_HAL_SERIAL) > 0U) /* Tx until hw buffer is full */
    {
        if(Ring_IsEmpty(&p_serial->TxRing) == true) /* Tx until software buffer is empty */
        {
            HAL_Serial_DisableTxInterrupt(p_serial->CONST.P_HAL_SERIAL);
            break;
        }
        else
        {
            Ring_Dequeue(&p_serial->TxRing, &txChar);
            HAL_Serial_WriteTxChar(p_serial->CONST.P_HAL_SERIAL, txChar);
        }
    }
}

bool Serial_PollRestartRxIsr(const Serial_T * p_serial)
{
    bool isOverrun = false;
    /* Continue waiting for buffer read, before restarting interrupts, if buffer is full */
    if((HAL_Serial_ReadRxOverrun(p_serial->CONST.P_HAL_SERIAL) == true) && (Ring_IsFull(&p_serial->RxRing) == false))
    {
        HAL_Serial_ClearRxErrors(p_serial->CONST.P_HAL_SERIAL);
        HAL_Serial_EnableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
        isOverrun = true;
    }
    return isOverrun;
}


void Serial_Init(Serial_T * p_serial)
{
    HAL_Serial_Init(p_serial->CONST.P_HAL_SERIAL);
    HAL_Serial_WriteTxSwitch(p_serial->CONST.P_HAL_SERIAL, true);
    HAL_Serial_WriteRxSwitch(p_serial->CONST.P_HAL_SERIAL, true);
    Ring_Init(&p_serial->TxRing);
    Ring_Init(&p_serial->RxRing);
    Serial_EnableRxIsr(p_serial);
    Serial_EnableTxIsr(p_serial);
}

void Serial_Deinit(Serial_T * p_serial)
{
    HAL_Serial_Deinit(p_serial->CONST.P_HAL_SERIAL);
}

bool Serial_ConfigBaudRate(Serial_T * p_serial, uint32_t baudRate)
{
    bool isSuccess = true;

    HAL_Serial_WriteTxSwitch(p_serial->CONST.P_HAL_SERIAL, false);
    HAL_Serial_WriteRxSwitch(p_serial->CONST.P_HAL_SERIAL, false);

    isSuccess = HAL_Serial_ConfigBaudRate(p_serial->CONST.P_HAL_SERIAL, baudRate);

    HAL_Serial_WriteTxSwitch(p_serial->CONST.P_HAL_SERIAL, true);
    HAL_Serial_WriteRxSwitch(p_serial->CONST.P_HAL_SERIAL, true);

    return isSuccess;
}

bool Serial_SendByte(Serial_T * p_serial, uint8_t txChar)
{
    bool isSuccess = false;

    EnterCriticalTx(p_serial);
    //write directly to hw fifo/reg todo
    //    if (Ring_IsEmpty(&p_serial->TxRing) == true) && HAL_Serial_GetIsActive == false
    //    {?Tx need not disable interrupt after checking empty, hw buffer can only decrease.
    //        isSuccess = Hal_SendChar(p_serial, txChar);
    //    }
    //    else
    //    {
    isSuccess = Ring_Enqueue(&p_serial->TxRing, &txChar);
    if(isSuccess == true) { HAL_Serial_EnableTxInterrupt(p_serial->CONST.P_HAL_SERIAL); }
    //    }
    ExitCriticalTx(p_serial);

    return isSuccess;
}

bool Serial_RecvByte(Serial_T * p_serial, uint8_t * p_rxChar)
{
    bool isSuccess = false;

    EnterCriticalRx(p_serial);
    //    if (Ring_IsEmpty(&p_serial->RxRing) == true)
    //    {?Rx must prevent interrupt after checking full, hw buffer can increase.
    //        isSuccess = Hal_RecvChar(p_serial, p_rxChar);
    //    }
    //    else
    //    {
    isSuccess = Ring_Dequeue(&p_serial->RxRing, p_rxChar);
    //    }
    ExitCriticalRx(p_serial);

    return isSuccess;
}

/*
    Calling function must check Serial_GetRxFullCount to avoid meta data collision
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
        //        if (Ring_IsEmpty(&p_serial->TxRing) == true)
        //        {
        //            charCount += Hal_Send(p_serial, p_srcBuffer, srcSize);
        //        }

        //        if (charCount < srcSize)
        //        {
        charCount += Ring_EnqueueMax(&p_serial->RxRing, p_srcBuffer, srcSize - charCount);
        if(charCount > 0U) { HAL_Serial_EnableTxInterrupt(p_serial->CONST.P_HAL_SERIAL); }
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
        //        if (Ring_IsEmpty(&p_serial->RxRing) == true)
        //        {
        //            EnterCriticalRx(p_serial);
        //            charCount += Hal_Recv(p_serial, p_destBuffer, destSize);
        //            ExitCriticalRx(p_serial);
        //        }
        //        else
        //        {
        charCount += Ring_DequeueMax(&p_serial->RxRing, p_destBuffer, destSize);
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
        status = Ring_EnqueueN(&p_serial->TxRing, p_srcBuffer, length);
        if(status == true) { HAL_Serial_EnableTxInterrupt(p_serial->CONST.P_HAL_SERIAL); }
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
        status = Ring_DequeueN(&p_serial->RxRing, p_destBuffer, length);
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
    Ring_Clear(&p_serial->TxRing);
    Ring_Clear(&p_serial->RxRing);
}

/*
    Experimental
*/

/*
    Passes buffer to upper layer to avoid double buffer write.
    Must flush buffer to start at 0 index.
*/
// uint8_t * Serial_AcquireTxBuffer(Serial_T * p_serial)
// {
//     uint8_t * p_buffer = 0U;

//     if(AcquireCriticalTx(p_serial) == true) //todo possibly must be mutex
//     {
//         p_buffer = Ring_AcquireBuffer(&p_serial->TxRing);
//     }

//     return p_buffer;
// }

// void Serial_ReleaseTxBuffer(Serial_T * p_serial, size_t writeSize)
// {
//     Ring_ReleaseBuffer(&p_serial->TxRing, writeSize);
//     if(Ring_IsEmpty(&p_serial->TxRing) == false) { HAL_Serial_EnableTxInterrupt(p_serial->CONST.P_HAL_SERIAL); }
//     ReleaseCriticalTx(p_serial);
// }

// bool Serial_SendCharString(Serial_T * p_serial, const uint8_t * p_srcBuffer)
// {
//     bool status = false;

//     const uint8_t * p_char = p_srcBuffer;

//     if(AcquireCriticalTx(p_serial) == true)
//     {
//         while(*p_char != '\0')
//         {
//             status = Ring_Enqueue(&p_serial->TxRing, p_char);
//             if(status == false) { break; }
//             p_char++;
//         }
//         if(p_char != p_srcBuffer) { HAL_Serial_EnableTxInterrupt(p_serial->CONST.P_HAL_SERIAL); }
//         ReleaseCriticalTx(p_serial);
//     }

//     return status;
// }

// //todo polling via hw fifo buffer
// void Serial_PollRxData(Serial_T * p_serial)
// {
//     HAL_Serial_DisableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
//     Serial_RxData_ISR(p_serial);
//     HAL_Serial_EnableRxInterrupt(p_serial->CONST.P_HAL_SERIAL);
// }

// void Serial_PollTxData(Serial_T * p_serial)
// {
//     HAL_Serial_DisableTxInterrupt(p_serial->CONST.P_HAL_SERIAL);
//     Serial_TxData_ISR(p_serial);
//     if(Ring_IsEmpty(&p_serial->TxRing) == false) { HAL_Serial_EnableTxInterrupt(p_serial->CONST.P_HAL_SERIAL); }
// }

