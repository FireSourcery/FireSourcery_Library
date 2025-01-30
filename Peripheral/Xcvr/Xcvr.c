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
    @file   Xcvr.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Xcvr.h"

/*
    Abstraction layer runtime interface.
    Outside module handle Xcvr_Entry_T hardware init
*/
void Xcvr_Init(Xcvr_T * p_xcvr, uint8_t xcvrDefaultIndex)
{
    // p_xcvr->p_Xcvr = &p_xcvr->CONST.P_XCVR_TABLE[0U];
    if(Xcvr_SetXcvr(p_xcvr, xcvrDefaultIndex) != true) { Xcvr_SetXcvr(p_xcvr, 0U); }
}

bool Xcvr_SetXcvr(Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
    bool status = xcvrIndex < p_xcvr->CONST.XCVR_TABLE_LENGTH;
    if(status == true) { p_xcvr->p_Xcvr = &p_xcvr->CONST.P_XCVR_TABLE[xcvrIndex]; }
    return status;
}

bool Xcvr_CheckIsSet(const Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
    return ((xcvrIndex < p_xcvr->CONST.XCVR_TABLE_LENGTH) && (p_xcvr->p_Xcvr->P_CONTEXT == p_xcvr->CONST.P_XCVR_TABLE[xcvrIndex].P_CONTEXT));
}

bool Xcvr_CheckIsValid(const Xcvr_T * p_xcvr, void * p_target)
{
    bool isValid = false;

    for(uint8_t iXcvr = 0U; iXcvr < p_xcvr->CONST.XCVR_TABLE_LENGTH; iXcvr++)
    {
        if(p_target == p_xcvr->CONST.P_XCVR_TABLE[iXcvr].P_CONTEXT) { isValid = true; break; }
    }

    return isValid;
}

bool Xcvr_ConfigBaudRate(const Xcvr_T * p_xcvr, uint32_t baudRate) //todo check valid baudrate
{
    bool isSuccess = true;
#if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
    switch(p_xcvr->p_Xcvr->TYPE)
    {
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
        case XCVR_TYPE_INTERFACE:
            if(p_xcvr->p_Xcvr->P_INTERFACE->CONST_BAUD_RATE != 0U) { p_xcvr->p_Xcvr->P_INTERFACE->CONST_BAUD_RATE(p_xcvr->p_Xcvr->P_CONTEXT, baudRate); }
            break;
#endif
        case XCVR_TYPE_SERIAL:      isSuccess = Serial_ConfigBaudRate(p_xcvr->p_Xcvr->P_CONTEXT, baudRate);    break;
        case XCVR_TYPE_I2C:         break;
        case XCVR_TYPE_SPI:         break;
        case XCVR_TYPE_VIRTUAL:     break;
        default: break;
    }
#elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
    if(p_xcvr->p_Xcvr->P_INTERFACE->CONST_BAUD_RATE != 0U) { p_xcvr->p_Xcvr->P_INTERFACE->CONST_BAUD_RATE(p_xcvr->p_Xcvr->P_CONTEXT, baudRate); }
#endif
    return isSuccess;
}

//todo
bool Xcvr_TxByte(const Xcvr_T * p_xcvr, uint8_t txChar)
{
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
    return p_xcvr->p_Xcvr->P_INTERFACE->TX_BYTE(p_xcvr->p_Xcvr->P_CONTEXT, txChar);
#endif
}

//todo
bool Xcvr_RxByte(const Xcvr_T * p_xcvr, uint8_t * p_rxChar)
{
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
    return p_xcvr->p_Xcvr->P_INTERFACE->RX_BYTE(p_xcvr->p_Xcvr->P_CONTEXT, p_rxChar);
#endif
}

bool Xcvr_TxN(const Xcvr_T * p_xcvr, const uint8_t * p_src, size_t length)
{
    bool status;
#if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
    switch(p_xcvr->p_Xcvr->TYPE)
    {
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
        case XCVR_TYPE_INTERFACE:   status = p_xcvr->p_Xcvr->P_INTERFACE->TX_N(p_xcvr->p_Xcvr->P_CONTEXT, p_src, length); break;
#endif
        case XCVR_TYPE_SERIAL:      status = Serial_SendN(p_xcvr->p_Xcvr->P_CONTEXT, p_src, length); break;
        case XCVR_TYPE_I2C:         status = false;     break;
        case XCVR_TYPE_SPI:         status = false;     break;
        case XCVR_TYPE_VIRTUAL:     status = false;     break;
        default:                    status = false;     break;
    }
#elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
    status = p_xcvr->p_Xcvr->P_INTERFACE->TX_N(p_xcvr->p_Xcvr->P_CONTEXT, p_src, length);
#endif
    return status;
}

//todo
bool Xcvr_RxN(const Xcvr_T * p_xcvr, uint8_t * p_dest, size_t length)
{
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
    return p_xcvr->p_Xcvr->P_INTERFACE->RX_N(p_xcvr->p_Xcvr->P_CONTEXT, p_dest, length);
#endif
}

//todo
size_t Xcvr_TxMax(const Xcvr_T * p_xcvr, const uint8_t * p_srcBuffer, size_t srcSize)
{
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
    return p_xcvr->p_Xcvr->P_INTERFACE->TX_MAX(p_xcvr->p_Xcvr->P_CONTEXT, p_srcBuffer, srcSize);
#endif
}

size_t Xcvr_RxMax(const Xcvr_T * p_xcvr, uint8_t * p_destBuffer, size_t destSize)
{
    size_t rxCount;
#if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
    switch(p_xcvr->p_Xcvr->TYPE)
    {
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
        case XCVR_TYPE_INTERFACE:   rxCount = p_xcvr->p_Xcvr->P_INTERFACE->RX_MAX(p_xcvr->p_Xcvr->P_CONTEXT, p_destBuffer, destSize); break;
#endif
        case XCVR_TYPE_SERIAL:      rxCount = Serial_RecvMax(p_xcvr->p_Xcvr->P_CONTEXT, p_destBuffer, destSize); break;
        case XCVR_TYPE_I2C:         rxCount = 0U; break;
        case XCVR_TYPE_SPI:         rxCount = 0U; break;
        case XCVR_TYPE_VIRTUAL:     rxCount = 0U; break;
        default:                    rxCount = 0U; break;
    }
#elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
    rxCount = p_xcvr->p_Xcvr->P_INTERFACE->RX_MAX(p_xcvr->p_Xcvr->P_CONTEXT, p_destBuffer, destSize);
#endif
    return rxCount;
}

size_t Xcvr_GetRxFullCount(const Xcvr_T * p_xcvr)
{
    size_t count;
#if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
    switch(p_xcvr->p_Xcvr->TYPE)
    {
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
        case XCVR_TYPE_INTERFACE:   count = p_xcvr->p_Xcvr->P_INTERFACE->GET_RX_FULL_COUNT(p_xcvr->p_Xcvr->P_CONTEXT); break;
#endif
        case XCVR_TYPE_SERIAL:      count = Serial_GetRxFullCount(p_xcvr->p_Xcvr->P_CONTEXT); break;
        case XCVR_TYPE_I2C:         count = 0U;    break;
        case XCVR_TYPE_SPI:         count = 0U;    break;
        case XCVR_TYPE_VIRTUAL:     count = 0U;    break;
        default:                    count = 0U; break;
    }
#elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
    count = p_xcvr->p_Xcvr->P_INTERFACE->GET_RX_FULL_COUNT(p_xcvr->p_Xcvr->P_CONTEXT);
#endif
    return count;
}

size_t Xcvr_GetTxEmptyCount(const Xcvr_T * p_xcvr)
{
    size_t count;
#if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
    switch(p_xcvr->p_Xcvr->TYPE)
    {
#if defined(CONFIG_XCVR_INTERFACE_VTABLE)
        case XCVR_TYPE_INTERFACE:   count = p_xcvr->p_Xcvr->P_INTERFACE->GET_TX_EMPTY_COUNT(p_xcvr->p_Xcvr->P_CONTEXT); break;
#endif
        case XCVR_TYPE_SERIAL:      count = Serial_GetTxEmptyCount(p_xcvr->p_Xcvr->P_CONTEXT); break;
        case XCVR_TYPE_I2C:         count = 0U;    break;
        case XCVR_TYPE_SPI:         count = 0U;    break;
        case XCVR_TYPE_VIRTUAL:     count = 0U;    break;
        default:                    count = 0U; break;
    }
#elif     defined(CONFIG_XCVR_INTERFACE_VTABLE_ONLY)
    count = p_xcvr->p_Xcvr->P_INTERFACE->GET_TX_EMPTY_COUNT(p_xcvr->p_Xcvr->P_CONTEXT);
#endif
    return count;
}


size_t Xcvr_FlushRxBuffer(const Xcvr_T * p_xcvr)
{


}

/* Experimental */
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
// uint8_t * Xcvr_AcquireTxBuffer(const Xcvr_T * p_xcvr)
// {
//     uint8_t * p_buffer;

//     switch(p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:  p_buffer = Serial_AcquireTxBuffer(p_xcvr->p_Xcvr->P_CONTEXT); break;
//         case XCVR_TYPE_I2C:     p_buffer = 0U; break;
//         case XCVR_TYPE_SPI:     p_buffer = 0U; break;
//         case XCVR_TYPE_VIRTUAL: p_buffer = 0U; break;
//         default:                p_buffer = 0U; break;
//     }
//     return p_buffer;
// }

// void Xcvr_ReleaseTxBuffer(const Xcvr_T * p_xcvr, size_t writeSize)
// {
//     switch(p_xcvr->p_Xcvr->TYPE)
//     {
//         case XCVR_TYPE_SERIAL:      Serial_ReleaseTxBuffer(p_xcvr->p_Xcvr->P_CONTEXT, writeSize); break;
//         case XCVR_TYPE_I2C:         break;
//         case XCVR_TYPE_SPI:         break;
//         case XCVR_TYPE_VIRTUAL:     break;
//         default: break;
//     }

// }
// #endif

//static inline void Xcvr_EnableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_EnableRxIsr(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableRxIsr(const Xcvr_T * p_xcvr){}