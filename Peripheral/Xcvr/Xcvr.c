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
    @version V0
    @brief
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
    if (Xcvr_SetXcvr(p_xcvr, xcvrDefaultIndex) != true) { Xcvr_SetXcvr(p_xcvr, 0U); }
}

bool Xcvr_SetXcvr(Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
    bool status = xcvrIndex < p_xcvr->CONST.XCVR_TABLE_LENGTH;
    if (status == true) { p_xcvr->p_Xcvr = &p_xcvr->CONST.P_XCVR_TABLE[xcvrIndex]; }
    return status;
}

bool Xcvr_IsSet(const Xcvr_T * p_xcvr, uint8_t xcvrIndex)
{
    return ((xcvrIndex < p_xcvr->CONST.XCVR_TABLE_LENGTH) && (p_xcvr->p_Xcvr->P_CONTEXT == p_xcvr->CONST.P_XCVR_TABLE[xcvrIndex].P_CONTEXT));
}

bool Xcvr_IsValid(const Xcvr_T * p_xcvr, void * p_target)
{
    bool isValid = false;

    for (uint8_t iXcvr = 0U; iXcvr < p_xcvr->CONST.XCVR_TABLE_LENGTH; iXcvr++)
    {
        if (p_target == p_xcvr->CONST.P_XCVR_TABLE[iXcvr].P_CONTEXT) { isValid = true; break; }
    }

    return isValid;
}



/* Experimental */
// #if     defined(CONFIG_XCVR_INTERFACE_PERIPHERAL)
// uint8_t * Xcvr_AcquireTxBuffer(const Xcvr_T * p_xcvr)
// {
//     uint8_t * p_buffer;
//     return p_buffer;
// }

// void Xcvr_ReleaseTxBuffer(const Xcvr_T * p_xcvr, size_t writeSize)
// {
//
// }
// #endif

//static inline void Xcvr_EnableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableTx(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_EnableRxIsr(const Xcvr_T * p_xcvr){}
//static inline void Xcvr_DisableRxIsr(const Xcvr_T * p_xcvr){}
// size_t Xcvr_FlushRxBuffer(const Xcvr_T * p_xcvr)
// {
// }