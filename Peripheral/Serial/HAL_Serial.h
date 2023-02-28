/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
    @file     HAL_Serial.h
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef HAL_SERIAL_H
#define HAL_SERIAL_H

#if     defined(CONFIG_HAL_SERIAL_PATH)
    #include STR(CONFIG_HAL_SERIAL_PATH/HAL_Serial.h)
#elif     defined(CONFIG_HAL_PERIPHERAL_PATH) || defined(CONFIG_HAL_PERIPHERAL_PLATFORM)
    #include "Peripheral/HAL/HAL_Peripheral.h"
    #include HAL_PERIPHERAL_PATH(HAL_Serial.h)
#else
    #error "HAL_Serial included but undefined."
    // typedef void HAL_Serial_T; /* Peripheral register map provided by chip manufacturer */
    // static inline void HAL_Serial_WriteTxChar(HAL_Serial_T * p_hal, uint8_t txChar) {}
    // static inline uint8_t HAL_Serial_ReadRxChar(const HAL_Serial_T * p_hal) {}
    // static inline uint8_t HAL_Serial_ReadTxEmptyCount(const HAL_Serial_T * p_hal) {}
    // static inline uint8_t HAL_Serial_ReadRxFullCount(const HAL_Serial_T * p_hal) {}
    // static inline bool HAL_Serial_ReadRxOverrun(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_ClearRxErrors(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_EnableTxInterrupt(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_DisableTxInterrupt(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_EnableRxInterrupt(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_DisableRxInterrupt(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_WriteTxSwitch(HAL_Serial_T * p_hal, bool enable) {}
    // static inline void HAL_Serial_WriteRxSwitch(HAL_Serial_T * p_hal, bool enable) {}
    // static inline void HAL_Serial_InitBaudRate(HAL_Serial_T * p_hal, uint32_t baudRate) {}
    // static inline void HAL_Serial_Init(HAL_Serial_T * p_hal) {}
    // static inline void HAL_Serial_Deinit(HAL_Serial_T * p_hal) {}
#endif

#endif
