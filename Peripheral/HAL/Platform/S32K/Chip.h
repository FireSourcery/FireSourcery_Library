/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSoucery / The Firebrand Forge Inc

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
    @file     .h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef CHIP_PLATFORM_H
#define CHIP_PLATFORM_H

#include "External/S32K142/include/S32K142.h"

#include <stdint.h>
#include <stdbool.h>

/*
    Expected to be called directly from main module. no common control layer
*/

static inline void Chip_PCC_SetClockSource(uint8_t peripheralIndex, uint8_t clockSource)
{
    PCC->PCCn[peripheralIndex] &= ~PCC_PCCn_CGC_MASK;                                /* Ensure clk disabled for config */
    PCC->PCCn[peripheralIndex] |= PCC_PCCn_PCS(clockSource) | PCC_PCCn_CGC_MASK;     /* Clock Src = 3 (CLK_SRC_FIRC_DIV2) */
}

static inline void Chip_PCC_EnableClock(uint8_t peripheralIndex)
{
    PCC->PCCn[peripheralIndex] |= PCC_PCCn_CGC_MASK;
}

static inline void Chip_PCC_DisableClock(uint8_t peripheralIndex)
{
    PCC->PCCn[peripheralIndex] &= ~PCC_PCCn_CGC_MASK;
}

///*!
// * @brief Configures the Pin mux selection
// * Implements : port_mux_t_Class
// */
//typedef enum
//{
//    PORT_PIN_DISABLED            = 0U,  /*!< corresponding pin is disabled, but is used as an analog pin       */
//    PORT_MUX_AS_GPIO             = 1U,  /*!< corresponding pin is configured as GPIO                           */
//    PORT_MUX_ALT2                = 2U,  /*!< chip-specific                                                     */
//    PORT_MUX_ALT3                = 3U,  /*!< chip-specific                                                     */
//    PORT_MUX_ALT4                = 4U,  /*!< chip-specific                                                     */
//    PORT_MUX_ALT5                = 5U,  /*!< chip-specific                                                     */
//    PORT_MUX_ALT6                = 6U,  /*!< chip-specific                                                     */
//    PORT_MUX_ALT7                = 7U,  /*!< chip-specific                                                     */
//#if FEATURE_PINS_HAS_ADC_INTERLEAVE_EN
//    PORT_MUX_ADC_INTERLEAVE      = 8U   /*!< when selected, ADC Interleaved channel is connected to current pin
//                                         *   and disconnected to opposed pin
//                                         * ADC1_SE14-PTB15 | ADC1_SE15-PTB16 | ADC0_SE8-PTC0  | ADC0_SE9-PTC1
//                                         * ADC1_SE14-PTB0  | ADC1_SE15-PTB1  | ADC0_SE8-PTB13 | ADC0_SE9-PTB14 */
//#endif /* FEATURE_PINS_HAS_ADC_INTERLEAVE_EN */
//} port_mux_t;
static inline void Chip_Port_SetMux(PORT_Type * p_port, uint8_t pin, uint8_t mux)
{
    p_port->PCR[pin] = (p_port->PCR[pin] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(mux);
}

/*
 *
 */
static inline void Chip_InitLpuart(uint8_t lpuartId, uint8_t clockSource, PORT_Type * p_port, uint8_t pinTx, uint8_t pinRx)
{
    uint8_t pccIndex = lpuartId + PCC_LPUART0_INDEX;
    uint8_t irqN;

    switch (lpuartId)
    {
        case 0: irqN = LPUART0_RxTx_IRQn; break;
        case 1: irqN = LPUART1_RxTx_IRQn; break;
    }

    Chip_PCC_SetClockSource(pccIndex, clockSource);
    Chip_Port_SetMux(p_port, pinTx, 2U); //mux 2 for alt lpuart function
    Chip_Port_SetMux(p_port, pinRx, 2U);
    S32_NVIC->ISER[(irqN) >> 5U] = (1UL << ((irqN) & 0x1FU));
}

static inline void Chip_DeinitLpuart(uint8_t lpuartId, PORT_Type * p_port, uint8_t pinTx, uint8_t pinRx)
{
    uint8_t pccIndex = lpuartId + PCC_LPUART0_INDEX;
    Chip_Port_SetMux(p_port, pinTx, 0U);
    Chip_Port_SetMux(p_port, pinRx, 0U);
    Chip_PCC_DisableClock(pccIndex);
}

static inline void Chip_InitGpio(PORT_Type * p_port, uint8_t pinId)
{
    uint8_t portIndex = ((uint32_t)(p_port) - PORTA_BASE) / 0x1000U + PCC_PORTA_INDEX;

    Chip_PCC_EnableClock(portIndex);
    Chip_Port_SetMux(p_port, pinId, 0x01U);
}

static inline void Chip_DeinitGpio(PORT_Type * p_port, uint8_t pinId)
{
    Chip_Port_SetMux(p_port, pinId, 0U);
//    PCC->PCCn[p_config->PORT_INDEX] &= ~PCC_PCCn_CGC_MASK; //defer deinit to cpu deinit to avoid deinit shared port pins
}


#endif
