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
    @file   HAL_ADC.h
    @author
    @brief

*/
/******************************************************************************/
#ifndef HAL_ADC_PLATFORM_H
#define HAL_ADC_PLATFORM_H

#include "KE0x.h"

#include <stdint.h>
#include <stdbool.h>

#define HAL_ADC_FIFO_LENGTH_MAX 8U

typedef ADC_Type HAL_ADC_T;

static inline uint32_t HAL_ADC_ReadResult(const HAL_ADC_T * p_hal, uint32_t pinChannel) { (void)pinChannel; return p_hal->R; }

static inline void HAL_ADC_Activate(HAL_ADC_T * p_hal, uint32_t pinChannel) { p_hal->SC1 = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(pinChannel); }

static inline void HAL_ADC_WriteFifoCount(HAL_ADC_T * p_hal, uint32_t count) { p_hal->SC4 = ADC_SC4_AFDEP(count - 1U); }
static inline uint8_t HAL_ADC_ReadFifoCount(HAL_ADC_T * p_hal) { return ((p_hal->SC4 & ADC_SC4_AFDEP_MASK) + 1U); }
static inline void HAL_ADC_WriteFifoPin(HAL_ADC_T * p_hal, uint32_t pinChannel) { p_hal->SC1 = ADC_SC1_ADCH(pinChannel); }
static inline void HAL_ADC_ActivateFifo(HAL_ADC_T * p_hal, uint32_t pinChannel) { HAL_ADC_Activate(p_hal, pinChannel); }


/*
    Use NVIC interrupt for local critical section
    p_hal->SC1 &= ~ADC_SC1_AIEN_MASK; aborts conversion
*/
static inline void HAL_ADC_DisableInterrupt(HAL_ADC_T * p_hal) { (void)p_hal; NVIC_DisableIRQ(ADC_IRQn); }
static inline void HAL_ADC_EnableInterrupt(HAL_ADC_T * p_hal) { (void)p_hal; NVIC_EnableIRQ(ADC_IRQn); }

/*
    Clear interrupt - Clears on reading p_hal->R
*/
static inline void HAL_ADC_ClearConversionCompleteFlag(const HAL_ADC_T * p_hal)   { (void)p_hal; }
static inline bool HAL_ADC_ReadConversionCompleteFlag(const HAL_ADC_T * p_hal)    { return ((p_hal->SC1 & (uint32_t)ADC_SC1_COCO_MASK) != 0U); }
static inline bool HAL_ADC_ReadConversionActiveFlag(const HAL_ADC_T * p_hal)      { return ((p_hal->SC2 & (uint32_t)ADC_SC2_ADACT_MASK) != 0U); }

/*
    Any conversion in progress is aborted in the following cases:
    • A write to ADC_SC1 occurs.
    • The current conversion will be aborted and a new conversion will be initiated, if
    ADC_SC1[ADCH] are not all 1s and ADC_SC4[AFDEP] are all 0s.
    • The current conversion and the rest of conversions will be aborted and no new
    conversion will be initialed, if ADC_SC4[AFDEP] are not all 0s.
    • A new conversion will be initiated when the FIFO is re-fulfilled upon the levels
    indicated by the ADC_SC4[AFDEP] bits).
    • A write to ADC_SC2, ADC_SC3, ADC_SC4, ADC_CV occurs. This indicates a
    mode of operation change has occurred and the current and rest of conversions (when
    ADC_SC4[AFDEP] are not all 0s) are therefore invalid
*/
static inline void HAL_ADC_AbortConversion(HAL_ADC_T * p_hal) { p_hal->SC1 |= ADC_SC1_ADCH_MASK; }

/*
    111111b - Module is disabled

    The successive approximation converter subsystem is turned off when the pinChannel bits are all set (i.e.
    ADCH set to all 1s). This feature allows explicit disabling of the ADC and isolation of the input channel
    from all sources. Terminating continuous conversions this way prevents an additional single conversion
    from being performed. It is not necessary to set ADCH to all 1s to place the ADC in a low-power state
    when continuous conversions are not enabled because the module automatically enters a low-power
    state when a conversion completes.
*/
static inline void HAL_ADC_Deactivate(HAL_ADC_T * p_hal)
{
    p_hal->SC1 = (p_hal->SC1 & ~(ADC_SC1_COCO_MASK | ADC_SC1_AIEN_MASK)) | ADC_SC1_ADCH_MASK;
}

static inline void HAL_ADC_EnableHwTrigger(HAL_ADC_T * p_hal)     { p_hal->SC2 |= ADC_SC2_ADTRG_MASK; }
static inline void HAL_ADC_DisableHwTrigger(HAL_ADC_T * p_hal)    { p_hal->SC2 &= ~(ADC_SC2_ADTRG_MASK); }
static inline void HAL_ADC_DisableContinuousConversion(HAL_ADC_T * p_hal) { p_hal->SC1 &= ~ADC_SC1_ADCO_MASK; }
static inline void HAL_ADC_EnableContinuousConversion(HAL_ADC_T * p_hal)  { p_hal->SC1 |= ADC_SC1_ADCO_MASK; }

/*
    Reference Manual pg 326
    The total conversion time depends on the sample time (as determined by
    ADC_SC3[ADLSMP]), the MCU bus frequency, the conversion mode (8-bit, 10-bit or
    12-bit), and the frequency of the conversion clock (fADCK). After the module becomes
    active, sampling of the input begins.ADC_SC3[ADLSMP] selects between short (3.5
    ADCK cycles) and long (23.5 ADCK cycles) sample times.

    Conversion type ADICLK ADLSMP Max total conversion time
    Single or first continuous 8-bit                0x, 10 0     20 ADCK cycles + 5 bus clock cycles
    Single or first continuous 10-bit or 12-bit     0x, 10 0     23 ADCK cycles + 5 bus clock cycles
    => 3.125us @8Mhz

    Subsequent continuous 10-bit or 12-bit; fBUS > fADCK
        20 ADCK cycles
    => 2.5us @8Mhz

    High speed (ADLPC=0) fADCK 8.0 Max
    Asynchronous clock (ADACK). + 5us

    Minimal Init, Board sets clock configuration
*/
static inline void HAL_ADC_Init(HAL_ADC_T * p_hal)
{
    p_hal->SC2 = (p_hal->SC2 & ~ADC_SC2_REFSEL_MASK) | ADC_SC2_REFSEL(1U); /*!< Analog supply pin pair (VDDA/VSSA). >*/
    /*!< Alternate clock (ALTCLK). >*/
    /*!< 12-bit conversion (N = 12) >*/
    /*!< Divide ration = 1, and clock rate = Input clock. >*/
    // p_hal->SC3 = ADC_SC3_ADICLK(2U) | ADC_SC3_MODE(2U) | ADC_SC3_ADIV(0U);
    //   |= ADC_SC3_ADLPC_MASK; enableLowPower
    //   |= ADC_SC3_ADLSMP_MASK; enableLongSampleTime
}

#endif
