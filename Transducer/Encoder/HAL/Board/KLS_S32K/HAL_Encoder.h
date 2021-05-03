/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

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
	@file 	HAL_Encoder.h
	@author FireSoucery
	@brief 	Encoder Timer Counter HAL for S32K
	@version V0
*/
/******************************************************************************/
#ifndef HAL_ENCODER_BOARD_H
#define HAL_ENCODER_BOARD_H

//#include "External/S32K142/include/S32K142.h"
#include "SDK/platform/devices/S32K142/include/S32K142.h"

#include "SDK/platform/drivers/inc/pins_driver.h"
#include "SDK/platform/drivers/inc/interrupt_manager.h"
#include "SDK/platform/drivers/inc/ftm_common.h"
#include "SDK/platform/drivers/inc/ftm_pwm_driver.h"
#include "SDK/platform/drivers/inc/ftm_qd_driver.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
//	uint8_t InstanceID; /* KLS has only 1 possible encoder */
} HAL_Encoder_T;

/*
 * TimerCounter can also captures Hall B. (Still use CaptureT with Hall)
 */

static inline uint32_t HAL_Encoder_ReadTimerCounter(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return FTM2->CNT;
}

static inline void HAL_Encoder_WriteTimerCounter(const HAL_Encoder_T * p_encoder, uint32_t count)
{
	(void) p_encoder;
	FTM2->CNT = count;
}

static inline void HAL_Encoder_WriteTimerCounterMax(const HAL_Encoder_T * p_encoder, uint32_t max)
{
	(void)p_encoder;
	FTM2->MOD = max;
}

static inline void HAL_Encoder_WriteTimerCounterFreq(const HAL_Encoder_T * p_encoder, uint32_t freq)
{
	(void)p_encoder;
//	FTM2-> = freq; todo
}

static inline bool HAL_Encoder_ReadDirection(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return (bool)(FTM2->QDCTRL & FTM_QDCTRL_QUADIR_MASK);
}

/*
 * Works only for PollingCaptureT mode. shared pin with Hall B on KLS_S32K.
 */
static inline bool HAL_Encoder_ReadPhaseA(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 5U));
}

static inline bool HAL_Encoder_ReadPhaseB(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
	return (bool)(PTE->PDIR & ((uint32_t)1U << 4U));
}

/*
	SW Polling capture mode? Or HW capture Hall B Only
 */
static inline void HAL_Encoder_ConfigCaptureTime(const HAL_Encoder_T * p_encoder)
{
	(void) p_encoder;

	/* Polling CaptureT Config */
	const pin_settings_config_t pin_mux_InitConfigArr1[3U] = {
		{
			.base            = PORTE,
			.pinPortIdx      = 4U,
			.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
			.driveSelect     = PORT_LOW_DRIVE_STRENGTH,
			.passiveFilter   = false,
			.mux             = PORT_MUX_AS_GPIO,
			.pinLock         = false,
			.intConfig       = PORT_DMA_INT_DISABLED,
			.clearIntFlag    = false,
			.gpioBase        = PTE,
			.direction       = GPIO_INPUT_DIRECTION,
			.digitalFilter   = false,
			.initValue       = 0U,
		},
		{
			.base            = PORTE,
			.pinPortIdx      = 5U,
			.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
			.driveSelect     = PORT_LOW_DRIVE_STRENGTH,
			.passiveFilter   = false,
			.mux             = PORT_MUX_AS_GPIO,
			.pinLock         = false,
			.intConfig       = PORT_DMA_INT_DISABLED,
			.clearIntFlag    = false,
			.gpioBase        = PTE,
			.direction       = GPIO_INPUT_DIRECTION,
			.digitalFilter   = false,
			.initValue       = 0U,
		},
		{
			.base            = PORTE,
			.pinPortIdx      = 10U,
			.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
			.driveSelect     = PORT_LOW_DRIVE_STRENGTH,
			.passiveFilter   = false,
			.mux             = PORT_MUX_AS_GPIO,
			.pinLock         = false,
			.intConfig       = PORT_DMA_INT_DISABLED,
			.clearIntFlag    = false,
			.gpioBase        = PTE,
			.direction       = GPIO_INPUT_DIRECTION,
			.digitalFilter   = false,
			.initValue       = 0U,
		}
	};


	/*
	 * CaptureT Init
	 */
	(FTM2->QDCTRL) &= ~(1UL << FTM_QDCTRL_QUADEN_SHIFT);
	INT_SYS_DisableIRQ(FTM2_Ovf_Reload_IRQn);

	PINS_DRV_Init(3U, pin_mux_InitConfigArr1);
}

/*
 * Enocder Res still needs to be configured.
 */
static inline void HAL_Encoder_ConfigCaptureCount(const HAL_Encoder_T * p_encoder)
{
	(void) p_encoder;

	const pin_settings_config_t pin_mux_InitConfigArr0[2] = {
	    {
	        .base            = PORTE,
	        .pinPortIdx      = 4U,
	        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
	        .passiveFilter   = false,
	        .mux             = PORT_MUX_ALT3,
	        .pinLock         = false,
	        .intConfig       = PORT_DMA_INT_DISABLED,
	        .clearIntFlag    = false,
	        .gpioBase        = NULL,
	        .digitalFilter   = false,
	    },
	    {
	        .base            = PORTE,
	        .pinPortIdx      = 5U,
	        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
	        .passiveFilter   = false,
	        .mux             = PORT_MUX_ALT3,
	        .pinLock         = false,
	        .intConfig       = PORT_DMA_INT_DISABLED,
	        .clearIntFlag    = false,
	        .gpioBase        = NULL,
	        .digitalFilter   = false,
	    },
	};

	const ftm_user_config_t flexTimer_qd_1_InitConfig =
	{
	    {
	        true,    /* Software trigger state */
	        false,  /* Hardware trigger 1 state */
	        false,  /* Hardware trigger 2 state */
	        false,  /* Hardware trigger 3 state */
	        false, /* Max loading point state */
	        false, /* Min loading point state */
	        FTM_SYSTEM_CLOCK, /* Update mode for INVCTRL register */
	        FTM_SYSTEM_CLOCK, /* Update mode for SWOCTRL register */
	        FTM_SYSTEM_CLOCK, /* Update mode for OUTMASK register */
	        FTM_SYSTEM_CLOCK, /* Update mode for CNTIN register */
	        false, /* Automatic clear of the trigger*/
	        FTM_UPDATE_NOW, /* Synchronization point */
	    },
	        FTM_MODE_QUADRATURE_DECODER, /*!< Mode of operation for FTM */
	        FTM_CLOCK_DIVID_BY_1, /* FTM clock prescaler */
	        FTM_CLOCK_SOURCE_SYSTEMCLK,   /* FTM clock source */
	        FTM_BDM_MODE_11, /* FTM debug mode */
	        true,    /* Interrupt state */
	        false     /* Initialization trigger */
	};

	const ftm_quad_decode_config_t flexTimer_qd_1_QuadDecoderConfig =
	{
	    FTM_QUAD_PHASE_ENCODE, /* Quadrature decoder mode */
	    0U, /* Initial counter value */
	    8191U, /* Maximum counter value */
	    {
	        false, /* Phase A input filter */
	        0U, /* Phase A input value */
	        FTM_QUAD_PHASE_NORMAL, /* Phase A polarity */
	    },
	    {
	        false, /* Phase B input filter */
	        0U, /* Phase B input value */
	        FTM_QUAD_PHASE_NORMAL, /* Phase B polarity */
	    }
	};

	ftm_state_t ftm2State;

	/*
	 * CaptureD Init
	 */
	PINS_DRV_Init(2U, pin_mux_InitConfigArr0);

	FTM_DRV_Init(2U, &flexTimer_qd_1_InitConfig, &ftm2State); 	/* FTM2 module initialized to work in quadrature decoder mode, specifically PhaseA and PhaseB mode */
	FTM_DRV_QuadDecodeStart(2U, &flexTimer_qd_1_QuadDecoderConfig);
	INT_SYS_EnableIRQ(FTM2_Ovf_Reload_IRQn);
}

static inline void HAL_Encoder_Init(const HAL_Encoder_T * p_encoder)
{
	(void)p_encoder;
//	HAL_Encoder_ConfigCaptureCount(0U);
}

#endif
