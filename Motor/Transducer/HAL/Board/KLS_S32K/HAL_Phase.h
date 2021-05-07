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
	@file 	HAL_Phase.h
	@author FireSoucery
	@brief
			Use compile time constant for improved performance.
	@version V0
*/
/******************************************************************************/
#ifndef HAL_PHASE_BOARD_H
#define HAL_PHASE_BOARD_H

//#include "External/S32K142/include/S32K142.h"
#include "SDK/platform/devices/S32K142/include/S32K142.h"

#include "SDK/platform/drivers/inc/pins_driver.h"
#include "SDK/platform/drivers/inc/interrupt_manager.h"
#include "SDK/platform/drivers/inc/ftm_common.h"
#include "SDK/platform/drivers/inc/ftm_pwm_driver.h"

#include <stdint.h>
#include <stdbool.h>

typedef const struct
{
//	uint32_t InstanceID;
} HAL_Phase_T;

static inline void HAL_Phase_WriteDuty(const HAL_Phase_T * p_phase, uint32_t pwmA, uint32_t pwmB, uint32_t pwmC)
{
	(void)p_phase;

	FTM0->CONTROLS[5].CnV = FTM_CnV_VAL(pwmA);
	FTM0->CONTROLS[6].CnV = FTM_CnV_VAL(pwmB);
	FTM0->CONTROLS[7].CnV = FTM_CnV_VAL(pwmC);
//	FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;
}

/*
	KLS board uses additional hw enable/disable pin
 */
static inline void HAL_Phase_WriteState(const HAL_Phase_T * p_phase, bool isOnPwmA, bool isOnPwmB, bool isOnPwmC)
{
	(void)p_phase;

	/* Must set all mask as a single reg write */
	FTM0->OUTMASK = (FTM0->OUTMASK & (~(FTM_OUTMASK_CH5OM_MASK | FTM_OUTMASK_CH6OM_MASK | FTM_OUTMASK_CH7OM_MASK))) |
					FTM_OUTMASK_CH5OM(!isOnPwmA) | FTM_OUTMASK_CH6OM(!isOnPwmB) | FTM_OUTMASK_CH7OM(!isOnPwmC);
//	FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;

	/*
	 * PTE0 -> ENA
	 * PTE1 -> ENB
	 * PTE2 -> ENC
	 */
	PTE->PDOR = (PTE->PDOR & (~(((uint32_t)1U << 0U) | ((uint32_t)1U << 1U) | ((uint32_t)1U << 2U)))) |
				((uint32_t)isOnPwmA << 0U) | ((uint32_t)isOnPwmB << 1U) | ((uint32_t)isOnPwmC << 2U);
}

/*
	true is inverted, false is noninverted
 */
static inline void HAL_Phase_WriteInvertPolarity(const HAL_Phase_T * p_phase, bool isInvPwmA, bool isInvPwmB, bool isInvPwmC)
{
	(void)p_phase;

	FTM0->POL = (FTM0->POL & (~(FTM_POL_POL5_MASK | FTM_POL_POL6_MASK | FTM_POL_POL7_MASK))) |
				FTM_OUTMASK_CH5OM(!isInvPwmA) | FTM_OUTMASK_CH6OM(!isInvPwmB) | FTM_OUTMASK_CH7OM(!isInvPwmC);
//	FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;
}

static inline void HAL_Phase_ClearInterrupt(const HAL_Phase_T * p_phase)
{
	(void)p_phase;
	FTM0->SC &= ~FTM_SC_TOF_MASK;
}

/*
 * PWM_FREQ = 20000
 *
 * FTM0_SRC_FREQ = 80000000
 * FTM0_DIV = 1
 * FTM0_CLK_FREQ = FTM0_SRC_FREQ / FTM0_DIV = 80000000
 *
 * FTM0_PERIOD = FTM0_CLK_FREQ/PWM_FREQ = 4000
 * FTM0_MODULO = FTM0_PERIOD/2 = 4000/2 [Center-aligned mode]
 *
 * PWM_PERIOD = FTM0_MODULO = 2000
 *
 * PWM ISR occur at the beginning of the PWM duty cyle, pulse is low.
 */
static inline void HAL_Phase_Init(const HAL_Phase_T * p_phase)
{
	(void)p_phase;

	///* Generate array of configured pin structures */
	//const pin_settings_config_t pin_mux_InitConfigArr0[6U] = {
	// {
	//		.base            = PORTE,
	//		.pinPortIdx      = 0U,
	//		.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	//		.driveSelect     = PORT_HIGH_DRIVE_STRENGTH,
	//		.passiveFilter   = false,
	//		.mux             = PORT_MUX_AS_GPIO,
	//		.pinLock         = false,
	//		.intConfig       = PORT_DMA_INT_DISABLED,
	//		.clearIntFlag    = false,
	//		.gpioBase        = PTE,
	//		.direction       = GPIO_OUTPUT_DIRECTION,
	//		.digitalFilter   = false,
	//		.initValue       = 0U,
	//	},
	//	{
	//		.base            = PORTE,
	//		.pinPortIdx      = 1U,
	//		.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	//		.driveSelect     = PORT_HIGH_DRIVE_STRENGTH,
	//		.passiveFilter   = false,
	//		.mux             = PORT_MUX_AS_GPIO,
	//		.pinLock         = false,
	//		.intConfig       = PORT_DMA_INT_DISABLED,
	//		.clearIntFlag    = false,
	//		.gpioBase        = PTE,
	//		.direction       = GPIO_OUTPUT_DIRECTION,
	//		.digitalFilter   = false,
	//		.initValue       = 0U,
	//	},
	//	{
	//		.base            = PORTE,
	//		.pinPortIdx      = 2U,
	//		.pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	//		.driveSelect     = PORT_HIGH_DRIVE_STRENGTH,
	//		.passiveFilter   = false,
	//		.mux             = PORT_MUX_AS_GPIO,
	//		.pinLock         = false,
	//		.intConfig       = PORT_DMA_INT_DISABLED,
	//		.clearIntFlag    = false,
	//		.gpioBase        = PTE,
	//		.direction       = GPIO_OUTPUT_DIRECTION,
	//		.digitalFilter   = false,
	//		.initValue       = 0U,
	//	},
	//    {
	//        .base            = PORTB,
	//        .pinPortIdx      = 5U,
	//        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	//        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
	//        .passiveFilter   = false,
	//        .mux             = PORT_MUX_ALT2,
	//        .pinLock         = false,
	//        .intConfig       = PORT_DMA_INT_DISABLED,
	//        .clearIntFlag    = false,
	//        .gpioBase        = NULL,
	//        .digitalFilter   = false,
	//    },
	//    {
	//        .base            = PORTE,
	//        .pinPortIdx      = 8U,
	//        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	//        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
	//        .passiveFilter   = false,
	//        .mux             = PORT_MUX_ALT2,
	//        .pinLock         = false,
	//        .intConfig       = PORT_DMA_INT_DISABLED,
	//        .clearIntFlag    = false,
	//        .gpioBase        = NULL,
	//        .digitalFilter   = false,
	//    },
	//    {
	//        .base            = PORTE,
	//        .pinPortIdx      = 9U,
	//        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
	//        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
	//        .passiveFilter   = false,
	//        .mux             = PORT_MUX_ALT2,
	//        .pinLock         = false,
	//        .intConfig       = PORT_DMA_INT_DISABLED,
	//        .clearIntFlag    = false,
	//        .gpioBase        = NULL,
	//        .digitalFilter   = false,
	//    },
	//};


//	PINS_DRV_Init(6U, pin_mux_InitConfigArr0);

	PCC->PCCn[PCC_FTM0_INDEX] = (PCC->PCCn[PCC_FTM0_INDEX] & ~PCC_PCCn_PCS_MASK) | PCC_PCCn_PCS(0x06) | PCC_PCCn_CGC_MASK;

	FTM0->OUTMASK = FTM_OUTMASK_CH0OM_MASK | FTM_OUTMASK_CH1OM_MASK | FTM_OUTMASK_CH2OM_MASK | FTM_OUTMASK_CH3OM_MASK | FTM_OUTMASK_CH4OM_MASK | FTM_OUTMASK_CH5OM_MASK | FTM_OUTMASK_CH6OM_MASK | FTM_OUTMASK_CH7OM_MASK;
	FTM0->MODE = FTM_MODE_WPDIS_MASK;//FTM_MODE_FTMEN_MASK | FTM_MODE_WPDIS_MASK | FTM_MODE_PWMSYNC_MASK;
	FTM0->MOD = FTM_MOD_MOD(2000U);
	FTM0->CONF = FTM_CONF_ITRIGR_MASK;
	FTM0->CONTROLS[5U].CnSC = FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[6U].CnSC = FTM_CnSC_ELSB_MASK;
	FTM0->CONTROLS[7U].CnSC = FTM_CnSC_ELSB_MASK;
	FTM0->SC = FTM_SC_CLKS(0x01) | FTM_SC_CPWMS_MASK | FTM_SC_TOIE_MASK | FTM_SC_PWMEN5_MASK | FTM_SC_PWMEN6_MASK | FTM_SC_PWMEN7_MASK;
//	FTM0->SYNC = FTM_SYNC_SWSYNC_MASK;
//	FTM0->SYNCONF = FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_SWWRBUF_MASK;
	INT_SYS_EnableIRQ(FTM0_Ovf_Reload_IRQn);
}

#endif
