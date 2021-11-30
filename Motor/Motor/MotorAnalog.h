/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file 	.h
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/
#ifndef MOTOR_ANALOG_N_H
#define MOTOR_ANALOG_N_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#define MOTOR_ANALOG_CHANNEL_COMMON_COUNT 	9U
#define MOTOR_ANALOG_CHANNEL_MOTOR_COUNT 	7U

/*!
	@brief Virtual channel identifiers, index into arrays containing Analog channel
 */
typedef enum
{
//	MOTOR_ANALOG_CHANNEL_VPOS, 				/* V battery, V in */
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS_HIGH,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS_LOW,
	MOTOR_ANALOG_CHANNEL_HEAT_PCB,
	MOTOR_ANALOG_CHANNEL_VACC,				/* V accessories ~12V*/
	MOTOR_ANALOG_CHANNEL_VSENSE,			/* V analog sensors ~5V*/
	MOTOR_ANALOG_CHANNEL_THROTTLE,
	MOTOR_ANALOG_CHANNEL_BRAKE,
} MotorAnalog_ChannelCommon_T;

typedef enum
{
	MOTOR_ANALOG_CHANNEL_VPOS, 			/* Physically shared channel, but save results in contiguous memory */
	MOTOR_ANALOG_CHANNEL_VA,
	MOTOR_ANALOG_CHANNEL_VB,
	MOTOR_ANALOG_CHANNEL_VC,
	MOTOR_ANALOG_CHANNEL_IA,
	MOTOR_ANALOG_CHANNEL_IB,
	MOTOR_ANALOG_CHANNEL_IC,
	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,	/* Temperature */
} MotorAnalog_ChannelMotor_T;


typedef enum
{
	MOTOR_ANALOG_CONVERSION_BEMF_A_INDEX_VPOS = 0U,
} MotorAnalog_ConversionBemfAIndex_T;



//typedef const struct
//{
//	volatile analog_adcdata_t VBus_ADCU;
//	volatile analog_adcdata_t Va_ADCU;
//	volatile analog_adcdata_t Vb_ADCU;
//	volatile analog_adcdata_t Vc_ADCU;
//	volatile analog_adcdata_t Ia_ADCU;
//	volatile analog_adcdata_t Ib_ADCU;
//	volatile analog_adcdata_t Ic_ADCU;
//	volatile analog_adcdata_t HeatMotor_ADCU;
////	volatile analog_adcdata_t HeatMosfets_ADCU; //if per motor mosfet sensor is implemented
//}
//MotorAnalog_AdcResultBuffer_T;

//shared between instances
//typedef const struct
//{
//	volatile analog_adcdata_t VBUS_ADCU;
//	volatile analog_adcdata_t VACC_ADCU;
//	volatile analog_adcdata_t VSENSE_ADCU;
//	volatile analog_adcdata_t HEAT_PCB_ADCU;
//	volatile analog_adcdata_t HEAT_MOSFETS_H_ADCU;
//	volatile analog_adcdata_t HEAT_MOSFETS_L_ADCU;
//	volatile analog_adcdata_t THROTTLE_ADCU;
//	volatile analog_adcdata_t BRAKE_ADCU;
//}
//Motor_AdcMapCommon_T;

/*
 * Misra violation
 */
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IABC;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_FOC_REMAINDER;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_A_REPEAT;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_B;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_C;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_BEMF_REMAINDER;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_IDLE;

extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_MONITOR;
extern const Analog_ConversionVirtual_T MOTOR_ANALOG_VIRTUAL_USER;
#endif
