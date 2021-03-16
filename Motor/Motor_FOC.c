/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Motor_FOC.h
    @author FireSoucery
    @brief  Motor FOC submodule. FOC control functions.
    @version V0
*/
/**************************************************************************/
#include "Motor_FOC.h"
#include "Motor.h"

#include "Config.h"
#include "Default.h"
//#include "Instance/MotorAnalog.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Math/Q/QFrac16.h"
#include "Math/Linear/Linear.h"

#include <stdbool.h>
#include <stdint.h>

/*!
	@brief Init motor with FOC parameters
	@param p_motor
 */
void Motor_FOC_LoadParameters(Motor_T * p_motor)
{
#ifdef CONFIG_MOTOR_LOAD_PARAMETERS_DEFAULT
	p_motor->Parameters.FocOpenLoopVq = MOTOR_PARAMETER_DEFAULT_FOC_OPEN_LOOP_VQ;

#elif defined(CONFIG_MOTOR_LOAD_PARAMETERS_FLASH)
	p_motor->Parameters.FocOpenLoopVq = Flash_Read(&p_motor->Flash, MEMORY_ADDRESS_FOC_OPEN_LOOP_VOLTAGE);
#endif
}



/******************************************************************************/
/*!
	@addtogroup FocAnalogGroup
	FOC ADC conversion
	@{
*/
/******************************************************************************/

void Motor_FOC_ConvertIa(Motor_T * p_motor)
{
	//Filter here if needed
//	qfrac16_t i_temp = Linear_Convert(&p_motor->UnitIa_Frac16, p_motor->AnalogChannelResults[MOTOR_VIRTUAL_CHANNEL_IA]);
//	FOC_SetIa(&p_motor->Foc, i_temp);
}

void Motor_FOC_ConvertIb(Motor_T * p_motor)
{
//	qfrac16_t i_temp = Linear_Convert(&p_motor->UnitIb_Frac16, p_motor->AnalogChannelResults[MOTOR_VIRTUAL_CHANNEL_IB]);
//	FOC_SetIb(&p_motor->Foc, i_temp);
}

void Motor_FOC_ConvertIc(Motor_T * p_motor)
{
//	qfrac16_t i_temp = Linear_Convert(&p_motor->UnitIc_Frac16, p_motor->AnalogChannelResults[MOTOR_VIRTUAL_CHANNEL_IC]);
//	FOC_SetIc(&p_motor->Foc, i_temp);
}

void Motor_FOC_ActivateCurrentFeedbackWrapper(Motor_T * p_motor);

/*!
	@brief FOC mode ADC channels to be sampled

	FOC mode use 1 large sample group. Sample all channels sequentially on PWM trigger
	Measure Ia, Ib, Ic first, so there is longer time for FOC calculations
	remaining channels will continue processing as FOC calculations are performed
 */
const uint8_t FOC_ANALOG_CONVERSION_CHANNELS_ANGLE_CONTROL[] =
{
	[0] = MOTOR_VIRTUAL_CHANNEL_IA,
	[1] = MOTOR_VIRTUAL_CHANNEL_IB,
	[2] = MOTOR_VIRTUAL_CHANNEL_IC,
};

void (* FOC_ANALOG_CONVERSION_CHANNEL_FUNCTIONS_ANGLE_CONTROL[])(Motor_T *) =
{
	[0] = Motor_FOC_ConvertIa,
	[1] = Motor_FOC_ConvertIb,
	[2] = Motor_FOC_ConvertIc, //or put start run wrapper here?
};

Analog_Conversion_T FOC_ANALOG_CONVERSION_ANGLE_CONTROL =
{
	.p_VirtualChannels 	= FOC_ANALOG_CONVERSION_CHANNELS_ANGLE_CONTROL,
	.p_OnCompleteChannels = (void (*(*))(void *))FOC_ANALOG_CONVERSION_CHANNEL_FUNCTIONS_ANGLE_CONTROL,
	.OnCompleteConversion = (void (*)(void *)) &Motor_FOC_ActivateCurrentFeedbackWrapper,
};

const uint8_t FOC_ANALOG_CONVERSION_CHANNELS_2[] =
{
	MOTOR_VIRTUAL_CHANNEL_VA,
	MOTOR_VIRTUAL_CHANNEL_VB,
	MOTOR_VIRTUAL_CHANNEL_VC,

	MOTOR_VIRTUAL_CHANNEL_VBUS,
	MOTOR_VIRTUAL_CHANNEL_VACC,
	MOTOR_VIRTUAL_CHANNEL_VSENS,

	MOTOR_VIRTUAL_CHANNEL_TEMP_MOTOR,
	MOTOR_VIRTUAL_CHANNEL_TEMP_CONTROLLER,

	MOTOR_VIRTUAL_CHANNEL_THROTTLE,
	MOTOR_VIRTUAL_CHANNEL_BRAKE,
};

Analog_Conversion_T FOC_ANALOG_CONVERSION_2 =
{
	.p_VirtualChannels 	= FOC_ANALOG_CONVERSION_CHANNELS_2,
};

void Motor_FOC_ActivateCurrentFeedbackWrapper(Motor_T * p_motor)
{
	Analog_ActivateConversion(&p_motor->Analog, &FOC_ANALOG_CONVERSION_2);
	Motor_FOC_ActivateCurrentFeedback(p_motor);
}

/******************************************************************************/
/*! @} */
/******************************************************************************/


