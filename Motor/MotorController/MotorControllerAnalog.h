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
    @file 	.h
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#ifndef MOT_ANALOG_H
#define MOT_ANALOG_H

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#define MOT_ANALOG_CHANNEL_COUNT 	9U

/*!
	@brief Virtual channel identifiers, index into arrays containing Analog channel
 */
typedef enum
{
	MOT_ANALOG_CHANNEL_VPOS, 				/* V battery, V in */
	MOT_ANALOG_CHANNEL_VACC,				/* V accessories ~12V*/
	MOT_ANALOG_CHANNEL_VSENSE,				/* V analog sensors ~5V*/
	MOT_ANALOG_CHANNEL_HEAT_PCB,
	MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP,
	MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT,
	MOT_ANALOG_CHANNEL_THROTTLE,
	MOT_ANALOG_CHANNEL_BRAKE,
	MOT_ANALOG_CHANNEL_HEAT_MOSFETS = MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP,
}
MotAnalog_Channel_T;

typedef union
{
	struct
	{
		analog_adcresult_t VPos_ADCU;
		analog_adcresult_t VAcc_ADCU;
		analog_adcresult_t VSense_ADCU;
		analog_adcresult_t HeatPcb_ADCU;
		union
		{
			analog_adcresult_t HeatMosfets_ADCU;
			analog_adcresult_t HeatMosfetsTop_ADCU;
		};
		analog_adcresult_t HeatMosfetsBot_ADCU;
		analog_adcresult_t Throttle_ADCU;
		analog_adcresult_t Brake_ADCU;
	};
	analog_adcresult_t Channels[MOT_ANALOG_CHANNEL_COUNT];
}
MotAnalog_Results_T;

typedef struct
{
	union
	{
		struct
		{
			const AnalogN_Conversion_T CONVERSION_VPOS;
			const AnalogN_Conversion_T CONVERSION_VACC;
			const AnalogN_Conversion_T CONVERSION_VSENSE;
			const AnalogN_Conversion_T CONVERSION_HEAT_PCB;
			union
			{
				const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS;
				const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_TOP;
			};
			const AnalogN_Conversion_T CONVERSION_HEAT_MOSFETS_BOT;
			const AnalogN_Conversion_T CONVERSION_THROTTLE;
			const AnalogN_Conversion_T CONVERSION_BRAKE;
		};
		AnalogN_Conversion_T CONVERSIONS[MOT_ANALOG_CHANNEL_COUNT];
	};

	const AnalogN_AdcFlags_T ADCS_GROUP_USER;
	const AnalogN_AdcFlags_T ADCS_GROUP_V;
	const AnalogN_AdcFlags_T ADCS_GROUP_HEAT;
}
MotAnalog_Conversions_T;



#endif
