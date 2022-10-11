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
	@file  	Blinky.c
	@author FireSourcery
	@brief 	Pin Indicator
	@version V0
*/
/******************************************************************************/
#include "Blinky.h"
#include "Peripheral/Pin/Pin.h"
#include "Utility/Timer/Timer.h"

/*!
	@brief initialize blink switch
*/
void Blinky_Init(Blinky_T * p_blinky)
{
	Pin_Output_Init(&p_blinky->Pin);
	Timer_Init(&p_blinky->Timer);
	Blinky_Off(p_blinky);
	p_blinky->PatternFunction = 0U;
}

static void Pattern_PeriodicToggle(Blinky_T * p_blinky);

/*!
	@brief
	@param[in]
*/
void Blinky_Proc(Blinky_T * p_blinky)
{
	if(Timer_Poll(&p_blinky->Timer) == true)
	{
		if(Timer_GetIsOneShot(&p_blinky->Timer) == true) /* Timer is in OneShot Mode */
		{
			if (p_blinky->Index < p_blinky->Max)
			{
				Pattern_PeriodicToggle(p_blinky);
				Timer_Restart(&p_blinky->Timer);
				p_blinky->Index++;
			}
			else
			{
				Blinky_Toggle(p_blinky);
				Timer_StartPeriodic(&p_blinky->Timer, p_blinky->OffTime);  /* Restore Periodic */
			}
		}
		else /* Timer is in Periodic Mode */
		{
			if (p_blinky->PatternFunction != 0U) { p_blinky->PatternFunction(p_blinky); }
		}
	}
}

void Blinky_On(Blinky_T * p_blinky)			{p_blinky->IsOn = true;		Pin_Output_High(&p_blinky->Pin);}
void Blinky_Off(Blinky_T * p_blinky)		{p_blinky->IsOn = false;	Pin_Output_Low(&p_blinky->Pin);}
void Blinky_Toggle(Blinky_T * p_blinky) 	{(p_blinky->IsOn == true) ? Blinky_Off(p_blinky) : Blinky_On(p_blinky);}
void Blinky_Disable(Blinky_T * p_blinky) 	{ Blinky_Off(p_blinky); Timer_Disable(&p_blinky->Timer); p_blinky->PatternFunction = Blinky_Disable;}
void Blinky_Stop(Blinky_T * p_blinky) 		{ Blinky_Disable(p_blinky);}

/* Always on first */
void Blinky_Blink_OnOff(Blinky_T * p_blinky, uint32_t duration)
{
	Blinky_On(p_blinky);
	Timer_StartOneShot(&p_blinky->Timer, duration);
}

/* Toggle */
void Blinky_Blink_Toggle(Blinky_T * p_blinky, uint32_t duration)
{
	Blinky_Toggle(p_blinky);
	Timer_StartOneShot(&p_blinky->Timer, duration);
}

void Blinky_Blink(Blinky_T * p_blinky, uint32_t onTime)
{
	p_blinky->Index = 0U;
	p_blinky->Max = 0U;
	Blinky_Blink_OnOff(p_blinky, onTime);
}

void Blinky_BlinkN(Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime, uint8_t nRepeat)
{
	p_blinky->Index = 0U;
	p_blinky->Max = nRepeat * 2U - 2U;
	p_blinky->OnTime = onTime;
	p_blinky->OffTime = offTime;
	Blinky_Blink_OnOff(p_blinky, onTime);
}

void Blinky_StartPeriodic(Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime)
{
	p_blinky->OnTime = onTime;
	p_blinky->OffTime = offTime;
	p_blinky->PatternFunction = Pattern_PeriodicToggle;
	Timer_StartPeriodic(&p_blinky->Timer, offTime);
}

static void Pattern_PeriodicToggle(Blinky_T * p_blinky)
{
	if(p_blinky->IsOn == true)
	{
		Blinky_Off(p_blinky);
		Timer_SetPeriod(&p_blinky->Timer, p_blinky->OffTime);
	}
	else
	{
		Blinky_On(p_blinky);
		Timer_SetPeriod(&p_blinky->Timer, p_blinky->OnTime);
	}
}