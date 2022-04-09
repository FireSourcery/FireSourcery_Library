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
 * @brief initialize blink switch
 */
void Blinky_Init(Blinky_T * p_blinky)
{
	Pin_Output_Init(&p_blinky->PIN);
	Timer_Init(&p_blinky->Timer);
	Blinky_Off(p_blinky);
	//	p_blinky->PatternFunction =
}

void Blinky_Proc(Blinky_T * p_blinky)
{
	if(Timer_Poll(&p_blinky->Timer) == true)
	{
		if(Timer_GetIsOneShot(&p_blinky->Timer) == true)
		{
			Blinky_Toggle(p_blinky);

			if (p_blinky->Index == p_blinky->Max)
			{
				Timer_SetPeriodic(&p_blinky->Timer); //restore
				Timer_SetPeriod(&p_blinky->Timer, 0U); //change to set by pattern
			}
			else
			{
				p_blinky->Index++;
			}
		}
		else
		{
			//p_blinky->PatternFunction(p_blinky);
		}
	}
}

/*!
 * @brief
 *
 * @param[in]
 */
void Blinky_On(Blinky_T * p_blinky)		{p_blinky->IsOn = true;		Pin_Output_On(&p_blinky->PIN);}
void Blinky_Off(Blinky_T * p_blinky)	{p_blinky->IsOn = false;	Pin_Output_Off(&p_blinky->PIN);}
void Blinky_Toggle(Blinky_T * p_blinky) {(p_blinky->IsOn == true) ? Blinky_Off(p_blinky) : Blinky_On(p_blinky);}

/*
 * User initiated one shot
 */
void Blinky_Blink_OnOff(Blinky_T * p_blinky, uint32_t duration)
{
	Blinky_On(p_blinky);
	Timer_SetOneShot(&p_blinky->Timer);	//use timer state as pattern lockout, or change pattern function?
	Timer_StartPeriod(&p_blinky->Timer, duration);
}

void Blinky_Blink_Toggle(Blinky_T * p_blinky, uint32_t duration)
{
	Blinky_On(p_blinky);
	Timer_SetOneShot(&p_blinky->Timer);	//use timer state as pattern lockout, or change pattern function?
	Timer_StartPeriod(&p_blinky->Timer, duration);
}

void Blinky_Blink(Blinky_T * p_blinky, uint32_t duration)
{
	Blinky_Blink_OnOff(p_blinky, duration);
}

void Blinky_BlinkN(Blinky_T * p_blinky, uint32_t duration, uint8_t n)
{
	p_blinky->Index = 0;
	p_blinky->Max = n;
	Blinky_Blink_OnOff(p_blinky, duration);
}



//void Blinky_Pattern_PeriodicToggle(Blinky_T * p_blinky)
//{
//	if(p_blinky->IsOn == true)
//	{
//		Blinky_Off(p_blinky);
//		Timer_SetPeriod(&p_blinky->Timer, p_blinky->OffTime);
//	}
//	else
//	{
//		Blinky_On(p_blinky);
//		Timer_SetPeriod(&p_blinky->Timer, p_blinky->OnTime);
//	}
//}
//
//void Blinky_Pattern_SetPeriodicToggle(Blinky_T * p_blinky, uint32_t onTime, uint32_t offTime)
//{
//	p_blinky->OnTime = onTime;
//	p_blinky->OffTime = offTime;
////	p_blinky->PatternFunction = Blinky_ProcCycle;
//}

