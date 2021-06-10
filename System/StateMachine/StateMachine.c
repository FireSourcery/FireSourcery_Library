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
	@file 	StateMachine.c
	@author FireSoucery
	@brief 	StateMachine module conventional function definitions
	@version V0
*/
/******************************************************************************/
#include "StateMachine.h"

#include <stdint.h>

/*
 * States const strut should be compile time def
 */

void StateMachine_Init
(
	StateMachine_T * p_stateMachine,
	const State_T * p_stateInitial,
	uint8_t transitionInputCount,
	uint8_t selfTransitionInputCount,
	const void * p_userData
)
{
	p_stateMachine->p_StateInitial 				= p_stateInitial;
	p_stateMachine->p_StateActive 				= p_stateInitial;
	p_stateMachine->TransitionInputCount 		= transitionInputCount;
	p_stateMachine->SelfTransitionInputCount 	= selfTransitionInputCount;
	p_stateMachine->p_UserData 					= p_userData;

#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_LIBRARY_DEFINED
	p_stateMachine->Mutex = 1U;
#endif
}

void StateMachine_Reset(StateMachine_T * p_stateMachine)
{
	if (Critical_MutexAquire(&p_stateMachine->Mutex))
	{
		p_stateMachine->p_StateActive 			= p_stateMachine->p_StateInitial;
		Critical_MutexRelease(&p_stateMachine->Mutex);
	}
}

