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
	@file 	Config.h
	@author FireSoucery
	@brief 	State machine module preprocessor configuration options and defaults.
	@version V0
*/
/******************************************************************************/
#ifndef CONFIG_STATE_MACHINE_H
#define CONFIG_STATE_MACHINE_H

#ifdef CONFIG_STATE_MACHINE_INPUT_ENUM_USER_DEFINED
/*
 * User provide typedef enum StateMachine_Input_Tag { ..., STATE_INPUT_RESERVED_NO_OP = 0xFFu } StateMachine_Input_T
 * #include defs before including State.h
 */
#elif defined (CONFIG_STATE_MACHINE_INPUT_UINT8)
/*
 * Default configuration
 */
#else
	#define CONFIG_STATE_MACHINE_INPUT_UINT8
#endif



#ifdef CONFIG_STATE_MACHINE_MULTITHREADED_OS_HAL


#elif defined(CONFIG_STATE_MACHINE_MULTITHREADED_USER_DEFINED)
/*
 * User provide functions
 * void Critical_Enter(void);
 * void Critical_Exit(void);
 */
#elif defined(CONFIG_STATE_MACHINE_MULTITHREADED_DISABLED)
/*
 * Default configuration
 */
#else
	#define CONFIG_STATE_MULTITHREADED_DISABLED
#endif

/*
 * Compile time defined array vs pointer
 *	if assuming only 1 state machine, compile time memory allocation is more convenient
 *	pointer allows varying sizes for multiple state machines
 */
//#if  (defined (CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY) && defined (STATE_TRANSITION_INPUT_COUNT) && defined (STATE_SELF_TRANSITION_INPUT_COUNT))
///*
// * User provide
// * #define STATE_TRANSITION_INPUT_COUNT
// * #define STATE_SELF_TRANSITION_INPUT_COUNT
// */
//#elif defined (CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_ARRAY)
//
//	#ifndef STATE_TRANSITION_INPUT_COUNT
//		#error "USER MUST DEFINE STATE_TRANSITION_INPUT_COUNT"
//	#endif
//
//	#ifndef STATE_SELF_TRANSITION_INPUT_COUNT
//		#error "USER MUST DEFINE STATE_SELF_TRANSITION_INPUT_COUNT"
//	#endif
//
//#elif defined (CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL)
///*
// * Default configuration
// */
//#else
//	#define CONFIG_STATE_MACHINE_MAPS_MEMORY_ALLOCATION_EXTERNAL
//#endif

#endif
