#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#include "HAL/HAL.h"

#if 	defined(CONFIG_HAL_MOTOR_USER_DEFINED) || defined(CONFIG_HAL_USER_DEFINED)
	#include HAL_PATH(CONFIG_HAL_USER_DIRECTORY, HAL_Motor.h)
#elif 	defined(CONFIG_HAL_MOTOR_LIBRARY_DEFINED) || defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include HAL_PATH_BOARD(Motor/HAL, HAL_Motor.h)
#endif

/*
	HAL_Motor_EnqueueConversionUser( );
	HAL_Motor_DequeueConversion( ); //starts chain if no conversion is currently active
 */
#endif
