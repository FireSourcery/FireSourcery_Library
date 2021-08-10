#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#include "HAL/HAL.h"

#if 	defined(CONFIG_HAL_MOTOR_USER_DEFINED) || defined(CONFIG_HAL_USER_DEFINED)
	#include HAL_PATH(CONFIG_HAL_USER_DIRECTORY, HAL_Motor.h)
#elif 	defined(CONFIG_HAL_MOTOR_LIBRARY_DEFINED) || defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include HAL_PATH_BOARD(Motor/HAL_Motor, HAL_Motor.h)
#endif



//static inline void HAL_Motor_LoadConversionBemfA(void * p_motor)
//{
//
//}
//
//static inline void HAL_Motor_LoadConversionBemfB(void * p_motor)
//{
//
//}
//
//static inline void HAL_Motor_LoadConversionBemfC(void * p_motor)
//{
//
//}
//
//static inline void HAL_Motor_LoadConversionIabc(void * p_motor)
//{
//
//}

//per controller init
//static inline void HAL_Motor_EnqueueConversionUser(void)
//{
//
//}
//
//static inline void HAL_Motor_EnqueueConversionIdle(void)
//{
//
//}
//
//static inline void HAL_Motor_EnqueueConversionMonitor(void)
//{
//
//}
//
//static inline void HAL_Motor_Init(void)
//{
//
//}

#endif
