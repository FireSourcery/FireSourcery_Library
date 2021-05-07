#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#if defined(CONFIG_HAL_LIBRARY_DEFINED)
	#include "Peripheral/HAL/HAL.h"

#elif defined(CONFIG_HAL_USER_DEFINED)
	#include "HAL.h"

#elif defined(CONFIG_MOTOR_HAL_LIBRARY_DEFINED)
	#include "Peripheral/HAL/HAL.h"

#elif defined(CONFIG_MOTOR_HAL_USER_DEFINED)
	#include "HAL.h"

#elif defined(CONFIG_MOTOR_HAL_XYZ)
	#include "HAL/Board/XYZ/Pins.h"
#endif

#endif
