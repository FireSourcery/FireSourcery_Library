#ifndef HAL_TRANSDUCER_MOTOR_H
#define HAL_TRANSDUCER_MOTOR_H

#ifdef CONFIG_HAL_KLS_S32K

	#include "Board/KLS_S32K/HAL_BEMF.h"
	#include "Board/KLS_S32K/HAL_Phase.h"
	#include "Board/KLS_S32K/HAL_Hall.h"

#elif defined(CONFIG_HAL_LIBRARY_XYZ)
	#include "Board/XYZ/HAL_Hardware.h"

#endif

//#if defined(CONFIG_HAL_LIBRARY_DEFINED)
//	#include "Peripheral/HAL/HAL.h"
//
//#elif defined(CONFIG_HAL_USER_DEFINED)
//	#include "HAL.h"
//
//#elif defined(CONFIG_MOTOR_HAL_LIBRARY_DEFINED)
//	#include "Peripheral/HAL/HAL.h"
//
//#elif defined(CONFIG_MOTOR_HAL_USER_DEFINED)
//	#include "HAL.h"
//
//#elif defined(CONFIG_MOTOR_HAL_XYZ)
//	#include "HAL/Board/XYZ/Pins.h"
//#endif

#endif
