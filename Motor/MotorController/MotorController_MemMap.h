#ifndef MEM_MAP_MAP_H
#define MEM_MAP_MAP_H

#include "Motor/MotorController/MotorController.h"
#include "Motor/Motor/Motor.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"

#include <stdint.h>

//	const uint8_t Bytes[60]; //CONFIG_MOT_FLASH_ONCE_SIZE

//User Read Only, One-time flash if available
typedef const struct __attribute__ ((aligned (4U)))
{
	const uint8_t NAME[8U];
	const uint8_t NAME_EXT[8U];
	const uint8_t MANUFACTURE_DAY;
	const uint8_t MANUFACTURE_MONTH;
	const uint8_t MANUFACTURE_YEAR;
	const uint8_t MANUFACTURE_RESV;
	const uint32_t SERIAL_NUMBER;

	const uint16_t CONTROLLER_VOLTAGE;
}
MemMap_Once_T;//, * MotFlash_PtrOnce_T;

#define MEM_MAP_ONCE	((MemMap_Once_T *)CONFIG_MEM_MAP_ONCE_ADDRESS)

//typedef	const struct __attribute__ ((aligned (4U)))
//{
//
//}
//MemMap_OnceExt_T;
//
//#ifdef CONFIG_MEM_MAP_ONCE_EXT_ADDRESS
//	#define MEM_MAP_ONCE_EXT	((MemMap_OnceExt_T *)CONFIG_MEM_MAP_ONCE_EXT_ADDRESS)
//#else
//	#define MEM_MAP_ONCE_EXT	((MemMap_OnceExt_T *)(CONFIG_MEM_MAP_ONCE_ADDRESS + sizeof(MemMap_Once_T)))
//#endif

//User Read Only, nonconfigurable flash
typedef const struct __attribute__ ((aligned (8U)))
{
	const uint8_t SOFTWARE_VERSION[4];
	const uint16_t SENSOR_RATED_I;
	const uint16_t SENSOR_RATED_AD;
}
MemMap_ReadOnly_T;

#define MEM_MAP_READ_ONLY	((MemMap_ReadOnly_T *)CONFIG_MEM_MAP_READ_ONLY_ADDRESS)


//User Configurable, eeprom if available
typedef const struct __attribute__ ((aligned (4U)))
{
//	const MemMap_AppParameters_T APP;
	const MotorController_Params_T 		MOTOR_CONTROLLER;
	const MotAnalogMonitor_Params_T 	ANALOG_MONITOR;
	const MotAnalogUser_Params_T 		ANALOG_USER;
	const Motor_Params_T 				MOTORS[1U]; //#define CONFIG_MEM_MAP_MAP_MOTOR_COUNT 1U
}
MemMap_ReadWrite_T;

#define MEM_MAP_READ_WRITE	((MemMap_ReadWrite_T *)CONFIG_MEM_MAP_READ_WRITE_ADDRESS)


#endif
