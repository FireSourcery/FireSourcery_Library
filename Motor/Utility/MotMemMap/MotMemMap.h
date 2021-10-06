#ifndef MOT_MEM_MAP_H
#define MOT_MEM_MAP_H

#include "Motor/Motor/Motor.h"
//#include "Motor/MotorController/MotorController_Params.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
#include "Motor/Utility/MotUserOptions/MotUserOptions.h"

#include <stdint.h>

////	const uint8_t Bytes[60]; //CONFIG_MOT_FLASH_ONCE_SIZE
//
////User Read Only, One-time flash if available
//typedef const volatile struct __attribute__ ((aligned (4U)))
//{
//	const volatile uint8_t NAME[8U];
//	const volatile uint8_t NAME_EXT[8U];
//	const volatile uint8_t MANUFACTURE_DAY;
//	const volatile uint8_t MANUFACTURE_MONTH;
//	const volatile uint8_t MANUFACTURE_YEAR;
//	const volatile uint8_t MANUFACTURE_RESV;
//	const volatile uint32_t SERIAL_NUMBER;
//}
//MotMemMap_Once_T;//, * MotFlash_PtrOnce_T;
//
//#define MOT_MEM_ONCE	((MotMemMap_Once_T *)CONFIG_MOT_MEM_ONCE_ADDRESS)
//
//typedef	const volatile struct __attribute__ ((aligned (4U)))
//{
//	const volatile uint16_t CONTROLLER_VOLTAGE;
//}
//MotMemMap_OnceExt_T;
//
//#ifdef CONFIG_MOT_MEM_ONCE_EXT_ADDRESS
//	#define MOT_MEM_ONCE_EXT	((MotMemMap_OnceExt_T *)CONFIG_MOT_MEM_ONCE_EXT_ADDRESS)
//#else
//	#define MOT_MEM_ONCE_EXT	((MotMemMap_OnceExt_T *)(CONFIG_MOT_MEM_ONCE_ADDRESS + sizeof(MotMemMap_Once_T)))
//#endif
//
////User Read Only, flash
//typedef const volatile struct __attribute__ ((aligned (8U)))
//{
//	const volatile uint8_t SOFTWARE_VERSION[4];
//	const volatile uint16_t SENSOR_RATED_I;
//	const volatile uint16_t SENSOR_RATED_AD;
//}
//MotMemMap_ReadOnly_T;
//
//#define MOT_MEM_READ_ONLY	((MotMemMap_ReadOnly_T *)CONFIG_MOT_MEM_READ_ONLY_ADDRESS)
//
//
////User nonvolatile Read Write, eeprom if available
//typedef const volatile struct __attribute__ ((aligned (4U)))
//{
//	const volatile Motor_Parameters_T MOTORS[1U]; //#define CONFIG_MOT_FLASH_MOTOR_COUNT 1U
//
//	const volatile MotAnalogMonitor_Params_T ANALOG_MONITOR;
////	const volatile MotorController_Parameters_T MC;
//
////	MotUserOptions_T USER_OPTIONS;
//
//	const volatile uint8_t APPLICATION_CONFIG;
//	const volatile uint8_t STARTUP_WAIT_TIME;
////	const volatile uint16_t CONTROLLER_VOLTAGE;
//}
//MotMemMap_ReadWrite_T;
//
//#define MOT_MEM_READ_WRITE	((MotMemMap_ReadWrite_T *)CONFIG_MOT_MEM_READ_WRITE_ADDRESS)

#endif
