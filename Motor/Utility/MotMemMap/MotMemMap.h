#ifndef MOT_MEM_MAP_H
#define MOT_MEM_MAP_H

#include "Motor/Motor/Motor.h"
#include "Motor/Utility/MotAnalogMonitor/MotAnalogMonitor.h"
//#include "Motor/Utility/MotUserOptions/MotParameters.h"

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
MotMemMap_Once_T;//, * MotFlash_PtrOnce_T;

#define MOT_MEM_ONCE	((MotMemMap_Once_T *)CONFIG_MOT_MEM_ONCE_ADDRESS)

typedef	const struct __attribute__ ((aligned (4U)))
{

}
MotMemMap_OnceExt_T;

#ifdef CONFIG_MOT_MEM_ONCE_EXT_ADDRESS
	#define MOT_MEM_ONCE_EXT	((MotMemMap_OnceExt_T *)CONFIG_MOT_MEM_ONCE_EXT_ADDRESS)
#else
	#define MOT_MEM_ONCE_EXT	((MotMemMap_OnceExt_T *)(CONFIG_MOT_MEM_ONCE_ADDRESS + sizeof(MotMemMap_Once_T)))
#endif

//User Read Only, nonconfigurable flash
typedef const struct __attribute__ ((aligned (8U)))
{
	const uint8_t SOFTWARE_VERSION[4];
	const uint16_t SENSOR_RATED_I;
	const uint16_t SENSOR_RATED_AD;
}
MotMemMap_ReadOnly_T;

#define MOT_MEM_READ_ONLY	((MotMemMap_ReadOnly_T *)CONFIG_MOT_MEM_READ_ONLY_ADDRESS)


//User Configurable, eeprom if available
typedef const struct __attribute__ ((aligned (4U)))
{
	const Motor_Parameters_T MOTORS[1U]; //#define CONFIG_MOT_FLASH_MOTOR_COUNT 1U

	const MotAnalogMonitor_Params_T ANALOG_MONITOR;
//	const MotAppParameters_T MOT_APP;

//	const uint8_t APPLICATION_CONFIG;
//	const uint8_t STARTUP_WAIT_TIME;
	//	uint8_t ShellConnectId;
	//	uint8_t ProtocolDataLinkId[1]; //per protocol
	//	uint8_t AuxProtocolSpecsId[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT];
	//	uint8_t MotProtocolSpecsId;
//		bool IsAnalogUserEnable;
//		bool IsBuzzerOnReverseEnable;
}
MotMemMap_ReadWrite_T;

#define MOT_MEM_READ_WRITE	((MotMemMap_ReadWrite_T *)CONFIG_MOT_MEM_READ_WRITE_ADDRESS)


#endif
