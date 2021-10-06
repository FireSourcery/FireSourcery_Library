#ifndef MOT_USER_PARAMS_H
#define MOT_USER_PARAMS_H

#include <stdint.h>

typedef const volatile struct __attribute__ ((aligned (4U)))
{
	const uint8_t SHELL_CONNECT_ID;

//	uint8_t ProtocolDataLinkId[1]; //per protocol
//	const uint8_t AUX_PROTOCOL_SPECS_ID[CONFIG_MOTOR_CONTROLLER_AUX_PROTOCOL_COUNT];
	const uint8_t MOT_PROTOCOL_SPECS_ID;

	const bool ANALOG_USER_ENABLE;
}
MotUserOptions_T;

#endif
