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
	@file 	MotProtocol.c
	@author FireSoucery
	@brief	Implement MotProtocol
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_CMDR_H
#define MOTOR_CMDR_H

#include "Utility/Protocol/Concrete/MotProtocol/MotProtocol.h" /* Include for interface */
#include "Utility/Protocol/Protocol_Cmdr.h" /* Include for Procotol handling state */
#include "Utility/Timer/Timer.h"
#include <stdint.h>


/*
	Local unit converisons
*/
typedef struct
{
	uint16_t SpeedFeedbackRef_Rpm;
	uint16_t IMaxRef_Amp;
}
MotorCmdr_Units_T;

typedef struct
{
    // //condifg
 	// Protocol_T * const P_PROTOCOLS;
	// const uint8_t PROTOCOL_COUNT;
    // Protocol_T * p_ProtocolActive;

	Protocol_T Protocol;
	// Timer_T TimerMillis;

	MotorCmdr_Units_T Units;
	// uint8_t StateId;

	/* inheritance version */
	MotProtocol_Interface_T Interface;
}
MotorCmdr_T;

//protocol
#define CONFIG_MOTOR_CMDR()

// static inline void MotorCmdr_Proc_Thread(MotorCmdr_T * p_motorCmdr)
// {
// 	// if(Timer_Poll(&p_motorCmdr->TimerMillis) == true)
// 	{
// 		Protocol_Cmdr_ProcRx(&p_motorCmdr->Protocol);
// 	}
// }

// static inline bool MotorCmdr_PollResponse(MotorCmdr_T * p_motorCmdr)
// {
// 	return Protocol_Cmdr_CheckAvailable(&p_motorCmdr->Protocol);
// }

/*
	Outside module handle TxRx, check availabe
*/
static inline bool _MotorCmdr_ParseResp(MotorCmdr_T * p_motorCmdr) { return _Protocol_Cmdr_ParseResp(&p_motorCmdr->Protocol); }
// static inline uint8_t * _MotorCmdr_GetPtrTxPacket(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetPtrTxPacket(&p_motorCmdr->Protocol); }
static inline uint8_t _MotorCmdr_GetReqLength(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetReqLength(&p_motorCmdr->Protocol); }
static inline uint8_t _MotorCmdr_GetRespLength(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetRespRemaining(&p_motorCmdr->Protocol); }




static inline int32_t MotorCmdr_GetReadSpeed(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Interface.Speed.Speed; }
static inline  int16_t MotorCmdr_GetReadSpeed_Rpm(MotorCmdr_T * p_motorCmdr) { return ConvertToSpeedRpm(p_motorCmdr, p_motorCmdr->Interface.Speed.Speed); }



extern void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle);

extern uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle);
extern uint8_t _MotorCmdr_WriteControlType(MotorCmdr_T * p_motorCmdr);
extern uint8_t _MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr);
extern void _MotorCmdr_WriteVar16(MotorCmdr_T * p_motorCmdr, uint16_t var16);

#endif



// typedef enum
// {
// 	MOTOR_CMDR_READ_SPEED,
// 	MOTOR_CMDR_WRITE_THROTTLE,
//     MOTOR_CMDR_WRITE_BRAKE,

// 	// MOTPROTOCOL_MONITOR_I_PHASES = 0x02U,	/* Iabc in Q1.15 */
// 	// MOTPROTOCOL_MONITOR_I_FOC = 0x03U,		/* Idq, Ialphabeta in Q1.15 */
// 	// MOTPROTOCOL_MONITOR_V_PHASES = 0x04U,
// 	// MOTPROTOCOL_MONITOR_ANGLES = 0x05U,
// 	// MOTPROTOCOL_MONITOR_STATUS_FLAGS = 0x0FU,

// 	// MOTPROTOCOL_MONITOR_ANALOG_USER = 0x10U,
// 	// MOTPROTOCOL_MONITOR_HALL = 0x11U,
// 	// MOTPROTOCOL_MONITOR_SIN_COS = 0x12U,
// 	// MOTPROTOCOL_MONITOR_ENCODER = 0x13U,

// 	// MOTPROTOCOL_MONITOR_V_SENSORS = 0x21U, 		VSupply, ~5V, ~12V
// 	// MOTPROTOCOL_MONITOR_HEAT = 0x22U,			/* In ADCU, Lower is higher */

// 	// MOTPROTOCOL_MONITOR_METERS = 0x31U, /* Speed, Angles */

// 	// MOTPROTOCOL_MONITOR_SPEED_RPM = 0x81U,	/* Speed in RPM */

// 	// MOTPROTOCOL_MONITOR_ADC_BATCH_MSB = 0xE1U,

// 	// // MotProtocol_MONITOR_ADC_MOTOR_COMMON,
// 	// // MotProtocol_MONITOR_ADC_MOTOR_0,
// 	// // MotProtocol_MONITOR_ADC_MOTOR_1,
// 	// // MotProtocol_MONITOR_V_SENSORS_MV = 0x00U,
// 	// // MotProtocol_MONITOR_HEAT_DEG_C = 0x00U,

// 	// MOTPROTOCOL_MONITOR_RESERVED = 0xFFU,

//     // MOTPROTOCOL_CONTROL_STOP = 0x00U,
// 	// MOTPROTOCOL_CONTROL_THROTTLE = 0x01U,	/* 0:65536 */
// 	// MOTPROTOCOL_CONTROL_BRAKE = 0x02U,		/* 0:65536 */

// 	// MOTPROTOCOL_CONTROL_VOLTAGE = 0x03U, 	/* +/-32768 */
// 	// MOTPROTOCOL_CONTROL_TORQUE = 0x04U,	/* +/-32768 */
// 	// MOTPROTOCOL_CONTROL_SPEED = 0x05U,		/* +/-32768 */
// 	// MOTPROTOCOL_CONTROL_ANGLE = 0x06U,		/*   */

// 	// MOTPROTOCOL_CONTROL_DIRECTION = 0x10U,

// 	// // MOTPROTOCOL_CONTROL_VOLTAGE_VPWM = 0x83U, /* DutyCycle 65536 */
// 	// // MOTPROTOCOL_CONTROL_VOLTAGE_VOLTS = 0x83U,
// 	// // MOTPROTOCOL_CONTROL_TORQUE_AMPS = 0x84U,
// 	// MOTPROTOCOL_CONTROL_SPEED_RPM = 0x85U,
// 	// // MOTPROTOCOL_CONTROL_ANGLE = 0x85U,

// 	// MOTPROTOCOL_CONTROL_BATCH_1 = 0x71U,
// 	// MOTPROTOCOL_CONTROL_MODE_RESERVED = 0xFFU,
// }
// MotorCmdr_Id_T;



	// MotorCmdr_Id_T ReqActive;
	// int32_t Speed;
	// uint16_t Brake;
	// uint16_t Throttle;

	// union
	// {
	// 	uint16_t RegisterU16s[16U];
	// 	uint32_t RegisterU32s[8U];
	// 	uint8_t RegisterU8s[32U];
	// 	int16_t RegisterS16s[16U];
	// 	int32_t RegisterS32s[8U];
	// 	int8_t RegisterS8s[32U];
	// } ;
