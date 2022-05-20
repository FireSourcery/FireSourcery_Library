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
	@brief	Implements MotProtocol
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_CMDR_H
#define MOTOR_CMDR_H

#include "Utility/Protocol/Concrete/MotProtocol/Mot_Protocol.h"
#include "Utility/Protocol/Concrete/MotProtocol/MotProtocol.h" /* Include for interface */
#include "Utility/Protocol/Protocol_Cmdr.h" /* Include for Procotol handling state */
#include <stdint.h>

/*
	Local unit converisons
*/
typedef struct MotorCmdr_Units_Tag
{
	uint16_t SpeedFeedbackRef_Rpm;
	uint16_t IMaxRef_Amp;
}
MotorCmdr_Units_T;

typedef struct MotorCmdr_Tag
{
	Protocol_T Protocol;
	MotorCmdr_Units_T Units;
	// uint8_t StateId; //or enqueue

	/* App implements MotProtocol interface */
	MotProtocol_Interface_T Interface;


	// Alternatively, app provides interface
	// static void BuildReq_Control(MotProtocol_ReqPacket_Control_T * p_reqPacket, size_t * p_txLength, size_t * p_respLength, const MotorCmdr_T * p_app)
	// {
	// 	/* App uses its own structure */
	// 	switch(p_app->ReqActive)
	// 	{
	// 		case MOTOR_CMDR_WRITE_THROTTLE:
	// 			*p_txLength = MotProtocol_ReqPacket_Control_BuildThrottle(p_reqPacket, p_app->Throttle);
	// 			*p_respLength = MotProtocol_GetControlRespLength(MOTPROTOCOL_CONTROL_THROTTLE);
	// 			break;
	// 	}
	// }

}
MotorCmdr_T;

extern const Protocol_Specs_T * const _MOTOR_CMDR_PROTOCOL_SPECS_TABLE[1U];

#define MOTOR_CMDR_DEFINE(p_ThisMotorCmdr, p_RxBuffer, p_TxBuffer, p_XcvrTable, TableLength, p_Timer)  	\
{																										\
	.Protocol = PROTOCOL_DEFINE 																		\
	(																									\
		p_RxBuffer, p_TxBuffer, MOTPROTOCOL_PACKET_LENGTH_MAX, 											\
		&((p_ThisMotorCmdr)->Interface), 0U, 															\
		_MOTOR_CMDR_PROTOCOL_SPECS_TABLE, 1U, 															\
		p_XcvrTable, TableLength, 																		\
		p_Timer, 0U																						\
	),																									\
}

/*
	Outside module handle TxRx, CheckAvailable
*/
static inline bool _MotorCmdr_CaptureResp(MotorCmdr_T * p_motorCmdr) { return _Protocol_Cmdr_ParseResp(&p_motorCmdr->Protocol); }
static inline uint8_t _MotorCmdr_GetReqLength(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetReqLength(&p_motorCmdr->Protocol); }
static inline uint8_t _MotorCmdr_GetRespLength(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetRespRemaining(&p_motorCmdr->Protocol); }

// static inline void MotorCmdr_Proc_Thread(MotorCmdr_T * p_motorCmdr)
// {
// 	// if(Timer_Poll(&p_motorCmdr->TimerMillis) == true)
// 	{
// 		Protocol_Cmdr_ProcRx(&p_motorCmdr->Protocol);
// 	}
// }

static inline int32_t MotorCmdr_GetReadSpeed(MotorCmdr_T *p_motorCmdr) { return p_motorCmdr->Interface.Speed.Speed; }

/*
	For Cmdr side handling of unit conversion
*/
static inline int32_t _MotorCmdr_ConvertToSpeedFrac16(MotorCmdr_T * p_motorCmdr, int32_t speed_rpm) { return speed_rpm * 65535 / p_motorCmdr->Units.SpeedFeedbackRef_Rpm; }
static inline int16_t _MotorCmdr_ConvertToSpeedRpm(MotorCmdr_T * p_motorCmdr, int32_t speed_frac16) { return speed_frac16 * p_motorCmdr->Units.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t _MotorCmdr_ConvertToIFrac16(MotorCmdr_T * p_motorCmdr, int32_t i_amp) { return i_amp * 65535 / p_motorCmdr->Units.IMaxRef_Amp; }
static inline int16_t _MotorCmdr_ConvertToIAmp(MotorCmdr_T * p_motorCmdr, int32_t i_frac16) { return i_frac16 * p_motorCmdr->Units.IMaxRef_Amp / 65536; }

static inline int16_t MotorCmdr_GetReadSpeed_Rpm(MotorCmdr_T *p_motorCmdr) { return _MotorCmdr_ConvertToSpeedRpm(p_motorCmdr, p_motorCmdr->Interface.Speed.Speed); }


/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle);

extern uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle);
extern uint8_t _MotorCmdr_WriteControlType(MotorCmdr_T * p_motorCmdr);
extern uint8_t _MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr);
extern void _MotorCmdr_WriteVar16(MotorCmdr_T * p_motorCmdr, uint16_t var16);

#endif
