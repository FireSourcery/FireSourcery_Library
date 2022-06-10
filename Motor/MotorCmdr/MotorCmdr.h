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

#include "Motor/MotProtocol/MotProtocol.h"
#include "Utility/Protocol/Protocol_Cmdr.h" /* Include for Procotol handling state */
#include <stdint.h>

/*
	Local unit converisons
*/
typedef struct MotorCmdr_Units_Tag
{
	uint16_t SpeedFeedbackRef_Rpm;
	uint16_t IMaxRef_Amp;
	uint16_t VSupplyRef_Volts;
	uint16_t VSupply_R1;
	uint16_t VSupply_R2;
	uint16_t VSense_R1;
	uint16_t VSense_R2;
	uint16_t VAcc_R1;
	uint16_t VAcc_R2;
}
MotorCmdr_Units_T;

typedef struct MotorCmdr_Tag
{
	uint8_t TxPacket[MOT_PACKET_LENGTH_MAX];
	uint8_t RxPacket[MOT_PACKET_LENGTH_MAX];
	Protocol_T Protocol;
	MotProtocol_Substate_T Substate; /* Stateful Protocol Substate */

	/* Req Interface */
	uint16_t RespStatus;
	MotPacket_ControlId_T ControlIdActive;
	MotPacket_MonitorId_T MonitorIdActive;
	uint16_t MotorCmdValue;			/*  */

	uint16_t MotorWriteVarId;		/*  */
	uint32_t MotorWriteVarValue;	/*  */
	uint16_t MotorReadVarId;		/*  */
	uint32_t MotorReadVarValue;		/*  */

	/* Resp Interface */
	MotorCmdr_Units_T Units;
	uint8_t Version[4U];
	int32_t Speed;
	int16_t Ia;
	int16_t Ib;
	int16_t Ic;
	int16_t Ialpha;
	int16_t Ibeta;
	int16_t Id;
	int16_t Iq;

	// MotorController_T MotorControllerVars;					/* Use for definations only. */
	// Motor_T MotorsVars[CONFIG_MOTOR_CMDR_MOTOR_COUNT];		/* Use for definations only. */

	/* Alternatively, app implements MotProtocol Interface */
	// MotPacket_Interface_T Interface;
}
MotorCmdr_T;

extern const Protocol_Specs_T * const _MOTOR_CMDR_PROTOCOL_SPECS_TABLE[1U];

/*
	No Non-Volatile memory params by default
*/
#define MOTOR_CMDR_DEFINE(p_ThisMotorCmdr, p_XcvrTable, XcvrCount, p_Timer)  							\
{																										\
	.Protocol = PROTOCOL_DEFINE 																		\
	(																									\
		&((p_ThisMotorCmdr)->RxPacket[0U]), &((p_ThisMotorCmdr)->TxPacket[0U]), MOT_PACKET_LENGTH_MAX, 	\
		(p_ThisMotorCmdr), &((p_ThisMotorCmdr)->Substate), 												\
		_MOTOR_CMDR_PROTOCOL_SPECS_TABLE, 1U, 															\
		p_XcvrTable, XcvrCount, 																		\
		p_Timer, 0U																						\
	),																									\
}

/*
	For Cmdr side handling of unit conversion
*/
static inline int32_t _MotorCmdr_ConvertToSpeedFrac16(MotorCmdr_T * p_motorCmdr, int32_t speed_rpm) { return speed_rpm * 65535 / p_motorCmdr->Units.SpeedFeedbackRef_Rpm; }
static inline int16_t _MotorCmdr_ConvertToSpeedRpm(MotorCmdr_T * p_motorCmdr, int32_t speed_frac16) { return speed_frac16 * p_motorCmdr->Units.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t _MotorCmdr_ConvertToIFrac16(MotorCmdr_T * p_motorCmdr, int32_t i_amp) { return i_amp * 65535 / p_motorCmdr->Units.IMaxRef_Amp; }
static inline int16_t _MotorCmdr_ConvertToIAmp(MotorCmdr_T * p_motorCmdr, int32_t i_frac16) { return i_frac16 * p_motorCmdr->Units.IMaxRef_Amp / 65536; }

static inline int16_t MotorCmdr_GetReadSpeed_Rpm(MotorCmdr_T * p_motorCmdr) { return _MotorCmdr_ConvertToSpeedRpm(p_motorCmdr, p_motorCmdr->Speed); }

static inline int32_t MotorCmdr_GetReadSpeed(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Speed; }
static inline int16_t MotorCmdr_GetReadIa(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Ia; }
static inline int16_t MotorCmdr_GetReadIb(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Ib; }
static inline int16_t MotorCmdr_GetReadIc(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Ic; }
static inline int16_t MotorCmdr_GetReadIalpha(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Ialpha; }
static inline int16_t MotorCmdr_GetReadIbeta(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Ibeta; }
static inline int16_t MotorCmdr_GetReadId(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Id; }
static inline int16_t MotorCmdr_GetReadIq(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->Iq; }

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void MotorCmdr_Proc(MotorCmdr_T * p_motorCmdr);

extern void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr);

extern void MotorCmdr_Ping(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_StopMotors(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_SaveNvm(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId, uint32_t value);
extern void MotorCmdr_StartReadVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId);
extern void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle);
extern void MotorCmdr_WriteBrake(MotorCmdr_T * p_motorCmdr, uint16_t brake);
extern void MotorCmdr_WriteRelease(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_WriteDirectionForward(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_WriteDirectionReverse(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_WriteDirectionNeutral(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr);
extern void MotorCmdr_StartReadIFoc(MotorCmdr_T * p_motorCmdr);


/*
	Outside module handle TxRx, CheckAvailable
*/
// static inline bool _MotorCmdr_ParseResp(MotorCmdr_T * p_motorCmdr) { return _Protocol_Cmdr_ParseResp(&p_motorCmdr->Protocol); }
// static inline bool _MotorCmdr_PollTimeout(MotorCmdr_T * p_motorCmdr) { return _Protocol_Cmdr_PollTimeout(&p_motorCmdr->Protocol); }

// static inline uint8_t _MotorCmdr_GetReqLength(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetReqLength(&p_motorCmdr->Protocol); }
// static inline uint8_t _MotorCmdr_GetRespLength(MotorCmdr_T * p_motorCmdr) { return Protocol_Cmdr_GetRespRemaining(&p_motorCmdr->Protocol); }
// static inline uint8_t * _MotorCmdr_GetPtrTxPacket(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->TxPacket; }
// static inline uint8_t * _MotorCmdr_GetPtrRxPacket(MotorCmdr_T * p_motorCmdr) { return p_motorCmdr->RxPacket; }

// extern void _MotorCmdr_ProcTxIdle(MotorCmdr_T * p_motorCmdr);

// extern uint8_t _MotorCmdr_Ping(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_StopMotors(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_SaveNvm(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId, uint32_t value);
// extern uint8_t _MotorCmdr_StartReadVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId);
// extern uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle);
// extern uint8_t _MotorCmdr_WriteBrake(MotorCmdr_T * p_motorCmdr, uint16_t brake);
// extern uint8_t _MotorCmdr_WriteRelease(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_WriteDirectionForward(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_WriteDirectionReverse(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_WriteDirectionNeutral(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr);
// extern uint8_t _MotorCmdr_StartReadIFoc(MotorCmdr_T * p_motorCmdr);

#endif
