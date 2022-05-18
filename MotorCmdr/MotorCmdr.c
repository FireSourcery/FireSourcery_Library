
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
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorCmdr.h"
#include "Utility/Protocol/Concrete/MotProtocol/MotProtocol.h"
#include "Utility/Protocol/Protocol_Cmdr.h"

//move to shared?
static inline int32_t ConvertToSpeedFrac16(MotorCmdr_T * p_motorCmdr, int32_t speed_rpm) { return speed_rpm * 65535 / p_motorCmdr->Units.SpeedFeedbackRef_Rpm; }
static inline int16_t ConvertToSpeedRpm(MotorCmdr_T * p_motorCmdr, int32_t speed_frac16) { return speed_frac16 * p_motorCmdr->Units.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t ConvertToIFrac16(MotorCmdr_T * p_motorCmdr, int32_t i_amp) { return i_amp * 65535 / p_motorCmdr->Units.IMaxRef_Amp; }
static inline int16_t ConvertToIAmp(MotorCmdr_T * p_motorCmdr, int32_t i_frac16) { return i_frac16 * p_motorCmdr->Units.IMaxRef_Amp / 65536; }

void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr)
{
	// Timer_InitPeriodic(&p_motorCmdr->TimerMillis, 1U);
	Protocol_Init(&p_motorCmdr->Protocol);
}

void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr)
{
	// p_motorCmdr->Interface.ReqReadImmediate.VarId = MOT_VAR_SPEED_FEEDBACK_REF_RPM;
	// _Protocol_Cmdr_EnqueueReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_READ_IMMEDIATE);

	// p_motorCmdr->Interface.ReqReadImmediate.VarId = MOT_VAR_I_MAX_REF_AMP;
	// _Protocol_Cmdr_EnqueueReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_READ_IMMEDIATE);
}


void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
{
	/*
		Composition version, Upper layer access using uniform Protocol Module functions,
		dependency on Protocol Module for Cmdr_Proc, handle req/resp wait, and Xvcr TxRx
	*/
	/* Use own structure */
	// p_motorCmdr->Throttle = throttle;
	// p_motorCmdr->ReqActive = MOTOR_CMDR_WRITE_THROTTLE;
	// Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTOR_CMDR_WRITE_THROTTLE);

	/* Use Interface */
	p_motorCmdr->Interface.ReqThrottle.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	p_motorCmdr->Interface.ReqThrottle.ThrottleValue = throttle;
	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_CONTROL_TYPE);

	/*
		Inheritance version, directly call MotProtocol,
		Fixed to MotProtocol packets. Optionally use Protocol Module to handle req/responly, optionally Tx Rx.
	*/
	// p_motorCmdr->Interface.ReqThrottle.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	// p_motorCmdr->Interface.ReqThrottle.ThrottleValue = throttle;
	// MotProtocol_ReqPacket_Control_Build(p_motorCmdr->Protocol.CONFIG.P_TX_PACKET_BUFFER, &p_motorCmdr->Interface);
	// p_motorCmdr->RxRemaining = MotProtocol_GetControlRespLength(MOTPROTOCOL_CONTROL_THROTTLE);
	// alternatively use local buffer
	// Mot_Protocol_Cmdr_BuildReq_Control(p_motorCmdr->Protocol.CONFIG.P_TX_PACKET_BUFFER, &p_motorCmdr->Protocol.TxLength, &p_motorCmdr->Protocol.RxRemaining, p_motorCmdr->Interface);

}


/*!
	Proctected _MotorCmdr_Write functions - build packet, set wait state, without starting transmission
	@return length of full packet
*/
uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
{
	p_motorCmdr->Interface.ReqThrottle.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	p_motorCmdr->Interface.ReqThrottle.ThrottleValue = throttle;
	_Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_CONTROL_TYPE);
	return p_motorCmdr->Protocol.TxLength;
}

/* User sets Interface as MotProtocol_ReqPayload_Control_T */
uint8_t _MotorCmdr_WriteControlType(MotorCmdr_T * p_motorCmdr)
{
	_Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_CONTROL_TYPE);
	return p_motorCmdr->Protocol.TxLength;
}

uint8_t _MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->Interface.ReqMonitor.MonitorId = MOTPROTOCOL_MONITOR_SPEED;
	_Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_MONITOR_TYPE);
	return p_motorCmdr->Protocol.TxLength;
}

void _MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, uint32_t var)
{

}
