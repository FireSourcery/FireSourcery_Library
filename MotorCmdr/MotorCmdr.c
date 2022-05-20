
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

/*
	Composition - access uniform Protocol Module functions,
	Implmented dependency on Protocol module for StartReq, ReqTable,
	Optionally handle TxRx, req/resp wait
	Implements MotProtocol Interface
*/

/* MotorCmdr supports Mot_Protocol only */
const Protocol_Specs_T * const _MOTOR_CMDR_PROTOCOL_SPECS_TABLE[1U] =
{
	[0U] = &MOT_PROTOCOL_CMDR_SPECS,
};


void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr)
{
	Protocol_Init(&p_motorCmdr->Protocol);
}

void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr)
{
	// p_motorCmdr->Interface.ReqReadImmediate.VarId = MOT_VAR_SPEED_FEEDBACK_REF_RPM;
	// _Protocol_Cmdr_EnqueueReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_READ_IMMEDIATE);

	// p_motorCmdr->Interface.ReqReadImmediate.VarId = MOT_VAR_I_MAX_REF_AMP;
	// _Protocol_Cmdr_EnqueueReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_READ_IMMEDIATE);
}

void MotorCmdr_InitUnits_Blocking(MotorCmdr_T * p_motorCmdr)
{
	// p_motorCmdr->Interface.ReqReadImmediate.VarId = MOT_VAR_SPEED_FEEDBACK_REF_RPM;
	// _Protocol_Cmdr_EnqueueReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_READ_IMMEDIATE);

	// p_motorCmdr->Interface.ReqReadImmediate.VarId = MOT_VAR_I_MAX_REF_AMP;
	// _Protocol_Cmdr_EnqueueReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_READ_IMMEDIATE);
}

/*!
	Proctected Function - build packet, set wait state, without starting Tx
	@return length of full packet
*/
uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
{
	p_motorCmdr->Interface.ReqThrottle.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	p_motorCmdr->Interface.ReqThrottle.ThrottleValue = throttle;
	_Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_CONTROL_TYPE);
	return Protocol_Cmdr_GetReqLength(&p_motorCmdr->Protocol);
}

/* User sets Interface as MotProtocol_ReqPayload_Control_T */
uint8_t _MotorCmdr_WriteControlType(MotorCmdr_T * p_motorCmdr)
{
	_Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_CONTROL_TYPE);
	return Protocol_Cmdr_GetReqLength(&p_motorCmdr->Protocol);
}

uint8_t _MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->Interface.ReqMonitor.MonitorId = MOTPROTOCOL_MONITOR_SPEED;
	_Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_MONITOR_TYPE);
	return Protocol_Cmdr_GetReqLength(&p_motorCmdr->Protocol);
}

void _MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, uint32_t var)
{

}


/* Public Functions - Protocol module handle TxRX */
void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
{
	/* Use Interface */
	p_motorCmdr->Interface.ReqThrottle.ControlId = MOTPROTOCOL_CONTROL_THROTTLE;
	p_motorCmdr->Interface.ReqThrottle.ThrottleValue = throttle;
	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOTPROTOCOL_CMD_CONTROL_TYPE);
}