
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
*/

extern const Protocol_Specs_T MOTOR_CMDR_MOT_PROTOCOL_SPECS;  //todo fix circular extern

/* MotorCmdr supports Mot_Protocol only */
const Protocol_Specs_T * const _MOTOR_CMDR_PROTOCOL_SPECS_TABLE[1U] =
{
	[0U] = &MOTOR_CMDR_MOT_PROTOCOL_SPECS,
};

void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr)
{
	Protocol_Init(&p_motorCmdr->Protocol);
	p_motorCmdr->Protocol.Params.RxLostTime = 500U;
	// p_motorCmdr->ReqActive = 0xFF;
}

//todo stateful req
void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr) { _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_INIT_UNITS); }

/******************************************************************************/
/*!
	Proctected Function - build packet, set wait state, without starting Tx
	@return length of full packet
*/
/******************************************************************************/
void _MotorCmdr_ProcTxIdle(MotorCmdr_T * p_motorCmdr)
{
	if(Protocol_Cmdr_CheckTxIdle(&p_motorCmdr->Protocol) == true) { _MotorCmdr_Ping(p_motorCmdr); }
}

uint8_t _MotorCmdr_Ping(MotorCmdr_T * p_motorCmdr) 			{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_PING); }
uint8_t _MotorCmdr_StopMotors(MotorCmdr_T * p_motorCmdr) 	{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_STOP_ALL); }
uint8_t _MotorCmdr_SaveNvm(MotorCmdr_T * p_motorCmdr) 		{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_SAVE_NVM); }
uint8_t _MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr) 	{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_INIT_UNITS); }

uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
{
	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_THROTTLE;
	p_motorCmdr->MotorCmdValue = throttle;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
}

uint8_t _MotorCmdr_WriteBrake(MotorCmdr_T * p_motorCmdr, uint16_t brake)
{
	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_BRAKE;
	p_motorCmdr->MotorCmdValue = brake;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
}

uint8_t _MotorCmdr_WriteRelease(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_RELEASE;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
}

uint8_t _MotorCmdr_WriteDirectionForward(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_DIRECTION_FORWARD;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
}

uint8_t _MotorCmdr_WriteDirectionReverse(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_DIRECTION_REVERSE;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
}

uint8_t _MotorCmdr_WriteDirectionNeutral(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_DIRECTION_NEUTRAL;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
}

// uint8_t _MotorCmdr_WriteStopMotorId(MotorCmdr_T * p_motorCmdr, uint8_t motorId)
// {
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol,  );
//
// }

/* User sets Interface as MOT_PROTOCOL_ReqPayload_Control_T */
// uint8_t _MotorCmdr_WriteControlType(MotorCmdr_T * p_motorCmdr, 15 regs)
// {
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
//
// }

uint8_t _MotorCmdr_StartReadSpeed(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->MonitorIdActive = MOT_PROTOCOL_MONITOR_SPEED;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_MONITOR_TYPE);
}

uint8_t _MotorCmdr_StartReadIFoc(MotorCmdr_T * p_motorCmdr)
{
	p_motorCmdr->MonitorIdActive = MOT_PROTOCOL_MONITOR_I_FOC;
	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_MONITOR_TYPE);
}

// void _MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, uint32_t var)
// {

// }


/******************************************************************************/
/*
	Public Functions - Protocol module handle TxRX
*/
/******************************************************************************/
// void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PROTOCOL_CONTROL_THROTTLE;
// 	p_motorCmdr->MotorCmdValue = throttle;
// 	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PROTOCOL_CMD_CONTROL_TYPE);
// }