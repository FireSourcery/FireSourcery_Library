
/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "MotorCmdr.h"

/*
	Composition - access uniform Protocol Module functions,
	Implemented dependency on Protocol module for StartReq, ReqTable,
	Optionally handle TxRx, req/resp wait
*/

extern const Protocol_Specs_T MOTOR_CMDR_MOT_PROTOCOL_SPECS;  /* circular extern defined in MotorCmdr_MotProtocol */

/* MotorCmdr supports MotorCmdr_MotProtocol only */
const Protocol_Specs_T * const _MOTOR_CMDR_PROTOCOL_SPECS_TABLE[1U] =
{
	[0U] = &MOTOR_CMDR_MOT_PROTOCOL_SPECS,
};

void MotorCmdr_Init(MotorCmdr_T * p_motorCmdr)
{
	Protocol_Init(&p_motorCmdr->Protocol);
	Protocol_Enable(&p_motorCmdr->Protocol);
	p_motorCmdr->Protocol.Params.WatchdogTime = 500U;
	// MotorCmdr_InitUnits(p_motorCmdr);
}

/******************************************************************************/
/*
	Public Functions - Protocol module handle TxRX

	Upper layer implements:
		Xcvr_Interface_RxMax_T 		RX_MAX;
		Xcvr_Interface_TxN_T 		TX_N;
*/
/******************************************************************************/
/*
	Protocol module handle TxRx via Xcvr

	Check Available Rx Req, Tx Resp
*/
void MotorCmdr_Proc(MotorCmdr_T * p_motorCmdr)
{
	Protocol_Proc(&p_motorCmdr->Protocol);
	if(Protocol_Cmdr_CheckTxIdle(&p_motorCmdr->Protocol) == true) { MotorCmdr_Ping(p_motorCmdr); }
}


//todo stateful req
void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr) { Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_INIT_UNITS); }

/*
	overwrite even if sync is active expected response invalidate active req repse
*/
void MotorCmdr_StopMotors(MotorCmdr_T * p_motorCmdr) 	{ Protocol_Cmdr_StartReq_Overwrite(&p_motorCmdr->Protocol, MOT_PACKET_STOP_ALL); }
void MotorCmdr_Ping(MotorCmdr_T * p_motorCmdr) 			{ Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_PING); }
void MotorCmdr_SaveNvm(MotorCmdr_T * p_motorCmdr) 		{ Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_SAVE_NVM); }

void MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId, uint32_t value)
{
	p_motorCmdr->MotorWriteVarId = motVarId;
	p_motorCmdr->MotorWriteVarValue = value;
	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_WRITE_VAR);
}

void MotorCmdr_ReadVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId)
{
	p_motorCmdr->MotorReadVarId = motVarId;
	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_READ_VAR);
}

/* Read speed back including:  */
void MotorCmdr_ReadSpeed(MotorCmdr_T * p_motorCmdr)
{
	// p_motorCmdr->MonitorIdActive = MOT_PACKET_MONITOR_SPEED;
	// Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_MONITOR_TYPE);
	MotorCmdr_ReadVar(p_motorCmdr, MOT_VAR_SPEED_RPM);
}

// void MotorCmdr_ReadIFoc(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->MonitorIdActive = MOT_PACKET_MONITOR_I_FOC;
// 	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_MONITOR_TYPE);
// }

/* Overwrites */
void MotorCmdr_WriteBrake(MotorCmdr_T * p_motorCmdr, uint16_t brake)
{
	// p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_BRAKE;
	// p_motorCmdr->MotorCmdValue = brake;
	// Protocol_Cmdr_StartReq_Overwrite(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
	p_motorCmdr->MotorWriteVarId = MOT_VAR_BRAKE;
	p_motorCmdr->MotorWriteVarValue = brake;
	Protocol_Cmdr_StartReq_Overwrite(&p_motorCmdr->Protocol, MOT_PACKET_WRITE_VAR);
}

void MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
{
	// p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_THROTTLE;
	// p_motorCmdr->MotorCmdValue = throttle;
	// Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);

	MotorCmdr_WriteVar(p_motorCmdr, MOT_VAR_THROTTLE, throttle);
}

// void MotorCmdr_WriteRelease(MotorCmdr_T * p_motorCmdr)
// {
// 	// p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_RELEASE;
// 	// Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// 	MotorCmdr_WriteVar(p_motorCmdr, MOT_VAR_THROTTLE, 0U);
// }

// void MotorCmdr_WriteDirectionForward(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_DIRECTION_FORWARD;
// 	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// void MotorCmdr_WriteDirectionReverse(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_DIRECTION_REVERSE;
// 	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// void MotorCmdr_WriteDirectionNeutral(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_DIRECTION_NEUTRAL;
// 	Protocol_Cmdr_StartReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// void MotorCmdr_WriteToggleAnalogIn(MotorCmdr_T * p_motorCmdr)
// {
// 	MotorCmdr_WriteVar(p_motorCmdr, MOT_VAR_USER_INPUT_MODE, 2U); /* 2 is Toggle */
// }
/******************************************************************************/
/*!
	Proctected Function - build packet, set wait state, without starting Tx
	@return length of full packet
*/
/******************************************************************************/
// void MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr) { _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_INIT_UNITS); }

// void _MotorCmdr_ProcTxIdle(MotorCmdr_T * p_motorCmdr)
// {
// 	if(Protocol_Cmdr_CheckTxIdle(&p_motorCmdr->Protocol) == true) { _MotorCmdr_Ping(p_motorCmdr); }
// }

// uint8_t _MotorCmdr_StopMotors(MotorCmdr_T * p_motorCmdr) 	{ return _Protocol_Cmdr_BuildTxReq_Overwrite(&p_motorCmdr->Protocol, MOT_PACKET_STOP_ALL); }
// uint8_t _MotorCmdr_Ping(MotorCmdr_T * p_motorCmdr) 			{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_PING); }
// uint8_t _MotorCmdr_SaveNvm(MotorCmdr_T * p_motorCmdr) 		{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_SAVE_NVM); }
// uint8_t _MotorCmdr_InitUnits(MotorCmdr_T * p_motorCmdr) 	{ return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_INIT_UNITS); }

// uint8_t _MotorCmdr_WriteVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId, uint32_t value)
// {
// 	p_motorCmdr->MotorWriteVarId = motVarId;
// 	p_motorCmdr->MotorWriteVarValue = value;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_WRITE_VAR);
// }

// uint8_t _MotorCmdr_ReadVar(MotorCmdr_T * p_motorCmdr, MotVarId_T motVarId)
// {
// 	p_motorCmdr->MotorReadVarId = motVarId;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_READ_VAR);
// }

// uint8_t _MotorCmdr_ReadSpeed(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->MonitorIdActive = MOT_PACKET_MONITOR_SPEED;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_MONITOR_TYPE);
// }

// uint8_t _MotorCmdr_ReadIFoc(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->MonitorIdActive = MOT_PACKET_MONITOR_I_FOC;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_MONITOR_TYPE);
// }

// /*
// 	overwrite even if sync is active expected response invalidate active req repse
// */
// uint8_t _MotorCmdr_WriteBrake(MotorCmdr_T * p_motorCmdr, uint16_t brake)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_BRAKE;
// 	p_motorCmdr->MotorCmdValue = brake;
// 	return _Protocol_Cmdr_BuildTxReq_Overwrite(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// uint8_t _MotorCmdr_WriteThrottle(MotorCmdr_T * p_motorCmdr, uint16_t throttle)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_THROTTLE;
// 	p_motorCmdr->MotorCmdValue = throttle;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// uint8_t _MotorCmdr_WriteRelease(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_RELEASE;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// uint8_t _MotorCmdr_WriteDirectionForward(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_DIRECTION_FORWARD;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// uint8_t _MotorCmdr_WriteDirectionReverse(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_DIRECTION_REVERSE;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }

// uint8_t _MotorCmdr_WriteDirectionNeutral(MotorCmdr_T * p_motorCmdr)
// {
// 	p_motorCmdr->ControlIdActive = MOT_PACKET_CONTROL_DIRECTION_NEUTRAL;
// 	return _Protocol_Cmdr_BuildTxReq(&p_motorCmdr->Protocol, MOT_PACKET_CONTROL_TYPE);
// }