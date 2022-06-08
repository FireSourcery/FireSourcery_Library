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
	@file 	MotorController_MotProtocol.c
	@author FireSoucery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_User.h"

/******************************************************************************/
/*!
	Req
*/
/******************************************************************************/

/******************************************************************************/
/*! Ping */
/******************************************************************************/
static void Req_Ping(MotorController_T * p_mc, MotPacket_PingResp_T * p_txPacket, size_t * p_txSize, const MotPacket_PingReq_T * p_rxPacket, size_t rxSize)
{
	(void)p_mc;
	(void)p_rxPacket;
	(void)rxSize;
	*p_txSize = MotPacket_PingResp_Build(p_txPacket);
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static void Req_StopAll(MotorController_T * p_mc, MotPacket_StopResp_T * p_txPacket, size_t * p_txSize, const MotPacket_StopReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	(void)p_rxPacket;
	(void)rxSize;
	MotorController_User_DisableControl(p_mc);
	*p_txSize = MotPacket_StopResp_Build(p_txPacket);
}

/******************************************************************************/
/*! Save Nvm All - Blocking, todo check exteneded timeout is needed */
/******************************************************************************/
static void Req_SaveNvm_Blocking(MotorController_T * p_mc, MotPacket_SaveNvmResp_T * p_txPacket, size_t * p_txSize, const MotPacket_SaveNvmReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	(void)p_rxPacket;
	(void)rxSize;
	MotorController_User_SaveParameters_Blocking(p_mc);
	*p_txSize = MotPacket_SaveNvmResp_Build(p_txPacket, p_mc->NvmStatus);
}


/******************************************************************************/
/*! Control */
/******************************************************************************/
static void Req_Control(MotorController_T * p_mc, MotPacket_ControlResp_T * p_txPacket, size_t * p_txSize, const MotPacket_ControlReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);

	switch(p_rxPacket->ControlReq.ControlId) 	// MotPacket_ControlReq_ParseId(p_monitorId, p_rxPacket);
	{
		case MOT_PACKET_CONTROL_RELEASE: 				MotorController_User_SetReleaseThrottle(p_mc); 									break;
		case MOT_PACKET_CONTROL_DIRECTION_FORWARD: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);	break;
		case MOT_PACKET_CONTROL_DIRECTION_REVERSE: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);	break;
		case MOT_PACKET_CONTROL_DIRECTION_NEUTRAL: 	MotorController_User_SetNeutral(p_mc); 		  									break;
		case MOT_PACKET_CONTROL_THROTTLE: 			Motor_User_SetThrottleCmd(p_motor, p_rxPacket->ControlReq.ValueU16s[0U]); 		break;
		case MOT_PACKET_CONTROL_BRAKE: 				Motor_User_SetBrakeCmd(p_motor, p_rxPacket->ControlReq.ValueU16s[0U]); 			break;
		default: break;
	}

	*p_txSize = MotPacket_ControlResp_MainStatus_Build(p_txPacket, 0U);
}

/******************************************************************************/
/*! Monitor */
/******************************************************************************/
// static void TxPacket_BuildMonitorAdcBatchMsb(MotorController_T * p_mc, MotPacket_MonitorResp_T * p_txPacket)
// {
// 	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
// 	p_txPacket->Monitor.ValueU8s[0U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_VPOS);
// 	p_txPacket->Monitor.ValueU8s[1U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_VACC);
// 	p_txPacket->Monitor.ValueU8s[2U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_VSENSE);
// 	p_txPacket->Monitor.ValueU8s[3U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_HEAT_PCB);
// 	p_txPacket->Monitor.ValueU8s[4U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP);
// 	p_txPacket->Monitor.ValueU8s[5U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT);
// 	p_txPacket->Monitor.ValueU8s[6U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_THROTTLE);
// 	p_txPacket->Monitor.ValueU8s[7U] = MotorController_User_GetAdcu_Msb8(p_mc, MOT_ANALOG_CHANNEL_BRAKE);
// 	p_txPacket->Monitor.ValueU8s[8U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_VA);
// 	p_txPacket->Monitor.ValueU8s[9U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_VB);
// 	p_txPacket->Monitor.ValueU8s[10U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_VC);
// 	p_txPacket->Monitor.ValueU8s[11U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_IA);
// 	p_txPacket->Monitor.ValueU8s[12U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_IB);
// 	p_txPacket->Monitor.ValueU8s[13U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_IC);
// 	p_txPacket->Monitor.ValueU8s[14U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_HEAT);
// 	p_txPacket->Monitor.ValueU8s[15U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_SIN);
// 	p_txPacket->Monitor.ValueU8s[16U] = Motor_User_GetAdcu_Msb8(p_motor, MOTOR_ANALOG_CHANNEL_COS);
// }

static void Req_Monitor(MotorController_T * p_mc, MotPacket_MonitorResp_T * p_txPacket, size_t * p_txSize, const MotPacket_MonitorReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	switch(p_rxPacket->MonitorReq.MonitorId)
	{
		case MOT_PACKET_MONITOR_SPEED: *p_txSize = MotPacket_MonitorResp_Speed_Build(p_txPacket, Motor_User_GetSpeed_Frac16(p_motor)); 												break;
		case MOT_PACKET_MONITOR_I_FOC:
			*p_txSize = MotPacket_MonitorResp_IFoc_Build
			(
				p_txPacket, FOC_GetIa(&p_motor->Foc), FOC_GetIb(&p_motor->Foc), FOC_GetIc(&p_motor->Foc),
				FOC_GetIalpha(&p_motor->Foc), FOC_GetIbeta(&p_motor->Foc),
				FOC_GetId(&p_motor->Foc), FOC_GetIq(&p_motor->Foc)
			);
			break;
		// case MOT_PACKET_MONITOR_ADC_BATCH_MSB:
		// 	TxPacket_BuildMonitorAdcBatchMsb(p_mc, p_txPacket);
		// 	*p_txSize = MotPacket_BuildHeader(p_txPacket, MOT_PACKET_MONITOR_ADC_BATCH_MSB, 16U);
		// 	break;
		default: break;
	}
}

/******************************************************************************/
/*! Write Single Var */
/******************************************************************************/
static void Req_WriteVar(MotorController_T * p_mc, MotPacket_WriteVarResp_T * p_txPacket, size_t * p_txSize, const MotPacket_WriteVarReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	MotPacket_HeaderStatus_T status = MOT_PACKET_HEADER_STATUS_OK;

	switch(p_rxPacket->WriteReq.MotVarId)
	{
		case MOT_VAR_POLE_PAIRS:  				Motor_User_SetPolePairs(p_motor, p_rxPacket->WriteReq.Value); 	break;
		case MOT_VAR_SPEED_FEEDBACK_REF_RPM:	break;
		case MOT_VAR_I_MAX_REF_AMP: 			status = MOT_PACKET_HEADER_STATUS_ERROR_WRITE_VAR_READ_ONLY; 	break;
		default: break;
	}

	*p_txSize = MotPacket_WriteVarResp_Build(p_txPacket, status);
}

/******************************************************************************/
/*! Read Single Var */
/******************************************************************************/
static void Req_ReadVar(MotorController_T * p_mc, MotPacket_ReadVarResp_T * p_txPacket, size_t * p_txSize, const MotPacket_ReadVarReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	// MotPacket_HeaderStatus_T status = MOT_PACKET_HEADER_STATUS_OK;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	uint32_t value = 0U;

	switch(p_rxPacket->ReadReq.MotVarId)
	{
		case MOT_VAR_POLE_PAIRS:  				value = Motor_User_GetPolePairs(p_motor); 	break;
		case MOT_VAR_SPEED_FEEDBACK_REF_RPM:	break;
		case MOT_VAR_I_MAX_REF_AMP: 			value = MotorController_User_GetIMax(p_mc); break;
		default: break;
	}

	*p_txSize = MotPacket_ReadVarResp_Build(p_txPacket, value);
}

/******************************************************************************/
/*! Init Units */
/******************************************************************************/
static void Req_InitUnits(MotorController_T * p_mc, MotPacket_InitUnitsResp_T * p_txPacket, size_t * p_txSize, const MotPacket_InitUnitsReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	(void)p_rxPacket;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	*p_txSize = MotPacket_InitUnitsResp_Build
	(
		p_txPacket,
		Motor_User_GetSpeedFeedbackRef_Rpm(p_motor), MotorController_User_GetIMax(p_mc), MotorController_User_GetVSupply(p_mc), /* Frac16 conversions */
		p_mc->VMonitorPos.CONFIG.UNITS_R1, p_mc->VMonitorPos.CONFIG.UNITS_R2,	/* Adcu <-> Volts conversions */
		p_mc->VMonitorSense.CONFIG.UNITS_R1, p_mc->VMonitorSense.CONFIG.UNITS_R2,
		p_mc->VMonitorAcc.CONFIG.UNITS_R1, p_mc->VMonitorAcc.CONFIG.UNITS_R2
	);
}



/******************************************************************************/
/*! Req Table */
/******************************************************************************/
static const Protocol_Req_T REQ_TABLE[] =
{
	PROTOCOL_REQ_DEFINE(MOT_PACKET_PING, 					Req_Ping, 				0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_STOP_ALL, 				Req_StopAll, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CMD_INIT_UNITS, 			Req_InitUnits, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CMD_SAVE_NVM, 			Req_SaveNvm_Blocking, 	0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CMD_WRITE_VAR, 			Req_WriteVar, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CMD_READ_VAR, 			Req_ReadVar, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CMD_MONITOR_TYPE, 		Req_Monitor, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CMD_CONTROL_TYPE, 		Req_Control, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
};


const Protocol_Specs_T MOTOR_CONTROLLER_MOT_PROTOCOL_SPECS =
{
	.RX_LENGTH_MIN 		= MOT_PACKET_LENGTH_MIN,
	.RX_LENGTH_MAX 		= MOT_PACKET_LENGTH_MAX,
	.PARSE_RX_META 		= (Protocol_ParseRxMeta_T)MotProtocol_ParseRxMeta,
	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)MotProtocol_BuildTxSync,
	.P_REQ_TABLE 		= &REQ_TABLE[0U],
	.REQ_TABLE_LENGTH 	= sizeof(REQ_TABLE)/sizeof(Protocol_Req_T),
	.REQ_EXT_RESET 		= 0U,

	.RX_START_ID 	= MOT_PACKET_START_BYTE,
	.RX_END_ID 		= 0x00U,
	.ENCODED 		= false,

	//default
	.RX_TIMEOUT 		= MOT_PROTOCOL_TIMEOUT_MS,
	.REQ_TIMEOUT		= MOT_PROTOCOL_TIMEOUT_MS,
	.BAUD_RATE_DEFAULT 	= MOT_PROTOCOL_BAUD_RATE_DEFAULT,
};