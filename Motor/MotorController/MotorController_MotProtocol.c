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
/*! Control */
/******************************************************************************/
static void Req_Control(MotorController_T * p_app, MotPacket_Resp_Control_T * p_txPacket, size_t * p_txSize, const MotPacket_Req_Control_T * p_rxPacket, size_t rxSize)
{
	(void)p_txPacket;
	(void)rxSize;

	Motor_T * p_motor = MotorController_GetPtrMotor(p_app, 0U);
	// void * p_ = &p_rxPacket->CmdControl;

	// MotPacket_ControlReq_ParseId(p_monitorId,  p_rxPacket);

	switch(p_rxPacket->CmdControl.ControlId)
	{
		case MOTPROTOCOL_CONTROL_THROTTLE:		Motor_User_SetThrottleCmd(p_motor, p_rxPacket->CmdControl.ValueU16s[0U]); 	break;	// ((MOTPROTOCOL_Payload_ControlType_Throttle_T *)p_rxCmd)->ThrottleValue
		case MOTPROTOCOL_CONTROL_BRAKE:			Motor_User_SetBrakeCmd(p_motor, p_rxPacket->CmdControl.ValueU16s[0U]); 		break;	// ((MOTPROTOCOL_Payload_ControlType_Throttle_T *)p_rxCmd)->ThrottleValue
		default: break;
	}

	// *p_txSize = MotPacket_ControlResp_Build(p_txPacket, status);

	*p_txSize = 0U;
}

/******************************************************************************/
/*! Monitor */
/******************************************************************************/
// static void TxPacket_BuildMonitorAdcBatchMsb(MotorController_T * p_app, MotPacket_Resp_Monitor_T * p_txPacket)
// {
// 	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_app, 0U);
// 	p_txPacket->Monitor.ValueU8s[0U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_VPOS);
// 	p_txPacket->Monitor.ValueU8s[1U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_VACC);
// 	p_txPacket->Monitor.ValueU8s[2U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_VSENSE);
// 	p_txPacket->Monitor.ValueU8s[3U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_HEAT_PCB);
// 	p_txPacket->Monitor.ValueU8s[4U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_HEAT_MOSFETS_TOP);
// 	p_txPacket->Monitor.ValueU8s[5U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_HEAT_MOSFETS_BOT);
// 	p_txPacket->Monitor.ValueU8s[6U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_THROTTLE);
// 	p_txPacket->Monitor.ValueU8s[7U] = MotorController_User_GetAdcu_Msb8(p_app, MOT_ANALOG_CHANNEL_BRAKE);
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


static void Req_Monitor(MotorController_T * p_app, MotPacket_Resp_Monitor_T * p_txPacket, size_t * p_txSize, const MotPacket_Req_Monitor_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_app, 0U);

	// MOTPROTOCOL_MONITOR_SPEED = 0x01U,		/* Speed in Q1.15 */
	// MOTPROTOCOL_MONITOR_I_PHASES = 0x02U,	/* Iabc in Q1.15 */
	// MOTPROTOCOL_MONITOR_I_FOC = 0x03U,		/* Idq, Ialphabeta in Q1.15 */
	// MOTPROTOCOL_MONITOR_V_PHASES = 0x04U,
	// MOTPROTOCOL_MONITOR_ANGLES = 0x05U,
	// MOTPROTOCOL_MONITOR_STATUS_FLAGS = 0x0FU,
	// MOTPROTOCOL_MONITOR_ANALOG_USER = 0x10U,
	// MOTPROTOCOL_MONITOR_HALL = 0x11U,
	// MOTPROTOCOL_MONITOR_SIN_COS = 0x12U,
	// MOTPROTOCOL_MONITOR_ENCODER = 0x13U,
	// MOTPROTOCOL_MONITOR_V_SENSORS = 0x21U, 		/* VSupply, ~5V, ~12V */
	// MOTPROTOCOL_MONITOR_HEAT = 0x22U,			/* In ADCU, Lower is higher */
	// MOTPROTOCOL_MONITOR_METERS = 0x31U, /* Speed, Angles */
	// MOTPROTOCOL_MONITOR_SPEED_RPM = 0x81U,	/* Speed in RPM */

	switch(p_rxPacket->CmdMonitor.MonitorId)
	{
		case MOTPROTOCOL_MONITOR_SPEED: 	*p_txSize = MotPacket_Resp_Monitor_Speed_Build(p_txPacket, Motor_User_GetSpeed_Fixed32(p_motor)); 												break;
		case MOTPROTOCOL_MONITOR_I_PHASES: 	*p_txSize = MotPacket_Resp_Monitor_IPhases_Build(p_txPacket, FOC_GetIa(&p_motor->Foc), FOC_GetIb(&p_motor->Foc), FOC_GetIc(&p_motor->Foc));		break;
		// case MOTPROTOCOL_MONITOR_ADC_BATCH_MSB:
		// 	TxPacket_BuildMonitorAdcBatchMsb(p_app, p_txPacket);
		// 	*p_txSize = MotPacket_BuildHeader(p_txPacket, MOTPROTOCOL_MONITOR_ADC_BATCH_MSB, 16U);
		// 	break;
		default: break;
	}
}

static void Req_Ping(MotorController_T * p_app, MotPacket_T * p_txPacket, size_t * p_txSize, const MotPacket_Sync_T * p_rxPacket, size_t rxSize)
{
	(void)p_app;
	(void)p_rxPacket;
	(void)rxSize;
	*p_txSize = MotPacket_PingResp_Build(p_txPacket);
}

static void Req_StopMotor(MotorController_T * p_app, MotPacket_T * p_txPacket, size_t * p_txSize, const MotPacket_Sync_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	(void)p_rxPacket;
	(void)rxSize;
	(void)p_txPacket;
	MotorController_User_DisableControl(p_app);
	// *p_txSize = MotPacket_BuildHeader(p_txPacket, MOTPROTOCOL_STOP_MOTORS, 0U);
	*p_txSize = 0U; //todo
}


/******************************************************************************/
/*! Req Table */
/******************************************************************************/
static const Protocol_Req_T REQ_TABLE[] =
{
	PROTOCOL_REQ_DEFINE(MOTPROTOCOL_PING, 				Req_Ping, 		0U, 	PROTOCOL_SYNC_ID_DISABLE ),
	PROTOCOL_REQ_DEFINE(MOTPROTOCOL_STOP_MOTORS, 		Req_StopMotor, 	0U, 	PROTOCOL_SYNC_ID_DISABLE ),
	PROTOCOL_REQ_DEFINE(MOTPROTOCOL_CMD_MONITOR_TYPE, 	Req_Monitor, 	0U, 	PROTOCOL_SYNC_ID_DISABLE ),
	PROTOCOL_REQ_DEFINE(MOTPROTOCOL_CMD_CONTROL_TYPE, 	Req_Control, 	0U, 	PROTOCOL_SYNC_ID_DISABLE ),
};


const Protocol_Specs_T MOTOR_CONTROLLER_MOT_PROTOCOL_SPECS =
{
	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

	.RX_TIMEOUT 		= MOTPROTOCOL_TIMEOUT_MS,
	.PARSE_RX_META		= (Protocol_ParseRxMeta_T)MotProtocol_ParseRxMeta,

	.P_REQ_TABLE 		= &REQ_TABLE[0U],
	.REQ_TABLE_LENGTH 	= sizeof(REQ_TABLE)/sizeof(Protocol_Req_T),
	.REQ_EXT_RESET 		= 0U,
	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,

	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)MotProtocol_BuildTxSync,

	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
	.RX_END_ID 		= 0x00U,
	.ENCODED 		= false,
	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
};