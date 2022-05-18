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
#include "Utility/Protocol/Concrete/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_User.h"


// /******************************************************************************/
// /*!
// 	Req
// */
// /******************************************************************************/
// /******************************************************************************/
// /*! Control */
// /******************************************************************************/
// static void Req_Control(MotorController_T * p_app, MotProtocol_RespPacket_Control_T * p_txPacket, size_t * p_txSize, const MotProtocol_ReqPacket_Control_T * p_rxPacket, size_t rxSize)
// {
// 	Motor_T * p_motor = MotorController_GetPtrMotor(p_app, 0U);
// 	// void * p_rxCmd = &p_rxPacket->CmdControl;

// 	switch(p_rxPacket->CmdControl.ControlId)
// 	{
// 		case MOTPROTOCOL_CONTROL_THROTTLE:		Motor_User_SetThrottleCmd(p_motor, p_rxPacket->CmdControl.ValueU16s[0U]); 	break;	// ((MOTPROTOCOL_Payload_ControlType_Throttle_T *)p_rxCmd)->ThrottleValue
// 		case MOTPROTOCOL_CONTROL_BRAKE:		Motor_User_SetBrakeCmd(p_motor, p_rxPacket->CmdControl.ValueU16s[0U]); 		break;	// ((MOTPROTOCOL_Payload_ControlType_Throttle_T *)p_rxCmd)->ThrottleValue

// 		default: break;
// 	}

// 	*p_txSize = 0U;
// }

// /******************************************************************************/
// /*! Monitor */
// /******************************************************************************/
// static void TxPacket_BuildMonitorAdcBatchMsb(MotorController_T * p_app, MotProtocol_RespPacket_Monitor_T * p_txPacket)
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


// // static void MotorCtrlr_BuildRespMonitor(const MotorController_T * p_app,  monitoId id)
// // {
// // 	MotProtocol_Interface_T * p_interface = p_app->MotProtocolInterface;

// // 	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_app, 0U);

// // 	switch(id)
// // 	{
// // 		case MOTPROTOCOL_MONITOR_SPEED:
// // 			p_interface->RespMonitor.S32s[0U] = Motor_User_GetSpeed_Frac16(p_motor);
// // 			break;
// // 		case MOTPROTOCOL_MONITOR_I_PHASES:
// // 			p_interface->IPhases.PhaseA = FOC_GetIa(&p_motor->Foc);
// // 			p_interface->IPhases.PhaseB = FOC_GetIb(&p_motor->Foc);
// // 			p_interface->IPhases.PhaseC = FOC_GetIc(&p_motor->Foc);
// // 			break;

// // 		case MOTPROTOCOL_MONITOR_ADC_BATCH_MSB:
// // 			// TxPacket_BuildMonitorAdcBatchMsb(p_interface);
// // 			break;
// // 		default: break;
// // 	}
// // }


// static void Req_Monitor(MotorController_T * p_app, MotProtocol_RespPacket_Monitor_T * p_txPacket, size_t * p_txSize, const MotProtocol_ReqPacket_Monitor_T * p_rxPacket, size_t rxSize)
// {
// 	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_app, 0U);

// 	// MOTPROTOCOL_MONITOR_SPEED = 0x01U,		/* Speed in Q1.15 */
// 	// MOTPROTOCOL_MONITOR_I_PHASES = 0x02U,	/* Iabc in Q1.15 */
// 	// MOTPROTOCOL_MONITOR_I_FOC = 0x03U,		/* Idq, Ialphabeta in Q1.15 */
// 	// MOTPROTOCOL_MONITOR_V_PHASES = 0x04U,
// 	// MOTPROTOCOL_MONITOR_ANGLES = 0x05U,
// 	// MOTPROTOCOL_MONITOR_STATUS_FLAGS = 0x0FU,
// 	// MOTPROTOCOL_MONITOR_ANALOG_USER = 0x10U,
// 	// MOTPROTOCOL_MONITOR_HALL = 0x11U,
// 	// MOTPROTOCOL_MONITOR_SIN_COS = 0x12U,
// 	// MOTPROTOCOL_MONITOR_ENCODER = 0x13U,
// 	// MOTPROTOCOL_MONITOR_V_SENSORS = 0x21U, 		/* VSupply, ~5V, ~12V */
// 	// MOTPROTOCOL_MONITOR_HEAT = 0x22U,			/* In ADCU, Lower is higher */
// 	// MOTPROTOCOL_MONITOR_METERS = 0x31U, /* Speed, Angles */
// 	// MOTPROTOCOL_MONITOR_SPEED_RPM = 0x81U,	/* Speed in RPM */

// 	switch(p_rxPacket->CmdMonitor.ControlId)
// 	{
// 		case MOTPROTOCOL_MONITOR_SPEED:
// 			p_txPacket->Monitor.ValueS32s[0U] = Motor_User_GetSpeed_Frac16(p_motor);
// 			*p_txSize = MotProtocol_Packet_BuildHeader(p_txPacket, MOTPROTOCOL_MONITOR_SPEED, 4U);
// 			break;
// 		case MOTPROTOCOL_MONITOR_I_PHASES:
// 			p_txPacket->Monitor.ValueS16s[0U] = FOC_GetIa(&p_motor->Foc);
// 			p_txPacket->Monitor.ValueS16s[1U] = FOC_GetIb(&p_motor->Foc);
// 			p_txPacket->Monitor.ValueS16s[2U] = FOC_GetIc(&p_motor->Foc);
// 			*p_txSize = MotProtocol_Packet_BuildHeader(p_txPacket, MOTPROTOCOL_MONITOR_I_PHASES, 6U);
// 			break;

// 		case MOTPROTOCOL_MONITOR_ADC_BATCH_MSB:
// 			TxPacket_BuildMonitorAdcBatchMsb(p_app, p_txPacket);
// 			*p_txSize = MotProtocol_Packet_BuildHeader(p_txPacket, MOTPROTOCOL_MONITOR_ADC_BATCH_MSB, 16U);
// 			break;
// 		default: break;
// 	}
// }

// static void Req_Ping(MotorController_T * p_app, MotProtocol_Packet_T * p_txPacket, size_t * p_txSize, const MotProtocol_Packet_T * p_rxPacket, size_t rxSize)
// {
// 	p_txPacket->Payload[0U] = MOTPROTOCOL_VERSION_BUGFIX;
// 	p_txPacket->Payload[1U] = MOTPROTOCOL_VERSION_MINOR;
// 	p_txPacket->Payload[2U] = MOTPROTOCOL_VERSION_MAJOR;
// 	p_txPacket->Payload[3U] = MOTPROTOCOL_VERSION_OPT;

// 	*p_txSize = MotProtocol_Packet_BuildHeader(p_txPacket, MOTPROTOCOL_PING, 4U);
// }

// static void Req_StopMotor(MotorController_T * p_app, MotProtocol_Packet_T * p_txPacket, size_t * p_txSize, const MotProtocol_Packet_T * p_rxPacket, size_t rxSize)
// {
// 	MotorController_User_DisableControl(p_app);
// 	MotProtocol_Packet_BuildHeader(p_txPacket, MOTPROTOCOL_STOP_MOTOR, 0U);
// }


// /******************************************************************************/
// /*! Req Table */
// /******************************************************************************/
// #define SYNC_DISABLE_ID 0U

// static const Protocol_Req_T REQ_TABLE[] =
// {
// 	PROTOCOL_CONFIG_REQ(MOTPROTOCOL_PING, 				Req_Ping, 		0U, 	SYNC_DISABLE_ID ),
// 	PROTOCOL_CONFIG_REQ(MOTPROTOCOL_STOP_MOTOR, 		Req_Ping, 		0U, 	SYNC_DISABLE_ID ),
// 	PROTOCOL_CONFIG_REQ(MOTPROTOCOL_CMD_MONITOR_TYPE, 	Req_Monitor, 	0U, 	SYNC_DISABLE_ID ),
// 	PROTOCOL_CONFIG_REQ(MOTPROTOCOL_CMD_CONTROL_TYPE, 	Req_Control, 	0U, 	SYNC_DISABLE_ID ),
// };


// /******************************************************************************/
// /*! RxParser */
// /******************************************************************************/
// //controller side parse
// static Protocol_RxCode_T RxPacket_Parse(protocol_reqid_t * p_reqId, size_t * p_rxRemaining, const MotProtocol_Packet_T * p_rxPacket, size_t rxCount)
// {
// 	Protocol_RxCode_T rxCode = PROTOCOL_RX_CODE_WAIT_PACKET;

// 	if(*p_rxRemaining == 0U) // if(rxCount >= 3U) && (rxCount >= p_rxPacket->Header.Length + sizeof(MotProtocol_Header_T))
// 	{
// 		*p_reqId = p_rxPacket->Header.TypeId;
// 		rxCode = (Packet_CalcChecksum(p_rxPacket) == p_rxPacket->Header.Crc) ? PROTOCOL_RX_CODE_REQ_ID_SUCCESS : PROTOCOL_RX_CODE_ERROR_PACKET_DATA;

// 		// optionally further refine cmd
// 		// if(rxCount >= p_rxPacket->Header.Length + sizeof(MotProtocol_Header_T))
// 		// {
// 		// 	switch(p_rxPacket->Header.TypeId)
// 		// 	{
// 		// 		case MOTPROTOCOL_CMD_MONITOR_TYPE: break;
// 		// 		case MOTPROTOCOL_CMD_CONTROL_TYPE: break;
// 		// 		default:  break;
// 		// 	}
// 		// 	*p_reqId = p_rxPacket->Header.TypeId;
// 		// 	rxCode = (Packet_CalcChecksum(p_rxPacket) == p_rxPacket->Header.Crc) ? PROTOCOL_RX_CODE_REQ_ID_SUCCESS : PROTOCOL_RX_CODE_ERROR_PACKET_DATA;
// 		// }
// 	}
// 	else if(rxCount >= 3U) /* Move this to module handle? */
// 	{
// 		p_rxRemaining = p_rxPacket->Header.Length + sizeof(MotProtocol_Header_T) - 3U;
// 		rxCode = PROTOCOL_RX_CODE_WAIT_PACKET_PAYLOAD;
// 	}
// 	else if(rxCount >= MOTPROTOCOL_PACKET_LENGTH_MIN)
// 	{
// 		switch(p_rxPacket->Header.TypeId)
// 		{
// 			case MOTPROTOCOL_STOP_MOTOR:	rxCode = PROTOCOL_RX_CODE_REQ_ID_SUCCESS; *p_reqId = MOTPROTOCOL_STOP_MOTOR;	break;
// 			case MOTPROTOCOL_PING:			rxCode = PROTOCOL_RX_CODE_REQ_ID_SUCCESS; *p_reqId = MOTPROTOCOL_PING;			break;
// 			case MOTPROTOCOL_SYNC_ACK:		rxCode = PROTOCOL_RX_CODE_ACK; 				break;
// 			case MOTPROTOCOL_SYNC_NACK:		rxCode = PROTOCOL_RX_CODE_NACK; 			break;
// 			case MOTPROTOCOL_SYNC_ABORT:	rxCode = PROTOCOL_RX_CODE_ABORT; 			break;
// 			default: break;
// 		}
// 	}

// 	return rxCode;
// }

// static void TxPacket_BuildSync(MotProtocol_SyncPacket_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId)
// {
// 	switch(txId)
// 	{
// 		case PROTOCOL_TX_SYNC_ACK_REQ_ID:			p_txPacket->Start = MOTPROTOCOL_START_BYTE, p_txPacket->SyncId = MOTPROTOCOL_SYNC_ACK; 		break;
// 		case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:	p_txPacket->Start = MOTPROTOCOL_START_BYTE, p_txPacket->SyncId = MOTPROTOCOL_SYNC_NACK;		break;
// 		case PROTOCOL_TX_SYNC_NACK_REQ_ID:			p_txPacket->Start = MOTPROTOCOL_START_BYTE, p_txPacket->SyncId = MOTPROTOCOL_SYNC_NACK;		break;
// 		case PROTOCOL_TX_SYNC_ACK_DATA:				p_txPacket->Start = MOTPROTOCOL_START_BYTE, p_txPacket->SyncId = MOTPROTOCOL_SYNC_ACK;		break;
// 		case PROTOCOL_TX_SYNC_NACK_DATA:			p_txPacket->Start = MOTPROTOCOL_START_BYTE, p_txPacket->SyncId = MOTPROTOCOL_SYNC_NACK;		break;
// 		case PROTOCOL_TX_SYNC_ABORT:				p_txPacket->Start = MOTPROTOCOL_START_BYTE, p_txPacket->SyncId = MOTPROTOCOL_SYNC_ABORT;	break;
// 		// case PROTOCOL_TX_SYNC_NACK_TIMEOUT: 				memcpy(&p_txPacket[0U], &NACK_STRING[0U], sizeof(NACK_STRING)); 	break;
// 		// case PROTOCOL_TX_SYNC_NACK_PACKET_ECC_ERROR: 	memcpy(&p_txPacket[0U], &NACK_STRING[0U], sizeof(NACK_STRING));		break;
// 		default: *p_txSize = 0U; break;
// 	}

// 	*p_txSize = 2U;
// 	// MotProtocol_SyncPacket_Build(  p_txPacket,  p_txSize,   txId);
// }


// static void TxPacket_BuildSync(MotProtocol_SyncPacket_T * p_txPacket, size_t * p_txSize, Protocol_TxSyncId_T txId)
// {
// 	switch(txId)
// 	{
// 		case PROTOCOL_TX_SYNC_ACK_REQ_ID:			MotProtocol_SyncPacket_Build(p_txPacket, p_txSize, MOTPROTOCOL_SYNC_ACK); 		break;
// 		case PROTOCOL_TX_SYNC_NACK_PACKET_ERROR:	MotProtocol_SyncPacket_Build(p_txPacket, p_txSize, MOTPROTOCOL_SYNC_NACK); 		break;
// 		case PROTOCOL_TX_SYNC_NACK_REQ_ID:			MotProtocol_SyncPacket_Build(p_txPacket, p_txSize, MOTPROTOCOL_SYNC_ACK); 		break;
// 		case PROTOCOL_TX_SYNC_ACK_DATA:				MotProtocol_SyncPacket_Build(p_txPacket, p_txSize, MOTPROTOCOL_SYNC_NACK); 		break;
// 		case PROTOCOL_TX_SYNC_NACK_DATA:			MotProtocol_SyncPacket_Build(p_txPacket, p_txSize, MOTPROTOCOL_SYNC_NACK); 		break;
// 		case PROTOCOL_TX_SYNC_ABORT:				MotProtocol_SyncPacket_Build(p_txPacket, p_txSize, MOTPROTOCOL_SYNC_ABORT); 	break;
// 		// case PROTOCOL_TX_SYNC_NACK_TIMEOUT: 				memcpy(&p_txPacket[0U], &NACK_STRING[0U], sizeof(NACK_STRING)); 	break;
// 		// case PROTOCOL_TX_SYNC_NACK_PACKET_ECC_ERROR: 	memcpy(&p_txPacket[0U], &NACK_STRING[0U], sizeof(NACK_STRING));		break;
// 		default: *p_txSize = 0U; break;
// 	}
// }


// static void Req_ResetState(void * p_subState)
// {
// 	// p_subState->StateIndex = 0U;
// }

// const Protocol_Specs_T MOTPROTOCOL_SPECS =
// {
// 	.RX_TIMEOUT 	= MOTPROTOCOL_TIMEOUT_MS,
// 	.RX_LENGTH_MIN 	= MOTPROTOCOL_PACKET_LENGTH_MIN,
// 	.RX_LENGTH_MAX 	= MOTPROTOCOL_PACKET_LENGTH_MAX,

// 	.PARSE_RX 			= (Protocol_ProcRx_T)RxPacket_Parse,
// 	.BUILD_TX_SYNC 		= (Protocol_BuildTxSync_T)TxPacket_BuildSync,

// 	.P_REQ_TABLE 		= &REQ_TABLE[0U],
// 	.REQ_TABLE_LENGTH 	= sizeof(REQ_TABLE)/sizeof(Protocol_Req_T),
// 	.REQ_EXT_RESET 		= 0U,
// 	.REQ_TIMEOUT		= MOTPROTOCOL_TIMEOUT_MS,

// 	.RX_START_ID 	= MOTPROTOCOL_START_BYTE,
// 	.RX_END_ID 		= 0x00U,
// 	.ENCODED 		= false,

// 	.BAUD_RATE_DEFAULT = MOTPROTOCOL_BAUD_RATE_DEFAULT,
// };