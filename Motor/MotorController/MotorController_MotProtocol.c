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
	@file 	MotorController_MotProtocol.c
	@author FireSourcery
	@brief
	@version V0
*/
/******************************************************************************/
#include "Motor/MotProtocol/MotPacket.h"
#include "Motor/MotProtocol/MotProtocol.h"
#include "Motor/MotorController/MotorController_User.h"


/******************************************************************************/
/*!
	Notes
	Directly use MotorController_T as Protocol interface (P_APP_INTERFACE)
	avoids double buffering
*/
/******************************************************************************/

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
	(void)p_mc; (void)p_rxPacket; (void)rxSize;
	*p_txSize = MotPacket_PingResp_Build(p_txPacket);
}

/******************************************************************************/
/*! Stop All */
/******************************************************************************/
static void Req_StopAll(MotorController_T * p_mc, MotPacket_StopResp_T * p_txPacket, size_t * p_txSize, const MotPacket_StopReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize; (void)p_rxPacket; (void)rxSize;
	MotorController_User_DisableControl(p_mc);
	*p_txSize = MotPacket_StopResp_Build(p_txPacket);
}

/******************************************************************************/
/*! Save Nvm All - Blocking, todo check exteneded timeout is needed */
/******************************************************************************/
static void Req_SaveNvm_Blocking(MotorController_T * p_mc, MotPacket_SaveNvmResp_T * p_txPacket, size_t * p_txSize, const MotPacket_SaveNvmReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize; (void)p_rxPacket; (void)rxSize;
	MotPacket_HeaderStatus_T status;

	MotorController_User_SaveParameters_Blocking(p_mc);
	status = (p_mc->NvmStatus == NV_MEMORY_STATUS_SUCCESS) ? MOT_PACKET_HEADER_STATUS_OK : MOT_PACKET_HEADER_STATUS_ERROR;
	*p_txSize = MotPacket_SaveNvmResp_Build(p_txPacket, status);
}

static uint8_t WriteVar(MotorController_T * p_mc, MotVarId_T varId, uint32_t varValue)
{
	uint8_t writeSize = 0U; //0 is error
	//bool, enum = 2
	//uint32_t = 4

	switch(varId)
	{
		case MOT_VAR_THROTTLE:		MotorController_User_SetCmdThrottle(p_mc, (uint16_t)varValue); 						break;	//todo mc statemachine handle
		case MOT_VAR_BRAKE:			MotorController_User_SetCmdBrake(p_mc, (uint16_t)varValue); 						break;
		case MOT_VAR_DIRECTION:		MotorController_User_SetDirection(p_mc, (MotorController_Direction_T)varValue); 	break; 	/* Value 0: Neutral, 1: Reverse, 2: Forward */

		case MOT_VAR_SPEED_RPM:			  	break;
		case MOT_VAR_MC_STATE:				break;
		case MOT_VAR_ERROR_CODE:		  	break;
		case MOT_VAR_BEEP:		  	MotorController_User_BeepN(p_mc, varValue, 1000U, 1U);	break;


		// case MOT_VAR_I_PEAK_AMP:		  	break;
		// case MOT_VAR_SPEED_GROUND_KMH:	  	break;
		// case MOT_VAR_HEAT_PCB_DEG_C:	 	break;
		// case MOT_VAR_FOC_IQ:			 	break;

		case MOT_VAR_PARAM_TEST_BEGIN:		break;					/*  */
		case MOT_VAR_PARAM_TEST_1: 			p_mc->Parameters.Test[0U] = varValue;	break;						/* Value 16-bit */
		case MOT_VAR_PARAM_TEST_2: 			p_mc->Parameters.Test[1U] = varValue;	break;						/* Value 32-bit */
		case MOT_VAR_PARAM_TEST_3: 			p_mc->Parameters.Test[2U] = varValue;	break;						/* Value 0, 1 */
		case MOT_VAR_PARAM_TEST_4: 			break;

		case MOT_VAR_POLE_PAIRS:  			Motor_User_SetPolePairs(MotorController_User_GetPtrMotor(p_mc, 0U), varValue); writeSize = 4U;	break;
		// case MOT_VAR_SPEED_RPM:	  	break;
		case MOT_VAR_SPEED_FEEDBACK_REF_RPM:	writeSize = 0U; 	break;
		case MOT_VAR_I_MAX_REF_AMP: 			writeSize = 0U; 	break;
		default: break;
	}

	return writeSize;
}

/******************************************************************************/
/*! Write Single Var */
/******************************************************************************/
static void Req_WriteVar(MotorController_T * p_mc, MotPacket_WriteVarResp_T * p_txPacket, size_t * p_txSize, const MotPacket_WriteVarReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	MotPacket_HeaderStatus_T status = (WriteVar(p_mc, p_rxPacket->WriteReq.MotVarId, p_rxPacket->WriteReq.Value) == 0U) ?
		MOT_PACKET_HEADER_STATUS_ERROR : MOT_PACKET_HEADER_STATUS_OK;

	*p_txSize = MotPacket_WriteVarResp_Build(p_txPacket, status);
}

static uint32_t GetVar(MotorController_T * p_mc, MotVarId_T varId)
{
	uint32_t value = 0U;

	switch(varId)
	{
		case MOT_VAR_THROTTLE:		value = MotAnalogUser_GetThrottle(&p_mc->AnalogUser); break;					/* Value 16-bit */
		case MOT_VAR_BRAKE:			value = MotAnalogUser_GetBrake(&p_mc->AnalogUser); break;					/* Value 16-bit */
		case MOT_VAR_DIRECTION:		value = (uint32_t)MotorController_User_GetDirection(p_mc); break;				/* Value 0: Neutral, 1: Reverse, 2: Forward */

		case MOT_VAR_SPEED_RPM:		value = Motor_User_GetSpeed_Rpm(MotorController_User_GetPtrMotor(p_mc, 0U)); 	break;
		case MOT_VAR_ERROR_CODE:	value = p_mc->FaultFlags.State;	break;
		case MOT_VAR_MC_STATE:		value = (uint32_t)MotorController_User_GetStateId(p_mc); 	break;
		// case MOT_VAR_I_PEAK_AMP:		Motor_User_GetIPhase_Amp(MotorController_User_GetPtrMotor(p_mc, 0U)); 	break;
		// case MOT_VAR_SPEED_GROUND_KMH:	Motor_User_GetSpeed_Rpm(MotorController_User_GetPtrMotor(p_mc, 0U)); 	break;
		// case MOT_VAR_HEAT_PCB_DEG_C:	MotorController_User_GetHeatPcb_DegC(p_mc, 1U); 			break;
		// case MOT_VAR_FOC_IQ:			MotorController_User_GetPtrMotor(p_mc, 0U)->Foc.Iq; 		break;

		case MOT_VAR_PARAM_TEST_BEGIN:	break;					/*  */
		case MOT_VAR_PARAM_TEST_1:	value = p_mc->Parameters.Test[0U];	break;						/* Value 16-bit */
		case MOT_VAR_PARAM_TEST_2:	value = p_mc->Parameters.Test[1U];	break;						/* Value 32-bit */
		case MOT_VAR_PARAM_TEST_3:	value = p_mc->Parameters.Test[2U];	break;						/* Value 0, 1 */
		case MOT_VAR_PARAM_TEST_4:	value = p_mc->Parameters.Test[3U];	break;						/* Value enum: 0:White, 1:Black, 2:Red, */
		// case MOT_VAR_PARAM_TEST_5:	value = p_mc->Parameters.Test[4U];	break;						/*   */

		case MOT_VAR_POLE_PAIRS:  				value = Motor_User_GetPolePairs(MotorController_User_GetPtrMotor(p_mc, 0U)); 	break;
		case MOT_VAR_SPEED_FEEDBACK_REF_RPM:	break;
		case MOT_VAR_I_MAX_REF_AMP: 			value = MotorController_User_GetIMax(p_mc); break;
		default: break;
	}

	return value;
}

/******************************************************************************/
/*! Read Single Var */
/******************************************************************************/
static void Req_ReadVar(MotorController_T * p_mc, MotPacket_ReadVarResp_T * p_txPacket, size_t * p_txSize, const MotPacket_ReadVarReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	*p_txSize = MotPacket_ReadVarResp_Build(p_txPacket, GetVar(p_mc, p_rxPacket->ReadReq.MotVarId));
}

/******************************************************************************/
/*! Write 8 Var */
/******************************************************************************/
// static void Req_WriteVars8(MotorController_T * p_mc, MotPacket_WriteVars8Resp_T * p_txPacket, size_t * p_txSize, const MotPacket_WriteVars8Req_T * p_rxPacket, size_t rxSize)
// {
// 	// (void)rxSize;
// 	// Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
// 	// MotPacket_HeaderStatus_T status = MOT_PACKET_HEADER_STATUS_OK;


// 	// for(uint8_t iId = 0U, iValue = 0U, varLength = 1U; iId < MotPacket_WriteVars8Req_GetVarCount(p_rxPacket); iId++, iValue += varLength)
// 	// {
// 	// 	// varLength = MotPacket_WriteVars8Req_GetVarLength(p_rxPacket, p_rxPacket->WriteVars8Req.MotVarIds[iCount])
// 	// 	status = WriteVarMap(p_mc, &varLength, p_rxPacket->WriteVars8Req.MotVarIds[iId], &p_rxPacket->WriteVars8Req.Value16[iValue]);
// 	// 	if(status != MOT_PACKET_HEADER_STATUS_OK) { break; }
// 	// }

// 	// *p_txSize = MotPacket_WriteVars8Resp_Build(p_txPacket, status);
// }

/******************************************************************************/
/*! Read 16 Var */
/******************************************************************************/
// static void Req_ReadVars16(MotorController_T * p_mc, MotPacket_ReadVars16Resp_T * p_txPacket, size_t * p_txSize, const MotPacket_ReadVarReq_T * p_rxPacket, size_t rxSize)
// {
// 	// (void)rxSize;
// 	// // MotPacket_HeaderStatus_T status = MOT_PACKET_HEADER_STATUS_OK;
// 	// Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
// 	// uint32_t value = 0U;
// 	// *p_txSize = MotPacket_ReadVarResp_Build(p_txPacket, value);
// }

/******************************************************************************/
/*! Control */
/******************************************************************************/
static void Req_Control(MotorController_T * p_mc, MotPacket_ControlResp_T * p_txPacket, size_t * p_txSize, const MotPacket_ControlReq_T * p_rxPacket, size_t rxSize)
{
	(void)rxSize;
	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);

	switch(p_rxPacket->ControlReq.ControlId) 	// MotPacket_ControlReq_ParseId(p_monitorId, p_rxPacket);
	{
		// case MOT_PACKET_CONTROL_RELEASE: 			MotorController_User_SetReleaseThrottle(p_mc); 									break;
		case MOT_PACKET_CONTROL_RELEASE: 			MotorController_User_SetCmdZero(p_mc); 											break;
		case MOT_PACKET_CONTROL_DIRECTION_FORWARD: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);	break;
		case MOT_PACKET_CONTROL_DIRECTION_REVERSE: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);	break;
		case MOT_PACKET_CONTROL_DIRECTION_NEUTRAL: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_NEUTRAL);	break;
		// case MOT_PACKET_CONTROL_DIRECTION_NEUTRAL: 	MotorController_User_SetNeutral(p_mc); 											break;
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
		Motor_User_GetSpeedFeedbackRef_Rpm(p_motor), MotorController_User_GetIMax(p_mc), MotorController_User_GetVSource(p_mc), /* Frac16 conversions */
		p_mc->VMonitorPos.CONFIG.UNITS_R1, p_mc->VMonitorPos.CONFIG.UNITS_R2,	/* Adcu <-> Volts conversions */
		p_mc->VMonitorSense.CONFIG.UNITS_R1, p_mc->VMonitorSense.CONFIG.UNITS_R2,
		p_mc->VMonitorAcc.CONFIG.UNITS_R1, p_mc->VMonitorAcc.CONFIG.UNITS_R2
	);
}

/******************************************************************************/
/*! Stateful Read Data */
/******************************************************************************/
static Protocol_ReqCode_T Req_ReadData
(
	MotProtocol_Substate_T * p_subState, void * p_appInterface,
	MotPacket_T * p_txPacket, size_t * p_txSize,
	const MotPacket_T * p_rxPacket, size_t rxSize
)
{
	(void)rxSize;
	(void)p_appInterface;
	Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_WAIT_PROCESS;
	// MotPacket_HeaderStatus_T headerStatus = MOT_PACKET_HEADER_STATUS_OK;
	uint16_t readSize;

	switch(p_subState->StateId)
	{
		case 0U: /* Tx Ack handled by common Sync */
			// p_respPayload = (MotPacket_ReadDataResp_Payload_T *)p_rxPacket->Payload;
			// p_subState->p_DataModeAddress = (uint8_t *)p_rxPacket->ReadDataReq.AddressStart;
			// p_subState->DataModeSize = p_rxPacket->ReadDataReq.SizeBytes;
			MotPacket_ReadDataReq_Parse(&p_subState->DataModeAddress, &p_subState->DataModeSize, (MotPacket_ReadDataReq_T *)p_rxPacket);
			p_subState->StateId = 1U;
			*p_txSize = MotPacket_ReadDataResp_Build((MotPacket_ReadDataResp_T *)p_txPacket, MOT_PACKET_HEADER_STATUS_OK);
			reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
			break;

		case 1U: /* Tx Data */
			if(p_subState->DataModeSize > 0U)
			{
				//sequenceid*32 == data
				readSize = (p_subState->DataModeSize > 32U) ? 32U : p_subState->DataModeSize;
				*p_txSize = MotPacket_DataPacket_Build((MotPacket_DataPacket_T *)p_txPacket, (uint8_t *)p_subState->DataModeAddress, readSize);
				p_subState->DataModeSize -= readSize;
				p_subState->DataModeAddress += readSize;
				reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
			}
			else /* (p_subState->DataModeSize == 0U) */
			{
				*p_txSize = MotPacket_ReadDataResp_Build((MotPacket_ReadDataResp_T *)p_txPacket, MOT_PACKET_HEADER_STATUS_OK);
				reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
			}

		default:
			break;
	}

	return reqCode;
}

/******************************************************************************/
/*! Stateful Write Data */
/******************************************************************************/
static Protocol_ReqCode_T Req_WriteData_Blocking
(
	MotProtocol_Substate_T * p_subState, MotorController_T * p_appInterface,
	MotPacket_WriteDataResp_T * p_txPacket, size_t * p_txSize,
	const MotPacket_WriteDataReq_T * p_rxPacket, size_t rxSize
)
{
	(void)rxSize;
	Protocol_ReqCode_T reqCode = PROTOCOL_REQ_CODE_WAIT_PROCESS;
	Flash_T * p_flash = p_appInterface->CONFIG.P_FLASH;
	Flash_Status_T flashStatus;
	MotPacket_HeaderStatus_T headerStatus;
	const uint8_t * p_writeData; /* DataPacket Payload */
	uint8_t writeSize; /* DataPacket Size */

	switch(p_subState->StateId)
	{
		case 0U: /* Tx Ack handled by Common Req Sync */
			MotPacket_WriteDataReq_Parse(&p_subState->DataModeAddress, &p_subState->DataModeSize, (MotPacket_WriteDataReq_T *)p_rxPacket);
			flashStatus = Flash_SetWrite(p_flash, (uint8_t *)p_subState->DataModeAddress, 0U, p_subState->DataModeSize);  //use full set to check boundaries. bytecount will be overwritten
			headerStatus = (flashStatus == FLASH_STATUS_SUCCESS) ? MOT_PACKET_HEADER_STATUS_OK : MOT_PACKET_HEADER_STATUS_ERROR;
			// motPacketStatus = RemapFlashStatus(flashStatus);
			// p_subState->DataModeSize = p_rxPacket->WriteDataReq.SizeBytes;
			// p_subState->IsDataModeActive = true;
			p_subState->StateId = 1U;
			*p_txSize = MotPacket_WriteDataResp_Build((MotPacket_WriteDataResp_T *)p_txPacket, headerStatus);
			reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_SYNC;
			break;

		case 1U: /* Await Data */
			p_subState->StateId = 2U;
			reqCode = PROTOCOL_REQ_CODE_AWAIT_RX_REQ_EXT;
			break;

		case 2U: /* Write Data - rxPacket is DataPacket */
			// writeSize = MotPacket_DataPacket_ParseDataLength((MotPacket_DataPacket_T *)p_rxPacket);
			MotPacket_DataPacket_Parse(&p_writeData, &writeSize, (const MotPacket_DataPacket_T *)p_rxPacket);
			if(p_subState->DataModeSize >= writeSize)
			{
				flashStatus = Flash_ContinueWrite_Blocking(p_flash, p_writeData, writeSize);
				if(flashStatus == FLASH_STATUS_SUCCESS)
				{
					p_subState->DataModeSize -= writeSize;
					// p_subState->DataPhaseBytes -= p_rxPacket->framingPacket.length;
					// if(p_subState->DataPhaseBytes > 0U) { p_subState->StateIndex = 1U; }
					// else { p_subState->StateIndex = 3U; }
					reqCode = PROTOCOL_REQ_CODE_TX_ACK; /* need separate state for tx response after tx ack */
				}
				else
				{
					// *p_txSize = NxpBootPacket_BuildGenericResponse(p_txPacket, RemapFlashStatus(flashStatus), kCommandTag_WriteMemory);
					// *p_txSize += NxpBootPacket_BuildFraming(p_txPacket, kFramingPacketType_Command, *p_txSize);
					reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
				}
			}
			else
			{
				// *p_txSize = NxpBootPacket_BuildGenericResponse(p_txPacket, kStatus_MemoryWriteFailed, kCommandTag_WriteMemory);
				// *p_txSize += NxpBootPacket_BuildFraming(p_txPacket, kFramingPacketType_Command, *p_txSize);
				reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
			}
			break;

		case 3U: /* Tx Final Response */ /* Rx Ack handled by Common Req Sync */
			// p_subState->IsDataModeActive = false;
			*p_txSize = MotPacket_WriteDataResp_Build((MotPacket_WriteDataResp_T *)p_txPacket, p_subState->WriteModeStatus);
			reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
			// *p_txSize = NxpBootPacket_BuildGenericResponse(p_txPacket, kStatus_Success, kCommandTag_WriteMemory);
			// *p_txSize += NxpBootPacket_BuildFraming(p_txPacket, kFramingPacketType_Command, *p_txSize);
			// reqCode = PROTOCOL_REQ_CODE_PROCESS_COMPLETE;
			break;

		default:
			break;
	}

	return reqCode;
}

/******************************************************************************/
/*! Req Table */
/******************************************************************************/
#define REQ_SYNC_DEFAULT PROTOCOL_SYNC_ID_DEFINE(1U, 1U, 3U)

static const Protocol_Req_T REQ_TABLE[] =
{
	PROTOCOL_REQ_DEFINE(MOT_PACKET_PING, 				Req_Ping, 				0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_STOP_ALL, 			Req_StopAll, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_INIT_UNITS, 			Req_InitUnits, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_SAVE_NVM, 			Req_SaveNvm_Blocking, 	0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_WRITE_VAR, 			Req_WriteVar, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_READ_VAR, 			Req_ReadVar, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	// PROTOCOL_REQ_DEFINE(MOT_PACKET_WRITE_VARS_8, 		Req_WriteVars8, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	// PROTOCOL_REQ_DEFINE(MOT_PACKET_READ_VARS_16, 		Req_ReadVars16, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_MONITOR_TYPE, 		Req_Monitor, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_CONTROL_TYPE, 		Req_Control, 			0U, 	PROTOCOL_SYNC_ID_DISABLE),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_DATA_MODE_READ, 		0U, 	Req_ReadData, 				REQ_SYNC_DEFAULT),
	PROTOCOL_REQ_DEFINE(MOT_PACKET_DATA_MODE_WRITE, 	0U, 	Req_WriteData_Blocking, 	REQ_SYNC_DEFAULT),
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

	.RX_START_ID 		= MOT_PACKET_START_BYTE,
	.RX_END_ID 			= 0x00U,
	.ENCODED 			= false,

	//default
	.RX_TIMEOUT 		= MOT_PROTOCOL_TIMEOUT_MS,
	.REQ_TIMEOUT		= MOT_PROTOCOL_TIMEOUT_MS,
	.BAUD_RATE_DEFAULT 	= MOT_PROTOCOL_BAUD_RATE_DEFAULT,
};