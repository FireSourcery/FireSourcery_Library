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
	@file
	@author FireSourcery
	@brief	Text based UI.
			This modules calls Motor_User and MotorController_User functions only.
	@version V0
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE
#include "MotorController_Shell.h"
#include "MotorController_User.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


//todo User_Wrapper functions

/******************************************************************************/
/*!
	@brief 	Cmd Functions
 */
/*! @{ */
/******************************************************************************/
static Cmd_Status_T Cmd_monitor(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;
	if(argc == 1U)
	{
		p_mc->ShellSubstate = 0U;
	}
	else if(argc == 2U)
	{
		// if(strncmp(argv[1U], "a", 2U) == 0U)
		// {
		// 	p_mc->ShellSubstate = 1U;
		// }
		// else if(strncmp(argv[1U], "s", 2U) == 0U)
		// {
		// 	p_mc->ShellSubstate = 2U;
		// }
	}
	return CMD_STATUS_PROCESS_LOOP;
}

static Cmd_Status_T Cmd_monitor_Proc(MotorController_T * p_mc)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	switch(p_mc->ShellSubstate)
	{
		case 0U:
			Terminal_SendString(p_terminal, "Speed: ");	Terminal_SendNum(p_terminal, Motor_User_GetSpeed_Rpm(p_motor)); Terminal_SendString(p_terminal, " RPM ");
			Terminal_SendNum(p_terminal, Motor_User_GetSpeed_Frac16(p_motor)); Terminal_SendString(p_terminal, " Frac16\r\n");

			// Terminal_SendString(p_terminal, "Speed2: "); Terminal_SendNum(p_terminal, p_motor->Speed2_RPM); Terminal_SendString(p_terminal, " RPM ");
			// Terminal_SendNum(p_terminal, p_motor->Speed2_Frac16); Terminal_SendString(p_terminal, " Frac16\r\n");

//    		Terminal_SendString(p_terminal, "DeltaAngle: "); Terminal_SendNum(p_terminal, (uint16_t)p_motor->DeltaAngle); Terminal_SendString(p_terminal, " Frac16\r\n");

			Terminal_SendString(p_terminal, "Throttle: "); 	Terminal_SendNum(p_terminal, MotAnalogUser_GetThrottle(&p_mc->AnalogUser)); Terminal_SendString(p_terminal, " Frac16\r\n");
			Terminal_SendString(p_terminal, "Brake: "); 	Terminal_SendNum(p_terminal, MotAnalogUser_GetBrake(&p_mc->AnalogUser)); Terminal_SendString(p_terminal, " Frac16\r\n");
			Terminal_SendString(p_terminal, "RampCmd: "); 	Terminal_SendNum(p_terminal, p_motor->RampCmd); Terminal_SendString(p_terminal, " Q1.15\r\n");
			//			Terminal_SendString(p_terminal, ", RampIndex: "); Terminal_SendNum(p_terminal, p_motor->RampIndex);

			Terminal_SendString(p_terminal, "Iq: "); 	Terminal_SendNum(p_terminal, p_motor->Foc.Iq);
			Terminal_SendString(p_terminal, ", Vq: "); 	Terminal_SendNum(p_terminal, p_motor->Foc.Vq); Terminal_SendString(p_terminal, " Q1.15\r\n");

			Terminal_SendString(p_terminal, "Id: "); 	Terminal_SendNum(p_terminal, p_motor->Foc.Id);
			Terminal_SendString(p_terminal, ", Vd: "); 	Terminal_SendNum(p_terminal, p_motor->Foc.Vd); Terminal_SendString(p_terminal, " Q1.15\r\n");

        	Terminal_SendString(p_terminal, "IPhase: "); Terminal_SendNum(p_terminal, Motor_User_GetIPhase_Amp(p_motor)); Terminal_SendString(p_terminal, " Amp\r\n");

			// Terminal_SendString(p_terminal, "AnalogUserCmd: ");
			// switch(p_mc->AnalogUser.Cmd)
			// {
			// 	case MOT_ANALOG_USER_CMD_SET_BRAKE:					Terminal_SendString(p_terminal, "brake");			break;
			// 	case MOT_ANALOG_USER_CMD_SET_THROTTLE:				Terminal_SendString(p_terminal, "throttle");		break;
			// 	// case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:		Terminal_SendString(p_terminal, "brake rel");		break;
			// 	// case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:			Terminal_SendString(p_terminal, "throttle rel");	break;
			// 	// case MOT_ANALOG_USER_CMD_SET_NEUTRAL:				Terminal_SendString(p_terminal, "set neutral");		break;
			// 	case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:				Terminal_SendString(p_terminal, "neutral");			break;
			// 	// case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD: 	Terminal_SendString(p_terminal, "set forward");		break;
			// 	// case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE: 	Terminal_SendString(p_terminal, "set reverse");		break;
			// 	case MOT_ANALOG_USER_CMD_PROC_ZERO:				Terminal_SendString(p_terminal, "release");			break;
			// 	default: break;
			// }
			// Terminal_SendString(p_terminal, "\r\n");

			Terminal_SendString(p_terminal, "MSM: ");
			switch(Motor_User_GetStateId(p_motor))
			{
				case MSM_STATE_ID_INIT:			Terminal_SendString(p_terminal, "Init");		break;
				case MSM_STATE_ID_STOP:			Terminal_SendString(p_terminal, "Stop");		break;
				case MSM_STATE_ID_ALIGN:		Terminal_SendString(p_terminal, "Align");		break;
				case MSM_STATE_ID_OPEN_LOOP:	Terminal_SendString(p_terminal, "OpenLoop");	break;
				case MSM_STATE_ID_RUN:			Terminal_SendString(p_terminal, "Run");			break;
				case MSM_STATE_ID_FREEWHEEL:	Terminal_SendString(p_terminal, "Freewheel");	break;
				case MSM_STATE_ID_CALIBRATION:	Terminal_SendString(p_terminal, "Calib");		break;
				case MSM_STATE_ID_FAULT:		Terminal_SendString(p_terminal, "Fault");		break;
				default: break;
			}
			Terminal_SendString(p_terminal, "\r\n");

			Terminal_SendString(p_terminal, "MCSM: ");
			switch(MotorController_User_GetStateId(p_mc))
			{
				case MCSM_STATE_ID_INIT:	Terminal_SendString(p_terminal, "Init");	break;
				case MCSM_STATE_ID_STOP:	Terminal_SendString(p_terminal, "Stop");	break;
				case MCSM_STATE_ID_RUN:		Terminal_SendString(p_terminal, "Run");		break;
				case MCSM_STATE_ID_FAULT:	Terminal_SendString(p_terminal, "Fault");	break;
				default: break;
			}
			Terminal_SendString(p_terminal, "\r\n");

			// Terminal_SendString(p_terminal, "IPhasePeak_Adcu: "); Terminal_SendNum(p_terminal, p_motor->IPhasePeak_Adcu); Terminal_SendString(p_terminal, "\r\n");
			// Terminal_SendString(p_terminal, "ILimitActiveId: "); Terminal_SendNum(p_terminal, p_motor->ILimitActiveId); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendString(p_terminal, "ILimitActiveScalar: "); Terminal_SendNum(p_terminal, p_motor->ILimitActiveScalar); Terminal_SendString(p_terminal, "\r\n");
			// Terminal_SendString(p_terminal, "ILimitMotoring_Frac16: "); Terminal_SendNum(p_terminal, p_motor->ILimitMotoring_Frac16); Terminal_SendString(p_terminal, "\r\n");
			// Terminal_SendString(p_terminal, "VoltageModeILimitActive: "); Terminal_SendNum(p_terminal, p_motor->RunStateFlags.VoltageModeILimitActive); Terminal_SendString(p_terminal, "\r\n");

			Terminal_SendString(p_terminal, "SpeedPid Limit: "); Terminal_SendNum(p_terminal, p_motor->PidSpeed.OutputMax); Terminal_SendString(p_terminal, "\r\n");
			// Terminal_SendString(p_terminal, "ElecAngle: "); Terminal_SendNum(p_terminal, Motor_User_GetElectricalAngle(p_motor)); Terminal_SendString(p_terminal, " Deg16\r\n");
			// Terminal_SendString(p_terminal, "MechAngle: "); Terminal_SendNum(p_terminal, Motor_User_GetMechanicalAngle(p_motor)); Terminal_SendString(p_terminal, " Deg16\r\n");

			break;

		default: break;
	}

	Terminal_SendString(p_terminal, "\r\n");

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_mode(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	if(argc == 1U) /* display modes */
	{
		Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "Motor:\r\n");

		Terminal_SendString(p_terminal, "CommutationMode: ");
		switch(Motor_User_GetCommutationMode(p_motor))
		{
			case MOTOR_COMMUTATION_MODE_SIX_STEP: 	Terminal_SendString(p_terminal, "SIX_STEP"); 	break;
			case MOTOR_COMMUTATION_MODE_FOC: 		Terminal_SendString(p_terminal, "FOC"); 		break;
			default: break;
		}
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "SensorMode: ");
		switch(Motor_User_GetSensorMode(p_motor))
		{
			case MOTOR_SENSOR_MODE_HALL: 			Terminal_SendString(p_terminal, "HALL"); 		break;
			case MOTOR_SENSOR_MODE_ENCODER: 		Terminal_SendString(p_terminal, "ENCODER");		break;
			default: break;
		}
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "FeedbackMode: ");
		switch(Motor_User_GetFeedbackMode(p_motor))
		{
			case MOTOR_FEEDBACK_MODE_OPEN_LOOP: 				Terminal_SendString(p_terminal, "OPEN_LOOP"); 		break;
			case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE: 			Terminal_SendString(p_terminal, "VOLTAGE"); 		break;
			case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT: 			Terminal_SendString(p_terminal, "CURRENT"); 		break;
			case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE: 	Terminal_SendString(p_terminal, "SPEED_VOLTAGE"); 	break;
			case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT: 	Terminal_SendString(p_terminal, "SPEED_CURRENT"); 	break;
			default: break;
		}
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "DirectionCalibration: ");
		switch(Motor_User_GetDirectionCalibration(p_motor))
		{
			case MOTOR_FORWARD_IS_CW: 			Terminal_SendString(p_terminal, "CW"); 		break;
			case MOTOR_FORWARD_IS_CCW: 			Terminal_SendString(p_terminal, "CCW"); 	break;
			default: break;
		}
		Terminal_SendString(p_terminal, "\r\n");
	}
	else if(argc == 2U) /* sets mode */
	{
		if(strncmp(argv[1U], "foc", 4U) == 0U) { Motor_User_SetCommutationMode(p_motor, MOTOR_COMMUTATION_MODE_FOC); }
		else if(strncmp(argv[1U], "sixstep", 8U) == 0U) { Motor_User_SetCommutationMode(p_motor, MOTOR_COMMUTATION_MODE_SIX_STEP); }
		else if(strncmp(argv[1U], "voltage", 8U) == 0U) { Motor_User_SetFeedbackModeParam(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE); }
		else if(strncmp(argv[1U], "current", 8U) == 0U) { Motor_User_SetFeedbackModeParam(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT); }
		else if(strncmp(argv[1U], "speedv", 13U) == 0U) { Motor_User_SetFeedbackModeParam(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE); }
		else if(strncmp(argv[1U], "speedi", 12U) == 0U) { Motor_User_SetFeedbackModeParam(p_motor, MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT); }
		else if(strncmp(argv[1U], "fwdccw", 13U) == 0U) { Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW); }
		else if(strncmp(argv[1U], "fwdcw", 12U) == 0U) { Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW); }
		else if(strncmp(argv[1U], "protocol", 9U) == 0U)
		{
			Shell_DisableOnInit(&p_mc->Shell);
			Protocol_EnableOnInit(&p_mc->CONFIG.P_PROTOCOLS[0U]);
		}
		else if(strncmp(argv[1U], "brakevf", 7U) == 0U)
		{
			MotorController_User_SetBrakeMode(p_mc, MOTOR_CONTROLLER_BRAKE_MODE_VFREQ_SCALAR);
			Terminal_SendString(p_terminal, "VFreq Brake\r\n");
		}
		else if(strncmp(argv[1U], "brakei", 7U) == 0U)
		{
			MotorController_User_SetBrakeMode(p_mc, MOTOR_CONTROLLER_BRAKE_MODE_TORQUE);
			Terminal_SendString(p_terminal, "Torque Brake\r\n");
		}
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_run(MotorController_T * p_mc, int argc, char ** argv)
{
	char * p_end;
	uint32_t value;

	if(argc == 2U) /* run [num] */
	{
		value = strtoul(argv[1U], &p_end, 10);
		MotorController_User_SetCmdThrottle(p_mc, value);
	}
	if(argc == 3U) /* run [throttle/brake] [num] */
	{
		value = strtoul(argv[2U], &p_end, 10);
		if(strncmp(argv[1U], "throttle", 9U) == 0U) 	{ MotorController_User_SetCmdThrottle(p_mc, value); }
		else if(strncmp(argv[1U], "brake", 6U) == 0U) 	{ MotorController_User_SetCmdBrake(p_mc, value); }
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_stop(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	MotorController_User_DisableControl(p_mc);
	return CMD_STATUS_SUCCESS;
}

// void PrintPhase12(Terminal_T * p_terminal, uint8_t step)
// {
// 	uint16_t index = step % 12U;
// 	switch(index)
// 	{
// 		case 0U: Terminal_SendString(p_terminal, "Phase A: ");break;
// 		case 1U: Terminal_SendString(p_terminal, "Phase AC: "); break;
// 		case 2U: Terminal_SendString(p_terminal, "Phase -C: "); break;
// 		case 3U: Terminal_SendString(p_terminal, "Phase BC: "); break;
// 		case 4U: Terminal_SendString(p_terminal, "Phase B: "); break;
// 		case 5U: Terminal_SendString(p_terminal, "Phase BA: "); break;
// 		case 6U: Terminal_SendString(p_terminal, "Phase -A: "); break;
// 		case 7U: Terminal_SendString(p_terminal, "Phase CA: "); break;
// 		case 8U: Terminal_SendString(p_terminal, "Phase C: "); break;
// 		case 9U: Terminal_SendString(p_terminal, "Phase CB: "); break;
// 		case 10U: Terminal_SendString(p_terminal, "Phase -B: "); break;
// 		case 11U: Terminal_SendString(p_terminal, "Phase AB: "); break;
// 		default: break;
// 	}
// }

	//todo to string functions
static Cmd_Status_T Cmd_calibrate_Proc(MotorController_T * p_mc)
{
	Cmd_Status_T status = CMD_STATUS_PROCESS_LOOP;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	if(Motor_User_GetStateId(p_motor) == MSM_STATE_ID_STOP)
	{
		status = CMD_STATUS_PROCESS_END;

		if(p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_HALL)
		{
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[0U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[1U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[2U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[3U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[4U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[5U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[6U]); Terminal_SendString(p_terminal, "\r\n");
			Terminal_SendNum(p_terminal, p_motor->Hall.Params.SensorsTable[7U]); Terminal_SendString(p_terminal, "\r\n");
		}
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		else if(p_motor->CalibrationState == MOTOR_CALIBRATION_STATE_SIN_COS)
		{
			Terminal_SendString(p_terminal, "AngleOffset: "); 		Terminal_SendNum(p_terminal, p_motor->SinCos.Params.AngleOffet); 		Terminal_SendString(p_terminal, " \r\n");
			Terminal_SendString(p_terminal, "IsCcwPositive: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.Params.IsCcwPositive); 	Terminal_SendString(p_terminal, " \r\n");

			Terminal_SendString(p_terminal, "DebugAPreMech: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.DebugAPre); 		Terminal_SendString(p_terminal, " \r\n");
			Terminal_SendString(p_terminal, "DebugBPreMech: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.DebugBPre); 		Terminal_SendString(p_terminal, " \r\n");
			Terminal_SendString(p_terminal, "DebugAPostMech: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.DebugAPostMech); 	Terminal_SendString(p_terminal, " \r\n");
			Terminal_SendString(p_terminal, "DebugBPostMech: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.DebugBPostMech); 	Terminal_SendString(p_terminal, " \r\n");
			Terminal_SendString(p_terminal, "DebugAPostElec: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.DebugAPostElec); 	Terminal_SendString(p_terminal, " \r\n");
			Terminal_SendString(p_terminal, "DebugBPostElec: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.DebugBPostElec); 	Terminal_SendString(p_terminal, " \r\n");

			//			Terminal_SendString(p_terminal, "Zero_Adcu: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.Params.Zero_Adcu); 	Terminal_SendString(p_terminal, " \r\n");
			//			Terminal_SendString(p_terminal, "Max_Adcu: "); 		Terminal_SendNum(p_terminal, p_motor->SinCos.Params.Max_Adcu); 		Terminal_SendString(p_terminal, " \r\n");
			//			Terminal_SendString(p_terminal, "Max_MilliV: "); 	Terminal_SendNum(p_terminal, p_motor->SinCos.Params.Max_MilliV); 	Terminal_SendString(p_terminal, " \r\n");
			//			Terminal_SendString(p_terminal, "Phase 0: "); Terminal_SendNum(p_terminal, p_motor->Debug[9U]); Terminal_SendString(p_terminal, "Deg16 \r\n");
			//			Terminal_SendString(p_terminal, "Phase A: "); Terminal_SendNum(p_terminal, p_motor->Debug[0U]); Terminal_SendString(p_terminal, "Deg16 \r\n");
			//			Terminal_SendString(p_terminal, "Sin: "); Terminal_SendNum(p_terminal, p_motor->Debug[3U]);
			//			Terminal_SendString(p_terminal, "Cos: "); Terminal_SendNum(p_terminal, p_motor->Debug[4U]);
			//			Terminal_SendString(p_terminal, "\r\n");
			//			Terminal_SendString(p_terminal, "Phase B: "); Terminal_SendNum(p_terminal, p_motor->Debug[1U]); Terminal_SendString(p_terminal, "Deg16 \r\n");
			//			Terminal_SendString(p_terminal, "Sin: "); Terminal_SendNum(p_terminal, p_motor->Debug[5U]);
			//			Terminal_SendString(p_terminal, "Cos: "); Terminal_SendNum(p_terminal, p_motor->Debug[6U]);
			//			Terminal_SendString(p_terminal, "\r\n");
			//			Terminal_SendString(p_terminal, "Phase C: "); Terminal_SendNum(p_terminal, p_motor->Debug[2U]); Terminal_SendString(p_terminal, "Deg16 \r\n");
			//			Terminal_SendString(p_terminal, "Sin: "); Terminal_SendNum(p_terminal, p_motor->Debug[7U]);
			//			Terminal_SendString(p_terminal, "Cos: "); Terminal_SendNum(p_terminal, p_motor->Debug[8U]);
			Terminal_SendString(p_terminal, "\r\n");

		}
#endif
		Terminal_SendString(p_terminal, "\r\n");
	}

	return status;
}

static Cmd_Status_T Cmd_calibrate(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Cmd_Status_T status = CMD_STATUS_INVALID_ARGS;

	if(argc == 2U)
	{
		if(strncmp(argv[1U], "encoder", 8U) == 0U) 		{ Motor_User_ActivateCalibrationEncoder(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
		else if(strncmp(argv[1U], "hall", 5U) == 0U) 	{ Motor_User_ActivateCalibrationHall(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
		else if(strncmp(argv[1U], "adc", 4U) == 0U) 	{ Motor_User_ActivateCalibrationAdc(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
		else if(strncmp(argv[1U], "sincos", 7U) == 0U) 	{ Motor_User_ActivateCalibrationSinCos(p_motor); status = CMD_STATUS_PROCESS_LOOP; }
	}

	return status;
}

static Cmd_Status_T Cmd_hall(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	if(argc == 1U)
	{
		Hall_CaptureSensors_ISR(&p_motor->Hall);
		Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "Hall Sensors: "); Terminal_SendNum(p_terminal, Hall_GetSensorsId(&p_motor->Hall)); Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "Electrical Angle: "); Terminal_SendNum(p_terminal, Hall_GetRotorAngle_Degrees16(&p_motor->Hall)); Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "\r\n");
	}
	// else if(argc == 2U)
	// {
	// 	if(strncmp(argv[1U], "table", 5U) == 0U) {} //tostroing
	// }

	return CMD_STATUS_SUCCESS;
}

//seperate print functions
static Cmd_Status_T Cmd_heat(MotorController_T * p_mc, int argc, char ** argv)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	if(argc == 1U)
	{
		Terminal_SendString(p_terminal, "PCB: "); 			Terminal_SendNum(p_terminal, MotorController_User_GetHeatPcb_DegC(p_mc, 1U)); 			Terminal_SendString(p_terminal, " C\r\n");
		Terminal_SendString(p_terminal, "MOSFETs Top: "); 	Terminal_SendNum(p_terminal, MotorController_User_GetHeatMosfetsTop_DegC(p_mc, 1U)); 	Terminal_SendString(p_terminal, " C\r\n");
		Terminal_SendString(p_terminal, "MOSFETs Bot: "); 	Terminal_SendNum(p_terminal, MotorController_User_GetHeatMosfetsBot_DegC(p_mc, 1U)); 	Terminal_SendString(p_terminal, " C\r\n");
		Terminal_SendString(p_terminal, "Motor0: "); 		Terminal_SendNum(p_terminal, Motor_User_GetHeat_DegC(p_motor, 1U)); 					Terminal_SendString(p_terminal, " C\r\n");
		Terminal_SendString(p_terminal, "Motor0: "); 		Terminal_SendNum(p_terminal, p_motor->AnalogResults.Heat_Adcu); 						Terminal_SendString(p_terminal, " Adcu\r\n");
	}
	else if(argc == 2U)
	{
		if(strncmp(argv[1U], "limits", 7U) == 0U)
		{
			Terminal_SendString(p_terminal, "PCB: ");
			Terminal_SendString(p_terminal, " Shutdown: "); 	Terminal_SendNum(p_terminal, Thermistor_GetShutdown_DegCInt(&p_mc->ThermistorPcb, 1U));
			Terminal_SendString(p_terminal, " Threshold: "); 	Terminal_SendNum(p_terminal, Thermistor_GetShutdownThreshold_DegCInt(&p_mc->ThermistorPcb, 1U));
			Terminal_SendString(p_terminal, " Warning: "); 		Terminal_SendNum(p_terminal, Thermistor_GetWarning_DegCInt(&p_mc->ThermistorPcb, 1U));
			Terminal_SendString(p_terminal, " C\r\n");

			Terminal_SendString(p_terminal, "MOSFETs Top: ");
			Terminal_SendString(p_terminal, " Shutdown: "); 	Terminal_SendNum(p_terminal, Thermistor_GetShutdown_DegCInt(&p_mc->ThermistorMosfetsTop, 1U));
			Terminal_SendString(p_terminal, " Threshold: "); 	Terminal_SendNum(p_terminal, Thermistor_GetShutdownThreshold_DegCInt(&p_mc->ThermistorMosfetsTop, 1U));
			Terminal_SendString(p_terminal, " Warning: "); 		Terminal_SendNum(p_terminal, Thermistor_GetWarning_DegCInt(&p_mc->ThermistorMosfetsTop, 1U));
			Terminal_SendString(p_terminal, " C\r\n");

			// Terminal_SendString(p_terminal, "MOSFETs Bot: "); 	Terminal_SendNum(p_terminal, heatMosfetsBot); 	Terminal_SendString(p_terminal, " C\r\n");
			// Terminal_SendString(p_terminal, "Motor0: "); 		Terminal_SendNum(p_terminal, heatMotor0); 		Terminal_SendString(p_terminal, " C\r\n");
			Terminal_SendString(p_terminal, "\r\n");

		}
	}

	return CMD_STATUS_SUCCESS;
}

//MotorController_String.c
// static const char * STR_VPOS 		= "V Supply: \r\n";
// static const char * STR_VSENSE 		= "V Sensor: \r\n";
// static const char * STR_VACC 		= "V Accessories: \r\n";

// static size_t MotorController_ToString_VMonitorsLimits(MotorController_T * p_mc, char * p_stringBuffer)
// {
// 	char * p_stringDest = p_stringBuffer;

// 	memcpy(p_stringDest, STR_VPOS, strlen(STR_VPOS)); 		p_stringDest += strlen(STR_VPOS);
// 	p_stringDest += VMonitor_ToString_Verbose(&p_mc->VMonitorPos, p_stringDest, 1000U);
// 	memcpy(p_stringDest, "\r\n", 2U); 	p_stringDest += 2U;

// 	memcpy(p_stringDest, STR_VSENSE, strlen(STR_VSENSE)); 	p_stringDest += strlen(STR_VSENSE);
// 	p_stringDest += VMonitor_ToString_Verbose(&p_mc->VMonitorSense, p_stringDest, 1000U);
// 	memcpy(p_stringDest, "\r\n", 2U); 	p_stringDest += 2U;

// 	memcpy(p_stringDest, STR_VACC, strlen(STR_VACC)); 		p_stringDest += strlen(STR_VACC);
// 	p_stringDest += VMonitor_ToString_Verbose(&p_mc->VMonitorAcc, p_stringDest, 1000U);
// 	memcpy(p_stringDest, "\r\n", 2U); 	p_stringDest += 2U;

// 	return p_stringDest - p_stringBuffer;
// }

static Cmd_Status_T Cmd_v(MotorController_T * p_mc, int argc, char ** argv)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	// char * p_txString;
	// uint8_t txSize = 0U;
	int32_t vSen, vAcc, vPos, battery;

	if(argc == 1U)
	{
		//todo to string
		vPos = MotorController_User_GetVPos(p_mc, 1000U);
		vSen = MotorController_User_GetVSense(p_mc, 1000U);
		vAcc = MotorController_User_GetVAcc(p_mc, 1000U);
		battery = MotorController_User_GetBatteryCharge_Unit1000(p_mc);
		Terminal_SendString(p_terminal, "VPos: "); Terminal_SendNum(p_terminal, vPos); Terminal_SendString(p_terminal, " mV\r\n");
		Terminal_SendString(p_terminal, "VSen: "); Terminal_SendNum(p_terminal, vSen); Terminal_SendString(p_terminal, " mV\r\n");
		Terminal_SendString(p_terminal, "VAcc: "); Terminal_SendNum(p_terminal, vAcc); Terminal_SendString(p_terminal, " mV\r\n");
		Terminal_SendString(p_terminal, "Battery: "); Terminal_SendNum(p_terminal, battery); Terminal_SendString(p_terminal, " 1000th\r\n");
		Terminal_SendString(p_terminal, "\r\n");
	}
	else if(argc == 2U)
	{
		if(strncmp(argv[1U], "limits", 7U) == 0U)
		{
		// 	p_txString = (char *)Terminal_AcquireTxBuffer(p_terminal);
		// 	txSize = MotorController_ToString_VMonitorsLimits(p_mc, p_txString);
		// 	Terminal_ReleaseTxBuffer(p_terminal, txSize);
		}
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_fault(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	if(argc == 1U)
	{
		MotorController_User_ToggleUserFault(p_mc);

		// Terminal_SendString(p_terminal, "FaultFlags [VPos][VAcc][VSense][Pcb][MosTop][MosBot]: ");
		// Terminal_SendNum(p_terminal, p_mc->FaultFlags.VPosLimit);
		// Terminal_SendNum(p_terminal, p_mc->FaultFlags.VAccLimit);
		// Terminal_SendNum(p_terminal, p_mc->FaultFlags.VSenseLimit);
		// Terminal_SendNum(p_terminal, p_mc->FaultFlags.PcbOverHeat);
		// Terminal_SendNum(p_terminal, p_mc->FaultFlags.MosfetsTopOverHeat);
		// Terminal_SendNum(p_terminal, p_mc->FaultFlags.MosfetsBotOverHeat);
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "FaultAdcu:\r\n");
		// Terminal_SendString(p_terminal, "VPos: "); 		Terminal_SendNum(p_terminal, p_mc->FaultAnalogRecord.VPos_Adcu); 			Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "VAcc: "); 		Terminal_SendNum(p_terminal, p_mc->FaultAnalogRecord.VAcc_Adcu); 			Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "VSense: "); 	Terminal_SendNum(p_terminal, p_mc->FaultAnalogRecord.VSense_Adcu); 			Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "Pcb: "); 		Terminal_SendNum(p_terminal, p_mc->FaultAnalogRecord.HeatPcb_Adcu); 		Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "MosTop: "); 	Terminal_SendNum(p_terminal, p_mc->FaultAnalogRecord.HeatMosfetsTop_Adcu); 	Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "MosBot: "); 	Terminal_SendNum(p_terminal, p_mc->FaultAnalogRecord.HeatMosfetsBot_Adcu); 	Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "Fault DegC:\r\n");
		// Terminal_SendString(p_terminal, "Pcb: "); 		Terminal_SendNum(p_terminal, Thermistor_ConvertToDegC_Int(&p_mc->ThermistorPcb, p_mc->FaultAnalogRecord.HeatPcb_Adcu, 1U)); 				Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "MosTop: "); 	Terminal_SendNum(p_terminal, Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsTop, p_mc->FaultAnalogRecord.HeatMosfetsTop_Adcu, 1U)); 	Terminal_SendString(p_terminal, "\r\n");
		// Terminal_SendString(p_terminal, "MosBot: "); 	Terminal_SendNum(p_terminal, Thermistor_ConvertToDegC_Int(&p_mc->ThermistorMosfetsBot, p_mc->FaultAnalogRecord.HeatMosfetsBot_Adcu, 1U)); 	Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "\r\n");
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_direction(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	if(argc == 2U)
	{
		if(strncmp(argv[1U], "fwd", 2U) == 0U) 			{ MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD); }
		else if(strncmp(argv[1U], "rev", 2U) == 0U) 	{ MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE); }
	}
	if(argc == 3U)
	{
		if(strncmp(argv[1U], "calfwd", 4U) == 0U)
		{
			if(strncmp(argv[2U], "ccw", 4U) == 0U) 		{ Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW); }
			else if(strncmp(argv[2U], "cw", 3U) == 0U) 	{ Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW); }
		}
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_phase(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	//	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;

	if(argc == 2U)
	{
		if(argv[1U][0U] == 'a' && argv[1U][1U] == 'b') { Phase_Polar_ActivateAB(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'a' && argv[1U][1U] == 'c') { Phase_Polar_ActivateAC(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'a' && argv[1U][1U] == '\0') { Phase_Polar_ActivateA(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'a' && argv[1U][1U] == '-') { Phase_Polar_ActivateInvA(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'b' && argv[1U][1U] == 'a') { Phase_Polar_ActivateBA(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'b' && argv[1U][1U] == 'c') { Phase_Polar_ActivateBC(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'b' && argv[1U][1U] == '\0') { Phase_Polar_ActivateB(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'b' && argv[1U][1U] == '-') { Phase_Polar_ActivateInvB(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'c' && argv[1U][1U] == 'a') { Phase_Polar_ActivateCA(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'c' && argv[1U][1U] == 'b') { Phase_Polar_ActivateCB(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'c' && argv[1U][1U] == '\0') { Phase_Polar_ActivateC(&p_motor->Phase, duty); }
		else if(argv[1U][0U] == 'c' && argv[1U][1U] == '-') { Phase_Polar_ActivateInvC(&p_motor->Phase, duty); }

		//print sensor info
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_jog(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Cmd_Status_T status;
	if(argc == 1U)
	{
		Motor_Jog6(p_motor);
		status = CMD_STATUS_SUCCESS;
	}
	else if(argc == 2U)
	{
		//		Motor_Jog (p_motor, strtoul(argv[1U], 0U, 10));
		//		return CMD_STATUS_PROCESS_LOOP;
	}

	return status;
}

static Cmd_Status_T Cmd_rev(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	p_motor->JogIndex = 0U;

	switch(Motor_User_GetSensorMode(p_motor))
	{
		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_HALL:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_Zero(&p_motor->Encoder);
			break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_SENSOR_MODE_SIN_COS:

#endif
		default:
			break;
	}

	return CMD_STATUS_PROCESS_LOOP;
}

static Cmd_Status_T Cmd_rev_Proc(MotorController_T * p_mc)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Cmd_Status_T status;

	if(p_motor->JogIndex < Motor_User_GetPolePairs(p_motor) * 6U + 1U)
	{
		switch(Motor_User_GetSensorMode(p_motor))
		{
			case MOTOR_SENSOR_MODE_SENSORLESS:
				break;

			case MOTOR_SENSOR_MODE_HALL:
				if(p_motor->JogIndex % 6U == 0U)
				{
					Terminal_SendString(p_terminal, "\r\n");
					Terminal_SendString(p_terminal, "AngularD: ");
					Terminal_SendNum(p_terminal, Encoder_GetAngularD(&p_motor->Encoder));
					Terminal_SendString(p_terminal, "\r\n");
					Terminal_SendString(p_terminal, "Hall: ");
				}
				Terminal_SendNum(p_terminal, Hall_ReadSensors(&p_motor->Hall).State); Terminal_SendString(p_terminal, " ");
				break;

			case MOTOR_SENSOR_MODE_ENCODER:
				Terminal_SendNum(p_terminal, Encoder_GetAngularD(&p_motor->Encoder));
				Terminal_SendString(p_terminal, ", ");
				Terminal_SendNum(p_terminal, Encoder_GetAngle(&p_motor->Encoder));
				Terminal_SendString(p_terminal, "\r\n");
				break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
			case MOTOR_SENSOR_MODE_SIN_COS:
				SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);

				Terminal_SendString(p_terminal, "Sin: "); 		Terminal_SendNum(p_terminal, p_motor->AnalogResults.Sin_Adcu);
				Terminal_SendString(p_terminal, " Cos: "); 		Terminal_SendNum(p_terminal, p_motor->AnalogResults.Cos_Adcu);
				Terminal_SendString(p_terminal, " Angle: "); 	Terminal_SendNum(p_terminal, SinCos_GetElectricalAngle(&p_motor->SinCos));
				Terminal_SendString(p_terminal, "\r\n");
				break;
#endif
			default:
				break;
		}

		Motor_Jog6(p_motor);
		status = CMD_STATUS_SUCCESS;
	}
	else
	{
		Terminal_SendString(p_terminal, "\r\n");
		MotorController_User_DisableControl(p_mc);
		status = CMD_STATUS_PROCESS_END;
	}

	return status;
}

static Cmd_Status_T Cmd_set(MotorController_T * p_mc, int argc, char ** argv)
{
	//	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	if(argc == 2U)
	{
		if(strncmp(argv[1U], "default", 8U) == 0U)
		{
			MotorController_User_SetLoadDefault(p_mc, true);
			MotorController_User_SaveBootReg_Blocking(p_mc);
			Terminal_SendString(p_terminal, "Need to reboot\r\n");
		}
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_save(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	MotorController_User_SaveParameters_Blocking(p_mc);
	return CMD_STATUS_SUCCESS;
}

extern void SystemSoftwareReset(void); //todo abstraction layer

static Cmd_Status_T Cmd_reboot(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)p_mc;
	(void)argc;
	(void)argv;
	// System_Reboot();
	// SystemSoftwareReset();
	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_beep(MotorController_T * p_mc, int argc, char ** argv)
{
	uint8_t count;

	if(argc == 1U)
	{
		MotorController_User_BeepN(p_mc, 500U, 500U, 1U);
	}
	else if(argc == 2U)
	{
		if(strncmp(argv[1U], "stop", 5U) == 0U) 		{ MotorController_User_BeepStop(p_mc); }
		else if(strncmp(argv[1U], "start", 6U) == 0U) 	{ MotorController_User_BeepStart(p_mc, 500U, 500U); }
		else
		{
			count = strtoul(argv[1U], 0U, 10);
			MotorController_User_BeepN(p_mc, 500U, 500U, count);
		}
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_vmatch(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	uint16_t rpm;

	if(argc == 2U)
	{
		rpm = strtoul(argv[1U], 0U, 10);
		Motor_User_SetSpeedVMatchRef_Rpm(p_motor, rpm);
		rpm = Motor_User_GetSpeedVMatchRef_Rpm(p_motor);

		Terminal_SendString(p_terminal, "\r\nSpeedVMatch_Rpm: "); Terminal_SendNum(p_terminal, rpm);
		Terminal_SendString(p_terminal, " PWM: "); Terminal_SendNum(p_terminal, Motor_User_ConvertToVMatchFrac16(p_motor, rpm));
		Terminal_SendString(p_terminal, " Frac16\r\n");

		Terminal_SendString(p_terminal, "RPM: "); Terminal_SendNum(p_terminal, rpm*3U/4U);
		Terminal_SendString(p_terminal, " PWM: "); Terminal_SendNum(p_terminal, Motor_User_ConvertToVMatchFrac16(p_motor, rpm*3U/4U));
		Terminal_SendString(p_terminal, " Frac16\r\n");

		Terminal_SendString(p_terminal, "RPM: "); Terminal_SendNum(p_terminal, rpm/2U);
		Terminal_SendString(p_terminal, " PWM: "); Terminal_SendNum(p_terminal, Motor_User_ConvertToVMatchFrac16(p_motor, rpm/2U));
		Terminal_SendString(p_terminal, " Frac16\r\n");

 		Terminal_SendString(p_terminal, "RPM: "); Terminal_SendNum(p_terminal, rpm*1U/4U);
		Terminal_SendString(p_terminal, " PWM: "); Terminal_SendNum(p_terminal, Motor_User_ConvertToVMatchFrac16(p_motor, rpm*1U/4U));
		Terminal_SendString(p_terminal, " Frac16\r\n");
	}

	return CMD_STATUS_SUCCESS;
}

static void PrintIPeak(Terminal_T * p_terminal, uint16_t min_Adcu, int32_t min_Frac16, uint16_t max_Adcu, int32_t max_Frac16)
{
	Terminal_SendString(p_terminal, "Min: ");
	Terminal_SendNum(p_terminal, min_Adcu); Terminal_SendString(p_terminal, " ADCU ");
	Terminal_SendNum(p_terminal, min_Frac16); Terminal_SendString(p_terminal, " Frac16 ");
	Terminal_SendString(p_terminal, "Max: ");
	Terminal_SendNum(p_terminal, max_Adcu); Terminal_SendString(p_terminal, " ADCU ");
	Terminal_SendNum(p_terminal, max_Frac16); Terminal_SendString(p_terminal, " Frac16\r\n");
}

static Cmd_Status_T Cmd_ipeak(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	uint16_t zeroToPeak_Adcu;
	uint16_t min_Adcu;
	uint16_t max_Adcu;

	if(argc == 2U)
	{
		zeroToPeak_Adcu = strtoul(argv[1U], 0U, 10);
		Motor_User_SetIPeakRef_Adcu(p_motor, zeroToPeak_Adcu);

		Terminal_SendString(p_terminal, "Phase A:\r\n");
		min_Adcu = p_motor->Parameters.IaZeroRef_Adcu - p_motor->Parameters.IPeakRef_Adcu;
		max_Adcu = p_motor->Parameters.IaZeroRef_Adcu + p_motor->Parameters.IPeakRef_Adcu;
		PrintIPeak(p_terminal, min_Adcu, Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, min_Adcu), max_Adcu, Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, max_Adcu));

		Terminal_SendString(p_terminal, "Phase B:\r\n");
		min_Adcu = p_motor->Parameters.IbZeroRef_Adcu - p_motor->Parameters.IPeakRef_Adcu;
		max_Adcu = p_motor->Parameters.IbZeroRef_Adcu + p_motor->Parameters.IPeakRef_Adcu;
		PrintIPeak(p_terminal, min_Adcu, Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, min_Adcu), max_Adcu, Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, max_Adcu));

		Terminal_SendString(p_terminal, "Phase C:\r\n");
		min_Adcu = p_motor->Parameters.IcZeroRef_Adcu - p_motor->Parameters.IPeakRef_Adcu;
		max_Adcu = p_motor->Parameters.IcZeroRef_Adcu + p_motor->Parameters.IPeakRef_Adcu;
		PrintIPeak(p_terminal, min_Adcu, Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, min_Adcu), max_Adcu, Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, max_Adcu));

		Terminal_SendString(p_terminal, "\r\n");
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_ilimit(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	uint16_t ilimit_Frac16;

	if(argc == 2U)
	{
		ilimit_Frac16 = strtoul(argv[1U], 0U, 10);
		Motor_User_SetILimitParam_Frac16(p_motor, ilimit_Frac16, ilimit_Frac16);
		Terminal_SendString(p_terminal, "ILimitMotoringParam: "); Terminal_SendNum(p_terminal, p_motor->Parameters.ILimitMotoring_Frac16);
		Terminal_SendString(p_terminal, " ILimitMotoringActive: "); Terminal_SendNum(p_terminal, p_motor->ILimitMotoring_Frac16);
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "PidSpeed Min, Max: ");
		Terminal_SendNum(p_terminal, p_motor->PidSpeed.OutputMin);		Terminal_SendString(p_terminal, " ");
		Terminal_SendNum(p_terminal, p_motor->PidSpeed.OutputMax);		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "PidIq Min, Max: ");
		Terminal_SendNum(p_terminal, p_motor->PidIq.OutputMin);		Terminal_SendString(p_terminal, " ");
		Terminal_SendNum(p_terminal, p_motor->PidIq.OutputMax);		Terminal_SendString(p_terminal, "\r\n");
	}

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_version(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	Terminal_SendString(p_terminal, "Library: ");
	Terminal_SendNum(p_terminal, MotorController_User_GetLibraryVersionIndex(3U)); Terminal_SendString(p_terminal, " ");
	Terminal_SendNum(p_terminal, MotorController_User_GetLibraryVersionIndex(2U)); Terminal_SendString(p_terminal, " ");
	Terminal_SendNum(p_terminal, MotorController_User_GetLibraryVersionIndex(1U)); Terminal_SendString(p_terminal, " ");
	Terminal_SendNum(p_terminal, MotorController_User_GetLibraryVersionIndex(0U)); Terminal_SendString(p_terminal, "\r\n");

	Terminal_SendString(p_terminal, "Firmware: ");
	Terminal_SendNum(p_terminal, MotorController_User_GetMainVersionIndex(p_mc, 3U)); Terminal_SendString(p_terminal, " ");
	Terminal_SendNum(p_terminal, MotorController_User_GetMainVersionIndex(p_mc, 2U)); Terminal_SendString(p_terminal, " ");
	Terminal_SendNum(p_terminal, MotorController_User_GetMainVersionIndex(p_mc, 1U)); Terminal_SendString(p_terminal, " ");
	Terminal_SendNum(p_terminal, MotorController_User_GetMainVersionIndex(p_mc, 0U)); Terminal_SendString(p_terminal, "\r\n");

	return CMD_STATUS_SUCCESS;
}

static Cmd_Status_T Cmd_debug(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	// const MotorController_Manufacture_T TEST =
	// {
	// 	.NAME = {'q', 'b', 'c', 'd'},
	// 	.MANUFACTURE_NUMBER_REG = 123U,
	// };

	// const MotorController_Manufacture_T TEST1 =
	// {
	// 	.NAME = {'x', 'y', 'z', 'z'},
	// 	.MANUFACTURE_NUMBER_REG = 9877U,
	// };

	// MotorController_Manufacture_T TestRead = {0};

	// Terminal_SendString(p_terminal, "Manufacture:\r\n");
	// MotorController_User_GetManufacture(p_mc, &TestRead);
	// Terminal_SendString_Len(p_terminal, TestRead.NAME, 8U); Terminal_SendString(p_terminal, " \r\n");
	// Terminal_SendNum(p_terminal, TestRead.MANUFACTURE_NUMBER_REG); Terminal_SendString(p_terminal, " \r\n");

	// Terminal_SendString(p_terminal, "write1:\r\n");
	// MotorController_User_WriteManufacture(p_mc, &TEST);
	// Terminal_SendNum(p_terminal, p_mc->NvmStatus);Terminal_SendString(p_terminal, " \r\n");
	// MotorController_ReadOnce_Blocking(p_mc);
	// MotorController_User_GetManufacture(p_mc, &TestRead);
	// Terminal_SendString_Len(p_terminal, TestRead.NAME, 8U); Terminal_SendString(p_terminal, " \r\n");
	// Terminal_SendNum(p_terminal, TestRead.MANUFACTURE_NUMBER_REG); Terminal_SendString(p_terminal, " \r\n");

	// Terminal_SendString(p_terminal, "write2:\r\n");
	// MotorController_User_WriteManufacture(p_mc, &TEST1);
	// Terminal_SendNum(p_terminal, p_mc->NvmStatus);Terminal_SendString(p_terminal, " \r\n");
	// MotorController_ReadOnce_Blocking(p_mc);
	// MotorController_User_GetManufacture(p_mc, &TestRead);
	// Terminal_SendString_Len(p_terminal, TestRead.NAME, 8U); Terminal_SendString(p_terminal, " \r\n");
	// Terminal_SendNum(p_terminal, TestRead.MANUFACTURE_NUMBER_REG); Terminal_SendString(p_terminal, " \r\n");

	// qfrac16_t atan2_0 = qfrac16_atan2(0, 32767);
	// qfrac16_t atan2_90 = qfrac16_atan2(32767, 0);
	// qfrac16_t atan2_180 = qfrac16_atan2(0, -32767);
	// qfrac16_t atan2_270 = qfrac16_atan2(-32767, 0);
	// qfrac16_t atan2_err = qfrac16_atan2(32767, 32767);

	// Terminal_SendString(p_terminal, "Atan2: "); Terminal_SendString(p_terminal, "\r\n");
	// Terminal_SendNum(p_terminal, atan2_0); Terminal_SendString(p_terminal, "\r\n");
	// Terminal_SendNum(p_terminal, atan2_90); Terminal_SendString(p_terminal, "\r\n");
	// Terminal_SendNum(p_terminal, atan2_180); Terminal_SendString(p_terminal, "\r\n");
	// Terminal_SendNum(p_terminal, atan2_270); Terminal_SendString(p_terminal, "\r\n");
	// Terminal_SendNum(p_terminal, atan2_err); Terminal_SendString(p_terminal, "\r\n");

		// Terminal_SendString(p_terminal, "PwmOn "); Terminal_SendNum(p_terminal, sampleCount); Terminal_SendString(p_terminal, " :\r\n");
		// for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
		// {
		// 	Terminal_SendNum(p_terminal, p_samples[iSample]); Terminal_SendString(p_terminal, ", ");
		// }
		// Terminal_SendString(p_terminal, "\r\n");

		// Terminal_SendString(p_terminal, "PwmOff "); Terminal_SendNum(p_terminal, sampleCount); Terminal_SendString(p_terminal, " :\r\n");
		// for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
		// {
		// 	Terminal_SendNum(p_terminal, p_samples[iSample]); Terminal_SendString(p_terminal, ", ");
		// }
		// Terminal_SendString(p_terminal, "\r\n");

	Terminal_SendString(p_terminal, "\r\n");
	return CMD_STATUS_SUCCESS;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

const Cmd_T MC_CMD_TABLE[MC_SHELL_CMD_COUNT] =
{
	{"monitor", 	"Display motor info",				(Cmd_Function_T)Cmd_monitor, 	{.FUNCTION = (Cmd_ProcessFunction_T)Cmd_monitor_Proc, 	.FREQ = 5U}		},
	{"heat", 		"Display temperatures",				(Cmd_Function_T)Cmd_heat, 		{0U}	},
	{"v", 			"Display voltages", 				(Cmd_Function_T)Cmd_v, 			{0U}	},
	{"hall", 		"Read hall", 						(Cmd_Function_T)Cmd_hall, 		{0U}	},
	{"fault", 		"Display fault info", 				(Cmd_Function_T)Cmd_fault, 		{0U}	},
	{"mode", 		"Sets mode using options",			(Cmd_Function_T)Cmd_mode, 		{0U}	},

	{"direction", 	"Sets direction",					(Cmd_Function_T)Cmd_direction,	{0U}	},
	{"calibrate", 	"calibrate",						(Cmd_Function_T)Cmd_calibrate, 	{.FUNCTION = (Cmd_ProcessFunction_T)Cmd_calibrate_Proc,	.FREQ = 10U}	},

	{"set", 		"Sets motor parameters",	 		(Cmd_Function_T)Cmd_set, 		{0U}	},
	{"save", 		"Save parameters to nv memory", 	(Cmd_Function_T)Cmd_save, 		{0U}	},
	{"reboot", 		"reboot", 							(Cmd_Function_T)Cmd_reboot, 	{0U}	},

	{"run", 		"Set motor to run mode", 			(Cmd_Function_T)Cmd_run, 		{0U}	},
	{"stop", 		"Set motor to freewheel mode", 		(Cmd_Function_T)Cmd_stop, 		{0U}	},
	{"phase", 		"Sets motor phase", 				(Cmd_Function_T)Cmd_phase, 		{0U}	},
	{"jog", 		"Jog motor", 						(Cmd_Function_T)Cmd_jog, 		{0U}	},
	{"rev", 		"Rev motor",	 					(Cmd_Function_T)Cmd_rev, 		{.FUNCTION = (Cmd_ProcessFunction_T)Cmd_rev_Proc, 		.FREQ = 10U}	},
	{"beep", 		"beep",								(Cmd_Function_T)Cmd_beep, 		{0U}	},
	{"vmatch", 		"vmatch",							(Cmd_Function_T)Cmd_vmatch, 	{0U}	},

	{"ipeak", 		"ipeak",							(Cmd_Function_T)Cmd_ipeak, 		{0U}	},
	{"ilimit", 		"ilimit",							(Cmd_Function_T)Cmd_ilimit, 	{0U}	},

	{"version", 	"version",							(Cmd_Function_T)Cmd_version, 	{0U}	},
	{"debug", 		"print debug info",					(Cmd_Function_T)Cmd_debug, 		{0U}	},

	// {"analoguser", 	"analoguser enable/disable", 		(Cmd_Function_T)Cmd_analoguser,	{0U}	},
};

#endif

// static Cmd_Status_T Cmd_analoguser(MotorController_T * p_mc, int argc, char ** argv)
// {
// 	if(argc == 2U)
// 	{
// 		if(strncmp(argv[1U], "disable", 8U) == 0U)
// 		{
// 			p_mc->Parameters.UserInputMode = MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL; //tdo
// 		}
// 		else if(strncmp(argv[1U], "enable", 7U) == 0U)
// 		{
// 			p_mc->Parameters.UserInputMode = MOTOR_CONTROLLER_INPUT_MODE_ANALOG;
// 		}
// 	}

// 	return CMD_STATUS_SUCCESS;
// }

//static Cmd_Status_T Cmd_configfile(int argc, char **argv)
//{
//
//	return MC_SHELL_CMD_RETURN_CODE_SUCCESS;
//}
