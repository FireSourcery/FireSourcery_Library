
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
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/******************************************************************************/
#include "MotorController_Shell.h"
#include "MotorController_User.h"
#include "Motor/Motor/Motor_User.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern void SystemSoftwareReset(void);

/******************************************************************************/
/*!
	@brief 	Cmd Functions
 */
/*! @{ */
/******************************************************************************/
static int Cmd_monitor(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {

    }

	return CMD_RESERVED_RETURN_CODE_LOOP;
}

static int Cmd_monitor_Proc(MotorController_T * p_mc)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

	uint32_t throttle = MotorController_User_GetThrottle(p_mc);
	uint32_t brake = MotorController_User_GetBrake(p_mc);

//    	for(uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
//    	{
        	Terminal_SendString(p_terminal, "Speed: ");
    		Terminal_SendNum(p_terminal, p_motor->Speed_RPM);
    		Terminal_SendString(p_terminal, " RPM ");
    		Terminal_SendNum(p_terminal, p_motor->Speed_Frac16);
    		Terminal_SendString(p_terminal, " Frac16\r\n");

			Terminal_SendString(p_terminal, "Throttle: ");
			Terminal_SendNum(p_terminal, throttle);
			Terminal_SendString(p_terminal, " Frac16\r\n");

			Terminal_SendString(p_terminal, "Brake: ");
			Terminal_SendNum(p_terminal, brake);
			Terminal_SendString(p_terminal, " Frac16\r\n");

			Terminal_SendString(p_terminal, "RampCmd: ");
			Terminal_SendNum(p_terminal, p_motor->RampCmd);
//			Terminal_SendString(p_terminal, ", RampIndex: ");
//			Terminal_SendNum(p_terminal, p_motor->RampIndex);
			Terminal_SendString(p_terminal, " Frac16\r\n");

        	Terminal_SendString(p_terminal, "Iq: ");
    		Terminal_SendNum(p_terminal, p_motor->Foc.Iq);
        	Terminal_SendString(p_terminal, ",	Vq: ");
    		Terminal_SendNum(p_terminal, p_motor->Foc.Vq);
    		Terminal_SendString(p_terminal, " Q1.15\r\n");

			Terminal_SendString(p_terminal, "Id: ");
			Terminal_SendNum(p_terminal, p_motor->Foc.Id);
			Terminal_SendString(p_terminal, ",	Vd: ");
			Terminal_SendNum(p_terminal, p_motor->Foc.Vd);
			Terminal_SendString(p_terminal, " Q1.15\r\n");

			Terminal_SendString(p_terminal, "AnalogUserCmd: ");

			switch(p_mc->AnalogUserCmd)
			{
				case MOT_ANALOG_USER_CMD_SET_BRAKE:					Terminal_SendString(p_terminal, "brake");			break;
				case MOT_ANALOG_USER_CMD_SET_THROTTLE:				Terminal_SendString(p_terminal, "throttle");		break;
				case MOT_ANALOG_USER_CMD_SET_NEUTRAL:				Terminal_SendString(p_terminal, "set neutral");		break;
				case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:				Terminal_SendString(p_terminal, "neutral");			break;
				case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD: 	Terminal_SendString(p_terminal, "set f");			break;
				case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE: 	Terminal_SendString(p_terminal, "set r");			break;
				case MOT_ANALOG_USER_CMD_SET_RELEASE:				Terminal_SendString(p_terminal, "set release");		break;
				case MOT_ANALOG_USER_CMD_PROC_RELEASE:				Terminal_SendString(p_terminal, "release");			break;
				default: break;
			}

//			Terminal_SendString(p_terminal, "\r\n");

//			Terminal_SendString(p_terminal, "Motor Direction: ");
//			Terminal_SendNum(p_terminal, p_motor->Direction);
//			Terminal_SendString(p_terminal, ",	User: ");
//			Terminal_SendNum(p_terminal, p_motor->UserDirection);
//			Terminal_SendString(p_terminal, "\r\n");
//			Terminal_SendString(p_terminal, "MC Direction: ");
//			Terminal_SendNum(p_terminal, p_mc->MainDirection);
//			Terminal_SendString(p_terminal, ",	User: ");
//			Terminal_SendNum(p_terminal, p_mc->UserDirection);
//			Terminal_SendString(p_terminal, "\r\n");
//
//			Terminal_SendString(p_terminal, "MC State Machine: ");
//			switch(MotorController_StateMachine_GetStateId(p_mc))
//			{
//				case MCSM_STATE_ID_INIT:	Terminal_SendString(p_terminal, "Init");	break;
//				case MCSM_STATE_ID_STOP:	Terminal_SendString(p_terminal, "Stop");	break;
//				case MCSM_STATE_ID_RUN:		Terminal_SendString(p_terminal, "Run");		break;
//				case MCSM_STATE_ID_FAULT:	Terminal_SendString(p_terminal, "Fault");	break;
//				default: break;
//			}
			Terminal_SendString(p_terminal, "\r\n");

//        	Terminal_SendString(p_terminal, "Iabc: ");
//    		Terminal_SendNum(p_terminal, p_motor->Foc.Ia);
//    		Terminal_SendString(p_terminal, " ");
//       	Terminal_SendNum(p_terminal, p_motor->Foc.Ib);
//       	Terminal_SendString(p_terminal, " ");
//       	Terminal_SendNum(p_terminal, p_motor->Foc.Ic);
//    		Terminal_SendString(p_terminal, " Q1.15\r\n");
//
//        	Terminal_SendString(p_terminal, "Iclarke: ");
//    		Terminal_SendNum(p_terminal, p_motor->Foc.Ialpha);
//    		Terminal_SendString(p_terminal, " ");
//       		Terminal_SendNum(p_terminal, p_motor->Foc.Ibeta);
//    		Terminal_SendString(p_terminal, " Q1.15\r\n");
//
//			Terminal_SendString(p_terminal, "ElAngle: ");
//			Terminal_SendNum(p_terminal, p_motor->ElectricalAngle);
//			Terminal_SendString(p_terminal, " Deg16\r\n");
//
//        	Terminal_SendString(p_terminal, "SinCos: ");
//    		Terminal_SendNum(p_terminal, p_motor->Foc.Sine);
//    		Terminal_SendString(p_terminal, " ");
//       		Terminal_SendNum(p_terminal, p_motor->Foc.Cosine);
//    		Terminal_SendString(p_terminal, " Q1.15\r\n");


//        	Terminal_SendString(p_terminal, "IBus: ");
//    		Terminal_SendNum(p_terminal, p_motor->IBus_Frac16);
//    		Terminal_SendString(p_terminal, " Frac16\r\n");

//        	Terminal_SendString(p_terminal, "VBemf_On: ");
//    		Terminal_SendNum(p_terminal, p_motor->Bemf.VPhase_ADCU);
//    		Terminal_SendString(p_terminal, " ADCU\r\n");
//
//        	Terminal_SendString(p_terminal, "VBemf_Off: ");
//    		Terminal_SendNum(p_terminal, p_motor->Bemf.VPhasePwmOff_ADCU);
//    		Terminal_SendString(p_terminal, " ADCU\r\n");

//        	Terminal_SendString(p_terminal, "PwmOnTime: ");
//    		Terminal_SendNum(p_terminal, p_motor->PwmOnTime);
//    		Terminal_SendString(p_terminal, " \r\n");

//        	Terminal_SendString(p_terminal, "ZC Period: ");
//    		Terminal_SendNum(p_terminal, p_motor->Bemf.TimeZeroCrossingPeriod);
//    		Terminal_SendString(p_terminal, " 20 kHz\r\n");
//
//        	Terminal_SendString(p_terminal, "Phase Period: ");
//    		Terminal_SendNum(p_terminal, p_motor->Bemf.TimePhasePeriod);
//    		Terminal_SendString(p_terminal, " 20 kHz\r\n");
//
//        	Terminal_SendString(p_terminal, "Encoder Capture Time: ");
//    		Terminal_SendNum(p_terminal, p_motor->Encoder.DeltaT);
//    		Terminal_SendString(p_terminal, " 625 kHz\r\n");


    		Terminal_SendString(p_terminal, "\r\n");

//    	}


	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}




static int Cmd_mode(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

    if(argc == 1U)
    {

    }
    else if(argc == 2U)
    {
    	if(strncmp(argv[1U], "foc", 4U) == 0U)
    	{
    		p_motor->Parameters.CommutationMode = MOTOR_COMMUTATION_MODE_FOC;
    	}
    	else if(strncmp(argv[1U], "sixstep", 5U) == 0U)
    	{
    		p_motor->Parameters.CommutationMode = MOTOR_COMMUTATION_MODE_SIX_STEP;
    	}
    	else if(strncmp(argv[1U], "voltage", 8U) == 0U)
    	{
    		Motor_User_SetThrottleControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE);
    	}
    	else if(strncmp(argv[1U], "current", 8U) == 0U)
    	{
    		Motor_User_SetThrottleControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_CURRENT);
    	}
    	else if(strncmp(argv[1U], "speedvoltage", 13U) == 0U)
    	{
    		Motor_User_SetThrottleControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE);
    	}
    	else if(strncmp(argv[1U], "speedcurrent", 12U) == 0U)
    	{
    		Motor_User_SetThrottleControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT);
    	}
    	else if(strncmp(argv[1U], "protocol", 9U) == 0U)
    	{
    		p_mc->Shell.Params.IsEnable = false;
    		p_mc->CONFIG.P_PROTOCOLS[0U].Params.IsEnable = true;

//							.PROTOCOLS[0U] =
//							{
//								.p_Xcvr 	= &Serials[1U],
//								.p_Specs	= &ETS_SPECS,
//								.IsEnable 	= false,
//							},
//
//							.SHELL =
//							{
//								.p_Xcvr 	= &Serials[1U],
//								.BaudRate 	= 19200U,
//								.IsEnable 	= true,
//							},
    	}

    }
    else if(argc == 3U)
    {
    	if(strncmp(argv[1U], "dircalib", 9U) == 0U)
		{
			if(strncmp(argv[2U], "ccw", 4U) == 0U)
			{
				Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW);
			}
			else if(strncmp(argv[2U], "cw", 3U) == 0U)
			{
				Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW);
			}
		}
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_analoguser(MotorController_T * p_mc, int argc, char ** argv)
{
	if(argc == 2U)
	{
		if(strncmp(argv[1U], "disable", 8U) == 0U)
		{
			p_mc->Parameters.InputMode = MOTOR_CONTROLLER_INPUT_MODE_SERIAL;
		}
		else if(strncmp(argv[1U], "enable", 7U) == 0U)
		{
			p_mc->Parameters.InputMode = MOTOR_CONTROLLER_INPUT_MODE_ANALOG;
		}
	}

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_run(MotorController_T * p_mc, int argc, char ** argv)
{
	char * end;
	uint32_t value;

	if(argc == 2U)
	{
		value = strtoul(argv[1U], &end, 10);
		MotorController_User_SetCmdThrottle(p_mc, value);
	}
	if(argc == 3U)
	{
		value = strtoul(argv[2U], &end, 10);

    	if(strncmp(argv[1U], "throttle", 9U) == 0U)
    	{
    		MotorController_User_SetCmdThrottle(p_mc, value);
    	}
    	else if (strncmp(argv[1U], "brake", 6U) == 0U)
		{
    		MotorController_User_SetCmdBrake(p_mc, value);
		}
	}

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_stop(MotorController_T * p_mc, int argc, char ** argv)
{
	uint32_t value;

	if(argc == 1U)
	{
		MotorController_User_DisableControl(p_mc);
	}

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_calibrate(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

    if(argc == 1U)
    {

    }
    else if(argc == 2U)
    {
    	if(strncmp(argv[1U], "encoder", 8U) == 0U)
    	{
    		Motor_User_ActivateCalibrationEncoder(p_motor);
    	}
    	else if(strncmp(argv[1U], "hall", 5U) == 0U)
    	{
    		Motor_User_ActivateCalibrationHall(p_motor);
    	}
    	else if(strncmp(argv[1U], "adc", 4U) == 0U)
    	{
    		Motor_User_ActivateCalibrationAdc(p_motor);
    	}
//    	else if(strncmp(argv[1U], "hallprint", 10U) == 0U)
//    	{
//
//        	Terminal_SendString(p_terminal, "\r\n");
//        	Terminal_SendString(p_terminal, "Phase A: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[1]);
//        	Terminal_SendString(p_terminal, "	");
//
//        	Terminal_SendString(p_terminal, "  Phase AC: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[2]);
//        	Terminal_SendString(p_terminal, "\r\n");
//
//        	Terminal_SendString(p_terminal, "Phase -C: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[3]);
//        	Terminal_SendString(p_terminal, "	");
//
//        	Terminal_SendString(p_terminal, "  Phase BC: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[4]);
//        	Terminal_SendString(p_terminal, "\r\n");
//
//        	Terminal_SendString(p_terminal, "Phase B: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[5]);
//        	Terminal_SendString(p_terminal, "	");
//
//        	Terminal_SendString(p_terminal, "  Phase BA: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[6]);
//        	Terminal_SendString(p_terminal, "\r\n");
//
//        	Terminal_SendString(p_terminal, "Phase -A: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[7]);
//        	Terminal_SendString(p_terminal, "	");
//
//        	Terminal_SendString(p_terminal, "  Phase CA: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[8]);
//        	Terminal_SendString(p_terminal, "\r\n");
//
//        	Terminal_SendString(p_terminal, "Phase C: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[9]);
//        	Terminal_SendString(p_terminal, "	");
//
//        	Terminal_SendString(p_terminal, "  Phase CB: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[10]);
//        	Terminal_SendString(p_terminal, "\r\n");
//
//        	Terminal_SendString(p_terminal, "Phase -B: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[11]);
//        	Terminal_SendString(p_terminal, "	");
//
//        	Terminal_SendString(p_terminal, "  Phase AB: ");
//    		Terminal_SendNum(p_terminal, p_motor->HallDebug[12]);
//        	Terminal_SendString(p_terminal, "\r\n");
//    	}
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_save(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	MotorController_User_SaveParameters_Blocking(p_mc);
    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_reboot(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argc;
	(void)argv;
	SystemSoftwareReset();
    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_phase(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
    if(argc == 2)
    {
    	if 		(argv[1][0] == 'a' && argv[1][1] == 'b') Phase_Polar_ActivateAB(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'a' && argv[1][1] == 'c') Phase_Polar_ActivateAC(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'a' && argv[1][1] == '\0') Phase_Polar_ActivateA(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'a' && argv[1][1] == '-') Phase_Polar_ActivateInvA(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'b' && argv[1][1] == 'a') Phase_Polar_ActivateBA(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'b' && argv[1][1] == 'c') Phase_Polar_ActivateBC(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'b' && argv[1][1] == '\0') Phase_Polar_ActivateB(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'b' && argv[1][1] == '-') Phase_Polar_ActivateInvB(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'c' && argv[1][1] == 'a') Phase_Polar_ActivateCA(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'c' && argv[1][1] == 'b') Phase_Polar_ActivateCB(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'c' && argv[1][1] == '\0') Phase_Polar_ActivateC(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);
    	else if (argv[1][0] == 'c' && argv[1][1] == '-') Phase_Polar_ActivateInvC(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16);

    	Terminal_SendString(p_terminal, "\r\n");
    	Terminal_SendString(p_terminal, "Hall: ");
		Terminal_SendNum(p_terminal, Hall_ReadSensors(&p_motor->Hall).State);
    	Terminal_SendString(p_terminal, "\r\n");
    }

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_hall(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;

    if(argc == 1U)
    {
    	Terminal_SendString(p_terminal, "\r\n");
    	Terminal_SendString(p_terminal, "Hall: ");
		Terminal_SendNum(p_terminal, Hall_ReadSensors(&p_motor->Hall).State);

//    	Terminal_SendString(p_terminal, "Electrical Angle: ");
//		Terminal_SendNum(p_terminal, Motor_User_GetHallRotorAngle(p_motor ));

    	Terminal_SendString(p_terminal, "\r\n");
    }

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}




static int Cmd_heat(MotorController_T * p_mc, int argc, char ** argv)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	int32_t heat 		= MotorController_User_GetHeatPcb_DegCInt(p_mc, 1U);
	int32_t limit 		= MotorController_User_GetHeatPcbLimit_DegCInt(p_mc, 1U);
	int32_t threshold 	= MotorController_User_GetHeatPcbThreshold_DegCInt(p_mc, 1U);

	Terminal_SendString(p_terminal, "\r\n");
	Terminal_SendString(p_terminal, "Pcb: ");
	Terminal_SendNum(p_terminal, heat);
	Terminal_SendString(p_terminal, " C");

	Terminal_SendString(p_terminal, "	Limit: ");
	Terminal_SendNum(p_terminal, limit);

	Terminal_SendString(p_terminal, "	Threshold: ");
	Terminal_SendNum(p_terminal, threshold);
	Terminal_SendString(p_terminal, "\r\n");

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_vmonitor(MotorController_T * p_mc, int argc, char ** argv)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	int32_t vSense 		= MotorController_User_GetVSense_MilliV(p_mc);
	int32_t vAcc 		= MotorController_User_GetVAcc_MilliV(p_mc);
	int32_t vPos 		= MotorController_User_GetVPos_MilliV(p_mc);
	int32_t battery 	= MotorController_User_GetBatteryCharge_Base10(p_mc);

	Terminal_SendString(p_terminal, "\r\n");
	Terminal_SendString(p_terminal, "VSense: ");
	Terminal_SendNum(p_terminal, vSense);
	Terminal_SendString(p_terminal, " mV");
	Terminal_SendString(p_terminal, "\r\n");

	Terminal_SendString(p_terminal, "VAcc: ");
	Terminal_SendNum(p_terminal, vAcc);
	Terminal_SendString(p_terminal, " mV");
	Terminal_SendString(p_terminal, "\r\n");

	Terminal_SendString(p_terminal, "VPos: ");
	Terminal_SendNum(p_terminal, vPos);
	Terminal_SendString(p_terminal, " mV");
	Terminal_SendString(p_terminal, "\r\n");

	Terminal_SendString(p_terminal, "Battery Charge: ");
	Terminal_SendNum(p_terminal, battery);
	Terminal_SendString(p_terminal, " Percent10");
	Terminal_SendString(p_terminal, "\r\n");

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_fault(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;

	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	int32_t vSense 		= MotorController_User_GetVSenseFault_MilliV(p_mc);
	int32_t vAcc 		= MotorController_User_GetVAccFault_MilliV(p_mc);
	int32_t vPos 		= MotorController_User_GetVPosFault_MilliV(p_mc);
//	int32_t battery 	= MotorController_GetBatteryCharge_Base10(p_mc);
	int32_t heat 		= MotorController_User_GetHeatPcbFault_DegCInt(p_mc, 1U);

	if(argc == 1U)
    {
    	Terminal_SendString(p_terminal, "ErrorFlags [VPos][VAcc][VSense][Pcb][MosfetT][MostfetB]: ");
    	Terminal_SendNum(p_terminal, p_mc->ErrorFlags.VPosLimit);
    	Terminal_SendNum(p_terminal, p_mc->ErrorFlags.VAccLimit);
    	Terminal_SendNum(p_terminal, p_mc->ErrorFlags.VSenseLimit);
    	Terminal_SendNum(p_terminal, p_mc->ErrorFlags.PcbOverHeat);
    	Terminal_SendNum(p_terminal, p_mc->ErrorFlags.MosfetsTopOverHeat);
    	Terminal_SendNum(p_terminal, p_mc->ErrorFlags.MosfetsBotOverHeat);
    	Terminal_SendString(p_terminal, "\r\n");

    	Terminal_SendString(p_terminal, "Fault: \r\n");

    	Terminal_SendString(p_terminal, "Pcb: ");
    	Terminal_SendNum(p_terminal, heat);
    	Terminal_SendString(p_terminal, " C");
    	Terminal_SendString(p_terminal, "\r\n");

    	Terminal_SendString(p_terminal, "VSense: ");
    	Terminal_SendNum(p_terminal, vSense);
    	Terminal_SendString(p_terminal, " mV");
    	Terminal_SendString(p_terminal, "\r\n");

    	Terminal_SendString(p_terminal, "VAcc: ");
    	Terminal_SendNum(p_terminal, vAcc);
    	Terminal_SendString(p_terminal, " mV");
    	Terminal_SendString(p_terminal, "\r\n");

    	Terminal_SendString(p_terminal, "VPos: ");
    	Terminal_SendNum(p_terminal, vPos);
    	Terminal_SendString(p_terminal, " mV");
    	Terminal_SendString(p_terminal, "\r\n");

    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

int Cmd_direction(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	if(argc == 2U)
	{
		if(strncmp(argv[1U], "f", 2U) == 0U)
		{
			MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);
		}
		else if(strncmp(argv[1U], "r", 2U) == 0U)
		{
			MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);
		}
	}
	if(argc == 3U)
	{
		if(strncmp(argv[1U], "cal", 4U) == 0U)
		{
			if(strncmp(argv[2U], "ccw", 4U) == 0U)
			{
				Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW);
			}
			else if(strncmp(argv[2U], "cw", 3U) == 0U)
			{
				Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW);
			}
		}
//		else if()
//		{
//
//		}
	}

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

int Cmd_set(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	if(argc == 3U)
	{
		if(strncmp(argv[1U], "dircalib", 9U) == 0U)
		{
			if(strncmp(argv[2U], "ccw", 4U) == 0U)
			{
				Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CCW);
			}
			else if(strncmp(argv[2U], "cw", 3U) == 0U)
			{
				Motor_User_SetDirectionCalibration(p_motor, MOTOR_FORWARD_IS_CW);
			}
		}
//		else if()
//		{
//
//		}
	}

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int PrintHeat( )
{

}

//#define PARAM_STR(PARAM_STR)  #PARAM_STR


#define PRINT_PARAM_STR(PARAM_STR)  				\
	Terminal_SendString(p_terminal, #PARAM_STR); 	\
	Terminal_SendString(p_terminal, ":	");


#define PRINT_PARAM_VAR_MOTOR(PARAM_STR) \
		PRINT_PARAM_STR(PARAM_STR) \
		Terminal_SendNum(p_terminal, p_motor->P##arameters.PARAM_STR); 	\
		Terminal_SendString(p_terminal, "\r\n");

#define PRINT_PARAM_VAR_MC(PARAM_STR) \
		PRINT_PARAM_STR(PARAM_STR) \
		Terminal_SendNum(p_terminal, p_mc->P##arameters.PARAM_STR); 	\
		Terminal_SendString(p_terminal, "\r\n");

static int Cmd_params(MotorController_T * p_mc, int argc, char ** argv)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	if(argc == 1U)
	{
		Terminal_SendString(p_terminal, "\r\n");
		Terminal_SendString(p_terminal, "Motor:\r\n");

		Terminal_SendString(p_terminal, "CommutationMode: ");
		switch(p_motor->Parameters.CommutationMode)
		{
			case MOTOR_COMMUTATION_MODE_SIX_STEP: 	Terminal_SendString(p_terminal, "SIX_STEP"); 	break;
			case MOTOR_COMMUTATION_MODE_FOC: 		Terminal_SendString(p_terminal, "FOC"); 		break;
			default : break;
		}
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "SensorMode: ");
		switch(p_motor->Parameters.SensorMode)
		{
			case MOTOR_SENSOR_MODE_HALL: 			Terminal_SendString(p_terminal, "HALL"); 	break;
			case MOTOR_SENSOR_MODE_ENCODER: 		Terminal_SendString(p_terminal, "ENCODER"); 		break;
			default : break;
		}
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "ControlMode: ");
		switch(p_motor->Parameters.ControlMode)
		{
			case MOTOR_CONTROL_MODE_OPEN_LOOP: 					Terminal_SendString(p_terminal, "OPEN_LOOP"); 		break;
			case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE: 			Terminal_SendString(p_terminal, "VOLTAGE"); 		break;
			case MOTOR_CONTROL_MODE_CONSTANT_CURRENT: 			Terminal_SendString(p_terminal, "CURRENT"); 		break;
			case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE: 	Terminal_SendString(p_terminal, "SPEED_VOLTAGE"); 	break;
			case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT: 	Terminal_SendString(p_terminal, "SPEED_CURRENT"); 	break;
			default : break;
		}
		Terminal_SendString(p_terminal, "\r\n");

		Terminal_SendString(p_terminal, "DirectionCalibration: ");
		switch(p_motor->Parameters.DirectionCalibration)
		{
			case MOTOR_FORWARD_IS_CW: 			Terminal_SendString(p_terminal, "CW"); 		break;
			case MOTOR_FORWARD_IS_CCW: 			Terminal_SendString(p_terminal, "CCW"); 	break;
			default : break;
		}
		Terminal_SendString(p_terminal, "\r\n");

//		Terminal_SendString(p_terminal, "SpeedRefMax_RPM: ");
//		Terminal_SendNum(p_terminal, p_motor->Parameters.SpeedRefMax_RPM);
//		Terminal_SendString(p_terminal, "\r\n");


		PRINT_PARAM_VAR_MOTOR(SpeedRefMax_RPM)
		PRINT_PARAM_VAR_MOTOR(SpeedRefVoltage_RPM)
		PRINT_PARAM_VAR_MOTOR(IBusLimit_Frac16)
		PRINT_PARAM_VAR_MOTOR(IqLimit)
		PRINT_PARAM_VAR_MOTOR(IRefMax_Amp)
		PRINT_PARAM_VAR_MOTOR(IaRefMax_ADCU)
		PRINT_PARAM_VAR_MOTOR(IbRefMax_ADCU)
		PRINT_PARAM_VAR_MOTOR(IcRefMax_ADCU)
		PRINT_PARAM_VAR_MOTOR(IaRefZero_ADCU)
		PRINT_PARAM_VAR_MOTOR(IbRefZero_ADCU)
		PRINT_PARAM_VAR_MOTOR(IcRefZero_ADCU)
		PRINT_PARAM_VAR_MOTOR(AlignVoltage_Frac16)
		PRINT_PARAM_VAR_MOTOR(AlignTime_ControlCycles)
//
//		PRINT_PARAM(p_motor->Encoder.Params, CountsPerRevolution)
//		PRINT_PARAM(p_motor->Encoder.Params, DistancePerCount)
//		PRINT_PARAM(p_motor->Encoder.Params, IsQuadratureCaptureEnabled)
//		PRINT_PARAM(p_motor->Encoder.Params, IsALeadBPositive)
//		PRINT_PARAM(p_motor->Encoder.Params, ExtendedTimerDeltaTStop)
//		PRINT_PARAM(p_motor->Encoder.Params, MotorPolePairs)
//


//
//		PRINT_PARAM(p_mc, Params.MotorPolePairs)
//		.MOTOR_CONTROLLER =
//		{
//
//			.InputMode 					= MOTOR_CONTROLLER_INPUT_MODE_ANALOG,
//			.CoastMode 					= MOTOR_CONTROLLER_COAST_MODE_REGEN, //MOTOR_CONTROLLER_COAST_MODE_FLOAT,
//			.IsBuzzerOnReverseEnable 	= false,
//			.BatteryZero_ADCU = 1293, //30V
//			.BatteryFull_ADCU = 1811, //42V
//		},
//
//		.ANALOG_USER =
//		{
//			.ThrottleZero_ADCU 	= 0U,
//			.ThrottleMax_ADCU 	= 4095U,
//			.BrakeZero_ADCU		= 0U,
//			.BrakeMax_ADCU		= 4095U,
//			.EnablePinThrottle 	= true,
//			.EnablePinBrake		= true,
//			.EnablePinNeutral 	= false,
//		},
//
//		.PROTOCOLS[0U] =
//		{
//			.p_Xcvr 	= &Serials[1U],
//			.p_Specs	= &ETS_SPECS,
//			.IsEnable 	= false,
//		},
//
//		.SHELL =
//		{
//			.p_Xcvr 	= &Serials[1U],
//			.BaudRate 	= 19200U,
//			.IsEnable 	= true,
//		},
	}


//	/*
//	 * Input Speed Q0.16
//	 * Output SpeedControl => VPwm, Vq, Iq,
//	 */
//	.PID_SPEED[0U] =
//	{
//		.CalcFreq = 1000U,
//		.Mode = PID_MODE_PI,
//		.Direction = PID_DIRECTION_DIRECT,
//
//		.KpFactor = 1,
//		.KpDivisor = 2,
//		.KiFactor = 1,
//		.KiDivisor = 2,
//		.KdFactor = 0,
//		.KdDivisor = 0,
//
//		.OutMin = 0,
//		.OutMax = 65535,
//	},
//
//	/*
//	 * Input  Iq
//	 * Output Vq, sign indicates direction
//	 */
//	.PID_FOC_IQ[0U] =
//	{
//		.CalcFreq = 20000U,
//		.Mode = PID_MODE_PI,
//		.Direction = PID_DIRECTION_DIRECT,
//
//		.KpFactor = 1,
//		.KpDivisor = 2,
//		.KiFactor = 1,
//		.KiDivisor = 1,
//		.KdFactor = 0,
//		.KdDivisor = 0,
//
//		.OutMin = -32767,
//		.OutMax = 32767,
//	},
//
//	.PID_FOC_ID[0U] =
//	{
//		.CalcFreq = 20000U,
//		.Mode = PID_MODE_PI,
//		.Direction = PID_DIRECTION_DIRECT,
//
//		.KpFactor = 1,
//		.KpDivisor = 2,
//		.KiFactor = 1,
//		.KiDivisor = 2,
//		.KdFactor = 0,
//		.KdDivisor = 0,
//
//		.OutMin = -32768/2,
//		.OutMax = 32768/2,
//	},
//
//	.PID_SIX_STEP_IBUS[0U] =
//	{
//		.CalcFreq = 20000U,
//		.Mode = PID_MODE_PI,
//		.Direction = PID_DIRECTION_DIRECT,
//
//		.KpFactor = 1,
//		.KpDivisor = 2,
//		.KiFactor = 1,
//		.KiDivisor = 2,
//		.KdFactor = 0,
//		.KdDivisor = 0,
//
//		.OutMin = 0,
//		.OutMax = 65535,
//	},
//
//	.THERMISTOR_MOTORS[0U] =
//	{
//		.RNominal = 100000U,
//		.TNominal = 298U,
//		.BConstant = 3950U,
//
//		.Limit_ADCU = 0,
//		.Threshold_ADCU = 0,
//
//		.IntScalar = 1U,
//		.IsEnable = false,
//	},



    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_debug(MotorController_T * p_mc, int argc, char ** argv)
{
//    	for(uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
//    	{


//	Terminal_SendString(p_terminal, "PwmOn ");
//	Terminal_SendNum(p_terminal, sampleCount);
//	Terminal_SendString(p_terminal, " :\r\n");
//
//	for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
//	{
//		Terminal_SendNum(p_terminal, p_samples[iSample]);
//		Terminal_SendString(p_terminal, ", ");
//	}
//	Terminal_SendString(p_terminal, "\r\n");
//
//
//	Terminal_SendString(p_terminal, "PwmOff ");
//	Terminal_SendNum(p_terminal, sampleCount);
//	Terminal_SendString(p_terminal, " :\r\n");
//
//	for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
//	{
//		Terminal_SendNum(p_terminal, p_samples[iSample]);
//		Terminal_SendString(p_terminal, ", ");
//	}
//	Terminal_SendString(p_terminal, "\r\n");

//    	}


	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


//
//
//int Cmd_configfile(int argc, char **argv)
//{
//
//	return MC_SHELL_CMD_RETURN_CODE_SUCCESS;
//}



//int Cmd_jog(MotorController_T * p_mc, int argc, char ** argv)
//{
//	if(argc == 1)
//	{
//		Motor_SetJogSteps(&Motor1, 1);
//
//	}
//	else if(argc == 2) Motor_SetJogSteps(&Motor1, strtoul(argv[1], 0, 10));
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}


//
//int Cmd_hold(MotorController_T * p_mc, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Hold(&Motor1);
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}
//
//int Cmd_release(MotorController_T * p_mc, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Release(&Motor1);
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}


int Cmd_rev_Proc(MotorController_T * p_mc)
{

//	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
//
//	if (UserAngle <=  65536*7)
//	{
//		Motor_FOC_ActivateAngle(p_motor, (uint16_t)UserAngle, p_motor->Parameters.AlignVoltage_Frac16);
//		Encoder_DeltaD_Capture(&p_motor->Encoder);
//
//		Terminal_SendNum(p_terminal, p_motor->Encoder.AngularD);
//		Terminal_SendString(p_terminal, " ");
//		Terminal_SendNum(p_terminal, UserAngle);
//		Terminal_SendString(p_terminal, "\r\n");
//
//		UserAngle += 1024;
//	}
//
//
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


int Cmd_rev(MotorController_T * p_mc, int argc, char ** argv)
{
//	(void)argv;
//	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);
//
//	//motor ramp does not proc during off state
//	Motor_FOC_StartAngleControl(MotorController_User_GetPtrMotor(p_mc, 0U));
//	UserAngle = 0U;
////	p_motor->RampCmd = p_motor->Parameters.AlignVoltage_Frac16;
//
//	Terminal_SendString(p_terminal, "\r\n");
//
//    return CMD_RESERVED_RETURN_CODE_LOOP;
}


const Cmd_T MC_CMD_TABLE[MC_SHELL_CMD_COUNT] =
{
	{"monitor", 	"Display motor info",				(Cmd_Function_T)Cmd_monitor, 	{.FUNCTION = (Cmd_ProcessFunction_T)Cmd_monitor_Proc, 	.FREQ = 5U}	},
	{"hall", 		"Read hall", 						(Cmd_Function_T)Cmd_hall, 		{0U}		},
	{"heat", 		"Display temperatures",				(Cmd_Function_T)Cmd_heat, 		{0U}		},
	{"vmonitor", 	"Display voltages", 				(Cmd_Function_T)Cmd_vmonitor, 	{0U}		},
	{"fault", 	"Display fault info", 						(Cmd_Function_T)Cmd_fault, 	{0U}		},

	{"params", 	" ", 						(Cmd_Function_T)Cmd_params, 	{0U}		},

	{"mode", 		"Sets mode using options",			(Cmd_Function_T)Cmd_mode, 		{0U}	},

	{"analoguser", 	"analoguser enable/disable", 		(Cmd_Function_T)Cmd_analoguser, 		{0U}	},

	{"direction", 	"Sets direction",						(Cmd_Function_T)Cmd_direction, 		{0U}	},


	{"calibrate", 	"calibrate",						(Cmd_Function_T)Cmd_calibrate, 	{0U}	},
	{"run", 		"Set motor to run mode", 			(Cmd_Function_T)Cmd_run, 		{0U}	},
	{"stop", 		"Set motor to freewheel mode", 		(Cmd_Function_T)Cmd_stop, 		{0U}	},
	{"set", 		"Sets motor parameters",	 		(Cmd_Function_T)Cmd_set, 		{0U}	},
	{"save", 		"Save parameters to nv memory", 	(Cmd_Function_T)Cmd_save, 		{0U}	},
	{"reboot", 		"reboot", 							(Cmd_Function_T)Cmd_reboot, 	{0U}	},

	{"rev", 		"rev",	 							(Cmd_Function_T)Cmd_rev, 		{.FUNCTION = (Cmd_ProcessFunction_T)Cmd_rev_Proc, 		.FREQ = 10U} },
	{"phase", 		"Sets motor phase", 				(Cmd_Function_T)Cmd_phase, 		{0U}		},
	{"debug", 		"debug",							(Cmd_Function_T)Cmd_debug, 		{0U}	},
////	{ "jog", 		"Jog motor", 				Cmd_jog		},
////	{ "float", 		"Float motor", 				Cmd_float	},
////	{ "hold", 		"hold position", 			Cmd_hold	},
////	{ "print", 		"print debug info",			Cmd_print	},
};


const Cmd_Status_T MC_CMD_STATUS_TABLE[MC_SHELL_CMD_STATUS_COUNT] =
{

};

/******************************************************************************/
/*! @} */
/******************************************************************************/

