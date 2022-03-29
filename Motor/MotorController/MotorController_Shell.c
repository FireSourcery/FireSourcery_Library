
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
#include "MotorController.h"
#include "MotorController_User.h"

#include "Motor/Motor/Motor_FOC.h"
#include "Motor/Motor/Motor_User.h"

#include "Utility/Shell/Shell.h"
#include "Utility/Shell/Terminal.h"
#include "Utility/Shell/Cmd.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

static int Cmd_monitor_Loop(MotorController_T * p_mc)
{
//    	for(uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
//    	{


        	Terminal_SendString(&p_mc->Shell.Terminal, "Speed = ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Speed_RPM);
    		Terminal_SendString(&p_mc->Shell.Terminal, " RPM\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Speed = ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Speed_Frac16);
    		Terminal_SendString(&p_mc->Shell.Terminal, " Frac16\r\n");

			Terminal_SendString(&p_mc->Shell.Terminal, "RampCmd = ");
			Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].RampCmd);
			Terminal_SendString(&p_mc->Shell.Terminal, " Frac16\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Iq = ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Iq);
        	Terminal_SendString(&p_mc->Shell.Terminal, ", Vq = ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Vq);
    		Terminal_SendString(&p_mc->Shell.Terminal, " Q1.15\r\n");

			Terminal_SendString(&p_mc->Shell.Terminal, "Id = ");
			Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Id);
			Terminal_SendString(&p_mc->Shell.Terminal, ", Vd = ");
			Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Vd);
			Terminal_SendString(&p_mc->Shell.Terminal, " Q1.15\r\n");

//        	Terminal_SendString(&p_mc->Shell.Terminal, "Iabc = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Ia);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " ");
//       		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Ib);
//       		Terminal_SendString(&p_mc->Shell.Terminal, " ");
//       		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Ic);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " Q1.15\r\n");
//
//        	Terminal_SendString(&p_mc->Shell.Terminal, "Iclarke = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Ialpha);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " ");
//       		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Ibeta);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " Q1.15\r\n");
//
//			Terminal_SendString(&p_mc->Shell.Terminal, "ElAngle = ");
//			Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].ElectricalAngle);
//			Terminal_SendString(&p_mc->Shell.Terminal, " Deg16\r\n");
//
//        	Terminal_SendString(&p_mc->Shell.Terminal, "SinCos = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Sine);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " ");
//       		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Foc.Cosine);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " Q1.15\r\n");


//        	Terminal_SendString(&p_mc->Shell.Terminal, "IBus = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].IBus_Frac16);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " Frac16\r\n");

//        	Terminal_SendString(&p_mc->Shell.Terminal, "VBemf_On = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Bemf.VPhase_ADCU);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " ADCU\r\n");
//
//        	Terminal_SendString(&p_mc->Shell.Terminal, "VBemf_Off = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Bemf.VPhasePwmOff_ADCU);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " ADCU\r\n");

//        	Terminal_SendString(&p_mc->Shell.Terminal, "PwmOnTime = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].PwmOnTime);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " \r\n");

//        	Terminal_SendString(&p_mc->Shell.Terminal, "ZC Period = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Bemf.TimeZeroCrossingPeriod);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " 20 kHz\r\n");
//
//        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase Period = ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Bemf.TimePhasePeriod);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " 20 kHz\r\n");
//
//        	Terminal_SendString(&p_mc->Shell.Terminal, "Encoder Capture Time 		= ");
//    		Terminal_SendNum(&p_mc->Shell.Terminal, p_mc->CONFIG.P_MOTORS[0U].Encoder.DeltaT);
//    		Terminal_SendString(&p_mc->Shell.Terminal, " 625 kHz\r\n");

			//    		Terminal_SendString(p_CmdTerminal, "\r\nMotor ");
			//     		Terminal_SendNum(p_CmdTerminal, iMotor + 1U);
			//        	Terminal_SendString(p_CmdTerminal, ":\r\n");

			//        	Terminal_SendString(p_CmdTerminal, "\r\nBackEMF A 			= ");
			//    		Terminal_SendNum(p_CmdTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].VBemfA_mV);
			//    		Terminal_SendString(p_CmdTerminal, " mV\r\n");
			//
			//        	Terminal_SendString(p_CmdTerminal, "\r\nBackEMF Peak 		= ");
			//    		Terminal_SendNum(p_CmdTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].VBemfPeak_mV);
			//    		Terminal_SendString(p_CmdTerminal, " mV\r\n");
			//
			//        	Terminal_SendString(p_CmdTerminal, "\r\nBus Voltage 			= ");
			//    		Terminal_SendNum(p_CmdTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].VBus_mV);
			//    		Terminal_SendString(p_CmdTerminal, " mV\r\n");

    		Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");



//    	}


	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_foc(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {
    	p_mc->CONFIG.P_MOTORS[0].Parameters.CommutationMode = MOTOR_COMMUTATION_MODE_FOC;
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_sixstep(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {
    	p_mc->CONFIG.P_MOTORS[0].Parameters.CommutationMode = MOTOR_COMMUTATION_MODE_SIX_STEP;
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_debug(MotorController_T * p_mc, int argc, char ** argv)
{
//    	for(uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
//    	{



//	Terminal_SendString(&p_mc->Shell.Terminal, "PwmOn ");
//	Terminal_SendNum(&p_mc->Shell.Terminal, sampleCount);
//	Terminal_SendString(&p_mc->Shell.Terminal, " :\r\n");
//
//	for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
//	{
//		Terminal_SendNum(&p_mc->Shell.Terminal, p_samples[iSample]);
//		Terminal_SendString(&p_mc->Shell.Terminal, ", ");
//	}
//	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
//
//
//	Terminal_SendString(&p_mc->Shell.Terminal, "PwmOff ");
//	Terminal_SendNum(&p_mc->Shell.Terminal, sampleCount);
//	Terminal_SendString(&p_mc->Shell.Terminal, " :\r\n");
//
//	for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
//	{
//		Terminal_SendNum(&p_mc->Shell.Terminal, p_samples[iSample]);
//		Terminal_SendString(&p_mc->Shell.Terminal, ", ");
//	}
//	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

//    	}


	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_calibrate(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;

	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);

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
    	else if(strncmp(argv[1U], "hallprint", 10U) == 0U)
    	{

        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase A: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[1]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "	");

        	Terminal_SendString(&p_mc->Shell.Terminal, "  Phase AC: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[2]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase -C: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[3]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "	");

        	Terminal_SendString(&p_mc->Shell.Terminal, "  Phase BC: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[4]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase B: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[5]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "	");

        	Terminal_SendString(&p_mc->Shell.Terminal, "  Phase BA: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[6]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase -A: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[7]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "	");

        	Terminal_SendString(&p_mc->Shell.Terminal, "  Phase CA: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[8]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase C: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[9]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "	");

        	Terminal_SendString(&p_mc->Shell.Terminal, "  Phase CB: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[10]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

        	Terminal_SendString(&p_mc->Shell.Terminal, "Phase -B: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[11]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "	");

        	Terminal_SendString(&p_mc->Shell.Terminal, "  Phase AB: ");
    		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->HallDebug[12]);
        	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
    	}
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

int Cmd_save(MotorController_T * p_mc, int argc, char ** argv)
{
	(void)argv;
	MotorController_User_SaveParameters_Blocking(p_mc);
    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}



int Cmd_phase(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);

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

    	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
    	Terminal_SendString(&p_mc->Shell.Terminal, "Hall: ");
		Terminal_SendNum(&p_mc->Shell.Terminal, Hall_ReadPhysicalSensors(&p_motor->Hall));
    	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
    }

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

int Cmd_hall(MotorController_T * p_mc, int argc, char ** argv)
{
	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);

    if(argc == 1U)
    {
    	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
    	Terminal_SendString(&p_mc->Shell.Terminal, "Hall: ");
		Terminal_SendNum(&p_mc->Shell.Terminal, Hall_ReadPhysicalSensors(&p_motor->Hall));

    	Terminal_SendString(&p_mc->Shell.Terminal, "Electrical Angle: ");
		Terminal_SendNum(&p_mc->Shell.Terminal, Motor_User_GetHallRotorAngle(p_motor ));

    	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
    }

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

uint32_t UserAngle;

int Cmd_rev_Loop(MotorController_T * p_mc)
{

//	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);
//
//	if (UserAngle <=  65536*7)
//	{
//		Motor_FOC_ActivateAngle(p_motor, (uint16_t)UserAngle, p_motor->Parameters.AlignVoltage_Frac16);
//		Encoder_DeltaD_Capture(&p_motor->Encoder);
//
//		Terminal_SendNum(&p_mc->Shell.Terminal, p_motor->Encoder.AngularD);
//		Terminal_SendString(&p_mc->Shell.Terminal, " ");
//		Terminal_SendNum(&p_mc->Shell.Terminal, UserAngle);
//		Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
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
//	Motor_T * p_motor = MotorController_GetPtrMotor(p_mc, 0U);
//
//	//motor ramp does not proc during off state
//	Motor_FOC_StartAngleControl(MotorController_GetPtrMotor(p_mc, 0U));
//	UserAngle = 0U;
////	p_motor->RampCmd = p_motor->Parameters.AlignVoltage_Frac16;
//
//	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
//
//    return CMD_RESERVED_RETURN_CODE_LOOP;
}


int Cmd_heat(MotorController_T * p_mc, int argc, char ** argv)
{
	int32_t heat 		= MotorController_User_GetHeatPcb_DegCInt(p_mc, 1U);
	int32_t limit 		= MotorController_User_GetHeatPcbLimit_DegCInt(p_mc, 1U);
	int32_t threshold 	= MotorController_User_GetHeatPcbThreshold_DegCInt(p_mc, 1U);

	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
	Terminal_SendString(&p_mc->Shell.Terminal, "Pcb: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, heat);
	Terminal_SendString(&p_mc->Shell.Terminal, " Celsius");
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

	Terminal_SendString(&p_mc->Shell.Terminal, "Limit: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, limit);
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

	Terminal_SendString(&p_mc->Shell.Terminal, "Threshold: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, threshold);
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


int Cmd_vmonitor(MotorController_T * p_mc, int argc, char ** argv)
{
	int32_t vSense 		= MotorController_User_GetVSense_MilliV(p_mc);
	int32_t vAcc 		= MotorController_User_GetVAcc_MilliV(p_mc);
	int32_t vPos 		= MotorController_User_GetVPos_MilliV(p_mc);
	int32_t battery 	= MotorController_GetBatteryCharge_Base(p_mc);

	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");
	Terminal_SendString(&p_mc->Shell.Terminal, "VSense: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, vSense);
	Terminal_SendString(&p_mc->Shell.Terminal, " mV");
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

	Terminal_SendString(&p_mc->Shell.Terminal, "VAcc: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, vAcc);
	Terminal_SendString(&p_mc->Shell.Terminal, " mV");
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

	Terminal_SendString(&p_mc->Shell.Terminal, "VPos: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, vPos);
	Terminal_SendString(&p_mc->Shell.Terminal, " mV");
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

	Terminal_SendString(&p_mc->Shell.Terminal, "Battery Charge: ");
	Terminal_SendNum(&p_mc->Shell.Terminal, battery);
	Terminal_SendString(&p_mc->Shell.Terminal, " Percent10");
	Terminal_SendString(&p_mc->Shell.Terminal, "\r\n");

    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}



//int Cmd_pwm(MotorController_T * p_mc, int argc, char ** argv)
//{
//	uint16_t pwm;
////	uint8_t motorIndex = 0U;
//
//	if (argc == 2U)
//	{
//		pwm = strtoul(argv[1U], 0U, 10);
//	}
////	else if (argc == 3U)
////	{
////		motorIndex = argv[2];
////		pwm = strtoul(argv[3U], 0U, 10);
////	}
//
////set user
////	MotorUser_SetInputThrottle(p_CmdMotorUser, pwm);
//
////set direct
//	//	p_Motors[0].Pwm;
//	//	Phase_Polar_SetDutyCyle(&MotorControllerShellMain.p_Motors[0].Phase, pwm);
//
//	return MC_SHELL_CMD_RETURN_CODE_SUCCESS;
//}
//
//
//////	.CommutationMode 	= MOTOR_COMMUTATION_MODE_FOC,
//////	.SensorMode 		= MOTOR_SENSOR_MODE_ENCODER,
//////	.ControlMode 		= MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
////
////	.PhasePwmMode		= PHASE_MODE_UNIPOLAR_1,
//////	.PhasePwmMode		= PHASE_MODE_BIPOLAR,
////	.BemfSampleMode 	= BEMF_SAMPLE_MODE_PWM_ON,
//int Cmd_param(MotorController_T * p_mc, int argc, char ** argv)
//{
//	if (argc == 2U)
//	{
//		//read
//	}
//	else if (argc == 3U)
//	{
//		//write
//
//	}
//
//	if (strcmp(argv[1U], "sensor") == 0U)
//	{
//		if (strcmp(argv[2U], "hall") == 0U)
//		{
////			for (uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
////			{
//////				CmdMotorParametersArray[iMotor]->SensorMode = MOTOR_SENSOR_MODE_HALL;
////			}
//		}
//	}
//}
//
//
//int Cmd_configfile(int argc, char **argv)
//{
//
//	return MC_SHELL_CMD_RETURN_CODE_SUCCESS;
//}


//int Cmd_v(MotorController_T * p_mc, int argc, char ** argv)
//{
//
//}

//int Cmd_rpm(MotorController_T * p_mc, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendString("\r\nRPM = ");
//    	Terminal_SendNum(Speed_GetRPM(&Motor1Speed));
//    	Terminal_SendString("\r\n");
//    }
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
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

//int Cmd_run(MotorController_T * p_mc, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Start(&Motor1);
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}

//int Cmd_stop(MotorController_T * p_mc, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Stop(&Motor1);
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
//
//int Cmd_vbat(MotorController_T * p_mc, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendString("\r\nVBat = ");
//    	Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));
//    	Terminal_SendString("\r\n");
//    }
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
//}
//
//


//extern int Cmd_pwm 		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_rpm 		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_jog 		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_run 		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_stop		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_release 	(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_vbat		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_v			(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_phase		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_hold		(MotorController_T * p_mc, int argc, char ** argv);
//extern int Cmd_print		(MotorController_T * p_mc, int argc, char ** argv);

//const Cmd_T CMD_PWM 		=	{ "pwm", 		"Sets pwm value", 			Cmd_pwm		};
//Cmd_T CMD_V	 		=	{ "v", 			"Sets applied voltage", 	Cmd_v		};
//Cmd_T CMD_RPM 		=	{ "rpm", 		"Display rpm", 				Cmd_rpm		};
//Cmd_T CMD_JOG 		=	{ "jog", 		"Jog motor", 				Cmd_jog		};
//Cmd_T CMD_RUN 		=	{ "run", 		"Set motor to run mode", 	Cmd_run		};
//Cmd_T CMD_STOP 		=	{ "stop", 		"Set motor to idle mode", 	Cmd_stop	};
//Cmd_T CMD_FLOAT 		=	{ "float", 		"Release hold", 			Cmd_float	};
//Cmd_T CMD_VBAT 		=	{ "vbat", 		"Display battery voltage", 	Cmd_vbat	};
//Cmd_T CMD_PHASE		=	{ "phase", 		"Set motor phase", 			Cmd_phase	};
//Cmd_T CMD_HOLD		=	{ "hold", 		"hold position", 			Cmd_hold	};
//Cmd_T CMD_PRINT		=	{ "print", 		"print debug info",			Cmd_print	};

const Cmd_T MC_CMD_TABLE[MC_SHELL_CMD_COUNT] =
{
	{"monitor", 	"Display motor info",			Cmd_monitor, 	{.FUNCTION = Cmd_monitor_Loop, 	.FREQ = 1U}	},
	{"foc", 		"Set commutation mode",			Cmd_foc, 		{0U}	},
	{"sixstep", 	"Set commutation mode",			Cmd_sixstep, 	{0U}	},
	{"debug", 		"debug",						Cmd_debug, 		{0U}	},
	{"calibrate", 	"calibrate",					Cmd_calibrate, 	{0U}	},

	{"rev", 		"rev",	 						Cmd_rev, 		{.FUNCTION = Cmd_rev_Loop, 		.FREQ = 10U}	},
	{"phase", 		"Set motor phase", 			Cmd_phase	},
	{"hall", 		"Read hall", 			Cmd_hall	},

	{"heat", 		"Display temperature", 			Cmd_heat		},
	{"vmonitor", 	"Display v", 					Cmd_vmonitor	},
//	{"pwm", 		"Sets pwm value", 				Cmd_pwm		},

//	{"param", 		"Sets motor parameterss",	 	Cmd_param	},
//	{"mode", 		"Sets mode using options",	 	Cmd_mode	},
//	{"hall", 		"Sets commutation hall mode", 	Cmd_hall	},

////	{ "v", 			"Sets applied voltage", 	Cmd_v		},
////	{ "rpm", 		"Display rpm", 				Cmd_rpm		},
////	{ "jog", 		"Jog motor", 				Cmd_jog		},
////	{ "run", 		"Set motor to run mode", 	Cmd_run		},
////	{ "stop", 		"Set motor to idle mode", 	Cmd_stop	},
////	{ "float", 		"Float motor", 				Cmd_float	},
////	{ "vbat", 		"Display battery voltage", 	Cmd_vbat	},

////	{ "hold", 		"hold position", 			Cmd_hold	},
////	{ "print", 		"print debug info",			Cmd_print	},
};


const Cmd_Status_T MC_CMD_STATUS_TABLE[MC_SHELL_CMD_STATUS_COUNT] =
{

};

/******************************************************************************/
/*! @} */
/******************************************************************************/

