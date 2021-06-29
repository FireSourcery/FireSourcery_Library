/*******************************************************************************/
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
/*******************************************************************************/
/*******************************************************************************/
/*!
    @file
    @author FireSoucery
    @brief
    @version V0
*/
/*******************************************************************************/
#include "System/Shell/Shell.h"
#include "System/Shell/Terminal.h"
#include "System/Shell/Cmd.h"

#include "Motor/Motor.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/******************************************************************************/
/*!
	@name	ShellSection
	Shell functions
 */
/******************************************************************************/
/*! @{ */
typedef struct
{

	 Motor_T * p_Motors;
	 uint8_t MotorCount;
}
MotorShell_T;

MotorShell_T MainMotorShell;

//extern int Cmd_pwm 		(int argc, char ** argv);
//extern int Cmd_rpm 		(int argc, char ** argv);
//extern int Cmd_jog 		(int argc, char ** argv);
//extern int Cmd_run 		(int argc, char ** argv);
//extern int Cmd_stop		(int argc, char ** argv);
//extern int Cmd_release 	(int argc, char ** argv);
//extern int Cmd_vbat		(int argc, char ** argv);
//extern int Cmd_v		(int argc, char ** argv);
//extern int Cmd_phase	(int argc, char ** argv);
//extern int Cmd_hold		(int argc, char ** argv);
//extern int Cmd_print	(int argc, char ** argv);

//Cmd_T Cmd_pwm 		=	{ "pwm", 		"Sets pwm value", 			Cmd_pwm		};
//Cmd_T Cmd_v	 		=	{ "v", 			"Sets applied voltage", 	Cmd_v		};
//Cmd_T Cmd_rpm 		=	{ "rpm", 		"Display rpm", 				Cmd_rpm		};
//Cmd_T Cmd_jog 		=	{ "jog", 		"Jog motor", 				Cmd_jog		};
//Cmd_T Cmd_run 		=	{ "run", 		"Set motor to run mode", 	Cmd_run		};
//Cmd_T Cmd_stop 		=	{ "stop", 		"Set motor to idle mode", 	Cmd_stop	};
//Cmd_T Cmd_release 	=	{ "release", 	"Release hold", 			Cmd_release	};
//Cmd_T Cmd_vbat 		=	{ "vbat", 		"Display battery voltage", 	Cmd_vbat	};
//Cmd_T Cmd_phase		=	{ "phase", 		"Set motor phase", 			Cmd_phase	};
//Cmd_T Cmd_hold		=	{ "hold", 		"hold position", 			Cmd_hold	};
//Cmd_T Cmd_print		=	{ "print", 		"print debug info",			Cmd_print	};


//const Cmd_T CMD_TABLE[] =
//{
//	{ "pwm", 		"Sets pwm value", 			Cmd_pwm		},
////	{ "v", 			"Sets applied voltage", 	Cmd_v		},
////	{ "rpm", 		"Display rpm", 				Cmd_rpm		},
////	{ "jog", 		"Jog motor", 				Cmd_jog		},
////	{ "run", 		"Set motor to run mode", 	Cmd_run		},
////	{ "stop", 		"Set motor to idle mode", 	Cmd_stop	},
////	{ "release", 	"Release hold", 			Cmd_release	},
////	{ "vbat", 		"Display battery voltage", 	Cmd_vbat	},
////	{ "phase", 		"Set motor phase", 			Cmd_phase	},
////	{ "hold", 		"hold position", 			Cmd_hold	},
////	{ "print", 		"print debug info",			Cmd_print	},
//};


void MotorShell_Init(Motor_T * p_motors, uint8_t motorCount)
{

//	Shell_Init
//	(
//		&CMD_TABLE,
//	);
//	Cmd_T * p_cmdTable,
//	uint8_t cmdCount,
//	Cmd_ReturnCode_T * p_returnCodeTable,
//	uint8_t returnCodeCount,
//	uint16_t cmdLoopFreq,
//	uint16_t shellProcFreq

	MainMotorShell.p_Motors  = p_motors;
	MainMotorShell.MotorCount = motorCount;
}

//#define SHELL_OUTTER_LOOP_FREQ 100

typedef enum
{
	MOTOR_SHELL_CMD_RETURN_CODE_OK = 0,
	MOTOR_SHELL_CMD_RETURN_CODE_ERROR_1 = 1,
} MotorShell_CmdReturnCode_T;

int Cmd_configfile(int argc, char **argv)
{


	return MOTOR_SHELL_CMD_RETURN_CODE_OK;
}

int Cmd_read(int argc, char **argv)
{
	static char buffer[5];
//	snprintf(buffer, 5, "%d", MainMotorShell.p_Motors[0].VBus);
//	sprintf()
	if (strcmp(argv[1], "vbus") == 0U)
	{
		Terminal_SendString("\r\nVBus = ");
		Terminal_SendString(buffer);
//    	Terminal_SendNum(MainMotorShell.p_Motors[0].VBus);
		Terminal_SendString("\r\n");
	}
	else
	{

	}

	return MOTOR_SHELL_CMD_RETURN_CODE_OK;
}

int Cmd_pwm(int argc, char **argv)
{
//	uint8_t motorN = argv[2];
	uint16_t pwm;

	if (argc == 2)
	{
		pwm = strtoul(argv[2], 0, 10);
	}
	else if (argc == 3)
	{
		pwm = strtoul(argv[3], 0, 10);
	}

	Phase_Polar_SetDutyCyle(&MainMotorShell.p_Motors[0].Phase, pwm);

	return MOTOR_SHELL_CMD_RETURN_CODE_OK;
}

//int Cmd_v(int argc, char ** argv)
//{
//
//}

//int Cmd_rpm(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendStr("\r\nRPM = ");
//    	Terminal_SendNum(Speed_GetRPM(&Motor1Speed));
//    	Terminal_SendStr("\r\n");
//    }
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_jog(int argc, char ** argv)
//{
//	if(argc == 1)
//	{
//		Motor_SetJogSteps(&Motor1, 1);
//
//	}
//	else if(argc == 2) Motor_SetJogSteps(&Motor1, strtoul(argv[1], 0, 10));
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_run(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Start(&Motor1);
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_stop(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Stop(&Motor1);
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}
//
//int Cmd_hold(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Hold(&Motor1);
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}
//
//int Cmd_release(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Release(&Motor1);
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}
//
//int Cmd_vbat(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendStr("\r\nVBat = ");
//    	Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));
//    	Terminal_SendStr("\r\n");
//    }
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}
//
//
//int Cmd_phase(int argc, char ** argv)
//{
//    if(argc == 2)
//    {
//    	if 		(argv[1][0] == 'a' && argv[1][1] == 'b') CommutateMotor1PhaseAB(Motor1.PWM);
//    	else if (argv[1][0] == 'a' && argv[1][1] == 'c') CommutateMotor1PhaseAC(Motor1.PWM);
//    	else if (argv[1][0] == 'b' && argv[1][1] == 'a') CommutateMotor1PhaseBA(Motor1.PWM);
//    	else if (argv[1][0] == 'b' && argv[1][1] == 'c') CommutateMotor1PhaseBC(Motor1.PWM);
//    	else if (argv[1][0] == 'c' && argv[1][1] == 'a') CommutateMotor1PhaseCA(Motor1.PWM);
//    	else if (argv[1][0] == 'c' && argv[1][1] == 'b') CommutateMotor1PhaseCB(Motor1.PWM);
//
////
////    	switch(argv[1][0])
////    	{
////
////    	case 'a':
////    	case 'A':
////    		switch(argv[1][1])
////    		{
////
////        	case 'b':
////        	case 'B':
////
////        		break;
////
////    		case 'c':
////    		case 'C':
////
////    			break;
////    		}
////    		break;
////
////		case 'b':
////		case 'B':
////			switch(argv[1][1])
////			{
////
////			}
////			break;
////
////		case 'c':
////		case 'C':
////			switch(argv[1][1])
////			{
////
////			}
////			break;
////
////    	}
//
//
//    }
//
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_print(int argc, char ** argv)
//{
////	//printf("Loop Time is %d \n", LoopDelta);
////	//printf("PWM %d \n", Motor1.PWM);
////	printf("BackEMF A ADCU %d \n", *Motor1.BackEMFPhaseA_ADCU);
////	printf("BackEMF B ADCU %d \n", *Motor1.BackEMFPhaseB_ADCU);
////	//printf("Battery Voltage %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));
////	printf("BackEMF A %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFPhaseA_ADCU));
////	printf("BackEMF B %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFPhaseB_ADCU));
////	printf("BackEMF Select %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFSelect_ADCU));
////	printf("Hall Delta %d \n", Speed_GetDeltaTicks(&Motor1Speed));
////	//printf("RPM %d \n", Speed_GetRPM(&Motor1Speed));
////	printf("I ADCU %d \n", *Motor1.I_ADCU);
////
//////	printf("Motor Desired %d \n", *Motor1PID.SetPoint);
//////	printf("Motor Input %d \n", *Motor1PID.Input);
//////	printf("Motor Error (Measured - Input) %d \n", *Motor1PID.SetPoint - *Motor1PID.Input);
//
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendStr("\r\nBackEMF A  = ");
//		Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFPhaseA_ADCU));
//		Terminal_SendStr("\r\n");
//
//    	Terminal_SendStr("\r\nBackEMF B  = ");
//		Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFPhaseB_ADCU));
//		Terminal_SendStr("\r\n");
//
//    	Terminal_SendStr("\r\nBackEMFSelect = ");
//    	Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.BackEMFSelect_ADCU));
//    	Terminal_SendStr("\r\n");
//
//    	Terminal_SendStr("\r\nDelta Ticks = ");
//		Terminal_SendNum(Speed_GetDeltaTicks(&Motor1Speed));
//		Terminal_SendStr("\r\n");
//
//    	Terminal_SendStr("\r\nI ADCU = ");
//    	Terminal_SendNum(*Motor1.I_ADCU);
//    	Terminal_SendStr("\r\n");
//
//    }
//    return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}






//void MotorShell_RegisterShellCmds()
//{
//	Shell_RegisterCmdLineEntry(&Cmd_pwm);
//	Shell_RegisterCmdLineEntry(&Cmd_rpm);
//	Shell_RegisterCmdLineEntry(&Cmd_jog);
//	Shell_RegisterCmdLineEntry(&Cmd_run);
//	Shell_RegisterCmdLineEntry(&Cmd_stop);
//	Shell_RegisterCmdLineEntry(&Cmd_release);
//	Shell_RegisterCmdLineEntry(&Cmd_vbat);
//	Shell_RegisterCmdLineEntry(&Cmd_v);
//	Shell_RegisterCmdLineEntry(&Cmd_phase);
//	Shell_RegisterCmdLineEntry(&Cmd_hold);
//	Shell_RegisterCmdLineEntry(&Cmd_print);
//}
/*! @} */
