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
#include "MotorControllerShell.h"

#include "MotorController.h"

#include "System/Shell/Shell.h"
#include "System/Shell/Terminal.h"
#include "System/Shell/Cmd.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MOTOR_SHELL_CMD_COUNT		 	10U
#define MOTOR_SHELL_CMD_RETURN_COUNT 	1U
#define MOTOR_SHELL_PROC_FREQ			100U

extern const Cmd_T MOTOR_CMD_TABLE[MOTOR_SHELL_CMD_COUNT];
extern const Cmd_Return_T MOTOR_CMD_RETURN_TABLE[MOTOR_SHELL_CMD_RETURN_COUNT];

//support 1 instance of shell
//Cmds function reference 1 instace of Motor Controller
//this way cmd functions do not need to include context data in arugments.
static MotorController_T * p_CmdContextMotorController;

//static Motor_T * p_CmdMotors; //pointer to array of motors
static const Shell_T * p_CmdContextShell; //Read-only
static const Terminal_T * p_CmdContextTerminal; //read-only

/*******************************************************************************/
/*!
    @brief Init

    inits single instance of cmd context
*/
/*******************************************************************************/
void MotorControllerShell_Init(const MotorController_T * p_motorController)
{
	Shell_Init
	(
		&p_motorController->MotorControllerShell,
		&MOTOR_CMD_TABLE,
		MOTOR_SHELL_CMD_COUNT,
		&MOTOR_CMD_RETURN_TABLE,
		MOTOR_SHELL_CMD_RETURN_COUNT,
		&p_motorController->Serials[0], //&p_motorController->Parameters.ShellSerialId
//		p_motorController,
		MOTOR_SHELL_PROC_FREQ
	);

	p_CmdContextMotorController 	= p_motorController;
	p_CmdContextShell 				= &p_motorController->MotorControllerShell;
	p_CmdContextTerminal 			= &p_motorController->MotorControllerShell.Terminal;
}


//void MotorControllerShell_SetContext(const MotorController_T * p_motorController, Shell_T * p_shell, Terminal_T * p_terminal)
//{
//	p_CmdContextMotorController 	= p_motorController;
//	p_CmdContextShell 				= p_shell;
//	p_CmdContextTerminal 			= p_terminal;
//}
//
//
//void MotorControllerShell_SetMotorController(const MotorController_T * p_motorController)
//{
//	p_CmdContextMotorController 	= p_motorController;
//	p_CmdContextShell 				= &p_motorController->MotorControllerShell;
//	p_CmdContextTerminal 			= &p_motorController->MotorControllerShell.Terminal;
//}

/*******************************************************************************/
/*!
	@brief 	Cmd Functions
 */
/*! @{ */
/*******************************************************************************/
//int Cmd_(const MotorController_T * p_motorController, int argc, char ** argv)

//
//int Cmd_print(int argc, char ** argv)
//{
////	//printf("Loop Time is %d \n", LoopDelta);
////	//printf("PWM %d \n", Motor1.PWM);
////	printf("BackEMF A ADCU %d \n", *Motor1.BackEMFPhaseA_ADCU);
////	printf("BackEMF B ADCU %d \n", *Motor1.BackEMFPhaseB_ADCU);
////	//printf("Battery Voltage %d \n", VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));

////
//////	printf("Motor Desired %d \n", *Motor1PID.SetPoint);
//////	printf("Motor Input %d \n", *Motor1PID.Input);
//////	printf("Motor Error (Measured - Input) %d \n", *Motor1PID.SetPoint - *Motor1PID.Input);
//
//
//    return MOTOR_SHELL_CMD_RETURN_CODE_SUCCESS;
//}

int Cmd_monitor(int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {
    	//printf("Loop Time is %d \n", LoopDelta);
    	//printf("PWM %d \n", Motor1.PWM);

    	//	printf("Motor Desired %d \n", *Motor1PID.SetPoint);
    	//	printf("Motor Input %d \n", *Motor1PID.Input);
    	//	printf("Motor Error (Measured - Input) %d \n", *Motor1PID.SetPoint - *Motor1PID.Input);

//    	Terminal_SendString(p_CmdContextTerminal, "\r\nBackEMF A 			= ");
//		Terminal_SendNum(p_CmdContextTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].VBemfA_mV);
//		Terminal_SendString(p_CmdContextTerminal, " mV\r\n");
//
//    	Terminal_SendString(p_CmdContextTerminal, "\r\nBackEMF Peak 		= ");
//		Terminal_SendNum(p_CmdContextTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].VBemfPeak_mV);
//		Terminal_SendString(p_CmdContextTerminal, " mV\r\n");
//
//    	Terminal_SendString(p_CmdContextTerminal, "\r\nBus Voltage 			= ");
//		Terminal_SendNum(p_CmdContextTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].VBus_mV);
//		Terminal_SendString(p_CmdContextTerminal, " mV\r\n");
//
//    	Terminal_SendString(p_CmdContextTerminal, "\r\nSpeed 				= ");
//		Terminal_SendNum(p_CmdContextTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].Speed_RPM);
//		Terminal_SendString(p_CmdContextTerminal, " RPM\r\n");
//
//    	Terminal_SendString(p_CmdContextTerminal, "\r\nCurrent 				= ");
//		Terminal_SendNum(p_CmdContextTerminal, p_CmdContextMotorController->p_Constants->P_MOTORS[0].IBus_Amp);
//		Terminal_SendString(p_CmdContextTerminal, " Amp\r\n");

    }

	return MOTOR_SHELL_CMD_RETURN_CODE_SUCCESS;
}



int Cmd_pwm(int argc, char ** argv)
{
	uint16_t pwm;
	uint8_t motorN;

	if (argc == 2U)
	{
		pwm = strtoul(argv[2U], 0U, 10);
	}
	else if (argc == 3U)
	{
		motorN = argv[2];
		pwm = strtoul(argv[3U], 0U, 10);
	}

//set user
	p_CmdContextMotorController->MotorUser.InputValueThrottlePrev 	= p_CmdContextMotorController->MotorUser.InputValueThrottle;
	p_CmdContextMotorController->MotorUser.InputValueThrottle 		= pwm;

//set direct
	//	p_Motors[0].Pwm;
	//	Phase_Polar_SetDutyCyle(&MotorControllerShellMain.p_Motors[0].Phase, pwm);

	return MOTOR_SHELL_CMD_RETURN_CODE_SUCCESS;
}


int Cmd_mode(int argc, char ** argv)
{

}

int Cmd_hall(int argc, char ** argv)
{

}

int Cmd_configfile(int argc, char **argv)
{

	return MOTOR_SHELL_CMD_RETURN_CODE_SUCCESS;
}
//
//int Cmd_read(int argc, char **argv)
//{
//	static char buffer[5];
////	snprintf(buffer, 5, "%d", MotorControllerShellMain.p_Motors[0].VBus);
////	sprintf()
//	if (strcmp(argv[1], "vbus") == 0U)
//	{
//		Terminal_SendString(&MotorControllerShellMain.Terminal, "\r\nVBus = ");
//		Terminal_SendString(&MotorControllerShellMain.Terminal, buffer);
////    	Terminal_SendNum(MotorControllerShellMain.p_Motors[0].VBus);
//		Terminal_SendString(&MotorControllerShellMain.Terminal, "\r\n");
//	}
//	else
//	{
//
//	}
//
//	return MOTOR_SHELL_CMD_RETURN_CODE_OK;
//}


//int Cmd_v(int argc, char ** argv)
//{
//
//}

//int Cmd_rpm(int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendString("\r\nRPM = ");
//    	Terminal_SendNum(Speed_GetRPM(&Motor1Speed));
//    	Terminal_SendString("\r\n");
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
//    	Terminal_SendString("\r\nVBat = ");
//    	Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));
//    	Terminal_SendString("\r\n");
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

//extern int Cmd_pwm 		(int argc, char ** argv);
//extern int Cmd_rpm 		(int argc, char ** argv);
//extern int Cmd_jog 		(int argc, char ** argv);
//extern int Cmd_run 		(int argc, char ** argv);
//extern int Cmd_stop		(int argc, char ** argv);
//extern int Cmd_release 	(int argc, char ** argv);
//extern int Cmd_vbat		(int argc, char ** argv);
//extern int Cmd_v			(int argc, char ** argv);
//extern int Cmd_phase		(int argc, char ** argv);
//extern int Cmd_hold		(int argc, char ** argv);
//extern int Cmd_print		(int argc, char ** argv);

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

const Cmd_T MOTOR_CMD_TABLE[MOTOR_SHELL_CMD_COUNT] =
{
	{"monitor", 	"Display motor info",			Cmd_monitor	},
	{"pwm", 		"Sets pwm value", 				Cmd_pwm		},
	{"mode", 		"Sets mode using options",	 	Cmd_mode	},
	{"hall", 		"Sets commutation hall mode", 	Cmd_hall	},

////	{ "v", 			"Sets applied voltage", 	Cmd_v		},
////	{ "rpm", 		"Display rpm", 				Cmd_rpm		},
////	{ "jog", 		"Jog motor", 				Cmd_jog		},
////	{ "run", 		"Set motor to run mode", 	Cmd_run		},
////	{ "stop", 		"Set motor to idle mode", 	Cmd_stop	},
////	{ "float", 		"Float motor", 				Cmd_float	},
////	{ "vbat", 		"Display battery voltage", 	Cmd_vbat	},
////	{ "phase", 		"Set motor phase", 			Cmd_phase	},
////	{ "hold", 		"hold position", 			Cmd_hold	},
////	{ "print", 		"print debug info",			Cmd_print	},
};


const Cmd_Return_T MOTOR_CMD_RETURN_TABLE[MOTOR_SHELL_CMD_RETURN_COUNT] =
{

};

/*******************************************************************************/
/*! @} */
/*******************************************************************************/

//void MotorControllerShell_RegisterShellCmds()
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
