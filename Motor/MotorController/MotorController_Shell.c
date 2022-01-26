
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

#include "Motor/Motor/Motor_FOC.h"
//#include "Motor/Motor/Motor_FOC.h"

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
static int Cmd_monitor(const MotorController_T * p_motorController, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {

    }

	return CMD_RESERVED_RETURN_CODE_LOOP;
}

static int Cmd_monitor_Loop(const MotorController_T * p_motorController)
{
//    	for(uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
//    	{
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

        	Terminal_SendString(&p_motorController->Shell.Terminal, "Speed = ");
    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Speed_RPM);
    		Terminal_SendString(&p_motorController->Shell.Terminal, " RPM\r\n");

			Terminal_SendString(&p_motorController->Shell.Terminal, "RampCmd = ");
			Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].RampCmd);
			Terminal_SendString(&p_motorController->Shell.Terminal, " Frac16\r\n");

        	Terminal_SendString(&p_motorController->Shell.Terminal, "SpeedFeedback = ");
    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].SpeedFeedback_Frac16);
    		Terminal_SendString(&p_motorController->Shell.Terminal, " Frac16\r\n");


        	Terminal_SendString(&p_motorController->Shell.Terminal, "Iq = ");
    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Foc.Iq);
    		Terminal_SendString(&p_motorController->Shell.Terminal, " Q1.15\r\n");


        	Terminal_SendString(&p_motorController->Shell.Terminal, "Iabc = ");
    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Foc.Ia);
    		Terminal_SendString(&p_motorController->Shell.Terminal, " ");
       		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Foc.Ib);
       		Terminal_SendString(&p_motorController->Shell.Terminal, " ");
       		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Foc.Ic);
    		Terminal_SendString(&p_motorController->Shell.Terminal, " Q1.15\r\n");

//        	Terminal_SendString(&p_motorController->Shell.Terminal, "Id 				= ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " SignedFrac16\r\n");

//        	Terminal_SendString(&p_motorController->Shell.Terminal, "IBus = ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].IBus_Frac16);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " Frac16\r\n");

//        	Terminal_SendString(&p_motorController->Shell.Terminal, "VBemf_On = ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Bemf.VPhase_ADCU);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " ADCU\r\n");
//
//        	Terminal_SendString(&p_motorController->Shell.Terminal, "VBemf_Off = ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Bemf.VPhasePwmOff_ADCU);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " ADCU\r\n");

//        	Terminal_SendString(&p_motorController->Shell.Terminal, "PwmOnTime = ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].PwmOnTime);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " \r\n");

//        	Terminal_SendString(&p_motorController->Shell.Terminal, "ZC Period = ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Bemf.TimeZeroCrossingPeriod);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " 20 kHz\r\n");
//
//        	Terminal_SendString(&p_motorController->Shell.Terminal, "Phase Period = ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Bemf.TimePhasePeriod);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " 20 kHz\r\n");
//
//        	Terminal_SendString(&p_motorController->Shell.Terminal, "Encoder Capture Time 		= ");
//    		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motorController->CONFIG.P_MOTORS[0U].Encoder.DeltaT);
//    		Terminal_SendString(&p_motorController->Shell.Terminal, " 625 kHz\r\n");

    		Terminal_SendString(&p_motorController->Shell.Terminal, "\n");



//    	}


	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_foc(const MotorController_T * p_motorController, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {
    	p_motorController->CONFIG.P_MOTORS[0].Parameters.CommutationMode = MOTOR_COMMUTATION_MODE_FOC;
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

static int Cmd_sixstep(const MotorController_T * p_motorController, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {
    	p_motorController->CONFIG.P_MOTORS[0].Parameters.CommutationMode = MOTOR_COMMUTATION_MODE_SIX_STEP;
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_debug(const MotorController_T * p_motorController, int argc, char ** argv)
{
//    	for(uint8_t iMotor = 0U; iMotor < CmdMotorCount; iMotor++)
//    	{

	uint32_t * p_samples = p_motorController->CONFIG.P_MOTORS[0U].VSamplesPwmOn;
	uint32_t sampleCount = p_motorController->CONFIG.P_MOTORS[0U].DebugCounterEndPwmOn;

	Terminal_SendString(&p_motorController->Shell.Terminal, "PwmOn ");
	Terminal_SendNum(&p_motorController->Shell.Terminal, sampleCount);
	Terminal_SendString(&p_motorController->Shell.Terminal, " :\r\n");

	for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
	{
		Terminal_SendNum(&p_motorController->Shell.Terminal, p_samples[iSample]);
		Terminal_SendString(&p_motorController->Shell.Terminal, ", ");
	}
	Terminal_SendString(&p_motorController->Shell.Terminal, "\r\n");


	p_samples = p_motorController->CONFIG.P_MOTORS[0U].VSamplesPwmOff;
	sampleCount = p_motorController->CONFIG.P_MOTORS[0U].DebugCounterEndPwmOff;

	Terminal_SendString(&p_motorController->Shell.Terminal, "PwmOff ");
	Terminal_SendNum(&p_motorController->Shell.Terminal, sampleCount);
	Terminal_SendString(&p_motorController->Shell.Terminal, " :\r\n");

	for(uint8_t iSample = 0U; iSample < sampleCount; iSample++)
	{
		Terminal_SendNum(&p_motorController->Shell.Terminal, p_samples[iSample]);
		Terminal_SendString(&p_motorController->Shell.Terminal, ", ");
	}
	Terminal_SendString(&p_motorController->Shell.Terminal, "\r\n");

//    	}


	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


static int Cmd_calibrate(const MotorController_T * p_motorController, int argc, char ** argv)
{
	(void)argv;

    if(argc == 1U)
    {
    	Motor_User_ActivateCalibrationEncoder(MotorController_GetPtrMotor(p_motorController, 0U));
    }
    else if(argc == 2U)
    {
    	if(strncmp(argv[1U], "encoder", 8U) == 0U)
    	{
    		Motor_User_ActivateCalibrationEncoder(MotorController_GetPtrMotor(p_motorController, 0U));
    	}
    	else if(strncmp(argv[1U], "hall", 5U) == 0U)
    	{
    		Motor_User_ActivateCalibrationHall(MotorController_GetPtrMotor(p_motorController, 0U));
    	}
    	else if(strncmp(argv[1U], "adc", 4U) == 0U)
    	{
    		Motor_User_ActivateCalibrationAdc(MotorController_GetPtrMotor(p_motorController, 0U));
    	}
    }

	return CMD_RESERVED_RETURN_CODE_SUCCESS;
}

uint32_t UserAngle;

int Cmd_rev_Loop(MotorController_T * p_motorController)
{

//	Motor_T * p_motor = MotorController_GetPtrMotor(p_motorController, 0U);
//
//	if (UserAngle <=  65536*7)
//	{
//		Motor_FOC_ActivateAngle(p_motor, (uint16_t)UserAngle, p_motor->Parameters.AlignVoltage_Frac16);
//		Encoder_DeltaD_Capture(&p_motor->Encoder);
//
//		Terminal_SendNum(&p_motorController->Shell.Terminal, p_motor->Encoder.AngularD);
//		Terminal_SendString(&p_motorController->Shell.Terminal, " ");
//		Terminal_SendNum(&p_motorController->Shell.Terminal, UserAngle);
//		Terminal_SendString(&p_motorController->Shell.Terminal, "\r\n");
//
//		UserAngle += 1024;
//	}
//
//
//    return CMD_RESERVED_RETURN_CODE_SUCCESS;
}


int Cmd_rev(const MotorController_T * p_motorController, int argc, char ** argv)
{
//	(void)argv;
//	Motor_T * p_motor = MotorController_GetPtrMotor(p_motorController, 0U);
//
//	//motor ramp does not proc during off state
//	Motor_FOC_StartAngleControl(MotorController_GetPtrMotor(p_motorController, 0U));
//	UserAngle = 0U;
////	p_motor->RampCmd = p_motor->Parameters.AlignVoltage_Frac16;
//
//	Terminal_SendString(&p_motorController->Shell.Terminal, "\r\n");
//
//    return CMD_RESERVED_RETURN_CODE_LOOP;
}



//int Cmd_pwm(const MotorController_T * p_motorController, int argc, char ** argv)
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
//int Cmd_param(const MotorController_T * p_motorController, int argc, char ** argv)
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


//int Cmd_v(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//
//}

//int Cmd_rpm(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendString("\r\nRPM = ");
//    	Terminal_SendNum(Speed_GetRPM(&Motor1Speed));
//    	Terminal_SendString("\r\n");
//    }
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_jog(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	if(argc == 1)
//	{
//		Motor_SetJogSteps(&Motor1, 1);
//
//	}
//	else if(argc == 2) Motor_SetJogSteps(&Motor1, strtoul(argv[1], 0, 10));
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_run(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Start(&Motor1);
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}

//int Cmd_stop(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Stop(&Motor1);
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}
//
//int Cmd_hold(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Hold(&Motor1);
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}
//
//int Cmd_release(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1) Motor_Release(&Motor1);
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}
//
//int Cmd_vbat(const MotorController_T * p_motorController, int argc, char ** argv)
//{
//	(void)argv;
//    if(argc == 1)
//    {
//    	Terminal_SendString("\r\nVBat = ");
//    	Terminal_SendNum(VoltageDivider_GetVoltage(&DividerCommon, *Motor1.VBat_ADCU));
//    	Terminal_SendString("\r\n");
//    }
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}
//
//
//int Cmd_phase(const MotorController_T * p_motorController, int argc, char ** argv)
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
//    return MC_SHELL_CMD_RETURN_CODE_OK;
//}

//extern int Cmd_pwm 		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_rpm 		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_jog 		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_run 		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_stop		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_release 	(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_vbat		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_v			(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_phase		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_hold		(const MotorController_T * p_motorController, int argc, char ** argv);
//extern int Cmd_print		(const MotorController_T * p_motorController, int argc, char ** argv);

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
////	{ "phase", 		"Set motor phase", 			Cmd_phase	},
////	{ "hold", 		"hold position", 			Cmd_hold	},
////	{ "print", 		"print debug info",			Cmd_print	},
};


const Cmd_Status_T MC_CMD_STATUS_TABLE[MC_SHELL_CMD_STATUS_COUNT] =
{

};

/******************************************************************************/
/*! @} */
/******************************************************************************/

