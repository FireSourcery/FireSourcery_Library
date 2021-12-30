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
    @file 	MotorStateMachine.c
    @author FireSoucery
    @brief  MotorStateMachine
    @version V0
*/
/******************************************************************************/
#include "Motor_StateMachine.h"
#include "Motor.h"
#include "Motor_FOC.h"
#include "Motor_SixStep.h"
#include "Utility/StateMachine/StateMachine.h"

#define MSM_TRANSITION_TABLE_LENGTH 	(11U)

static const StateMachine_State_T MOTOR_STATE_INIT;
static const StateMachine_State_T MOTOR_STATE_STOP;
static const StateMachine_State_T MOTOR_STATE_ALIGN;
static const StateMachine_State_T MOTOR_STATE_OPEN_LOOP;
static const StateMachine_State_T MOTOR_STATE_RUN;
static const StateMachine_State_T MOTOR_STATE_FREEWHEEL;
static const StateMachine_State_T MOTOR_STATE_CALIBRATION;
static const StateMachine_State_T MOTOR_STATE_FAULT;

/******************************************************************************/
/*!
    @brief State Machine
*/
/******************************************************************************/
const StateMachine_Machine_T MSM_MACHINE =
{
	.P_STATE_INITIAL 			= &MOTOR_STATE_INIT,
	.TRANSITION_TABLE_LENGTH 	= MSM_TRANSITION_TABLE_LENGTH,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_STOP] 			= (StateMachine_Transition_T)TransitionStop,
//	[MSM_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_INIT] 			= {&MOTOR_STATE_INIT, 			0U},
//	[MSM_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_RUN] 			= {&MOTOR_STATE_FAULT, 			0U},
};

static void Init_Entry(Motor_T * p_motor)
{
//	Motor_InitReboot(p_motor);
}

static void Init_Proc(Motor_T * p_motor)
{
//	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_STOP);
	StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
}

static const StateMachine_State_T MOTOR_STATE_INIT =
{
	.P_TRANSITION_TABLE		= INIT_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Init_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Init_Proc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static StateMachine_State_T * Stop_InputAccelerate(Motor_T * p_motor)
{
	return	((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP)) ? &MOTOR_STATE_ALIGN : &MOTOR_STATE_RUN;
}

//static StateMachine_State_T * StopTransitionDecelerate(Motor_T * p_motor)
//{
//	out pwm 0 on config
//	return &MOTOR_STATE_RUN;
//}

static StateMachine_State_T * Stop_InputCalibration(Motor_T * p_motor)
{
	return &MOTOR_STATE_CALIBRATION;
}


static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_ACCELERATE] 			= (StateMachine_Transition_T)Stop_InputAccelerate,
	[MSM_INPUT_CALIBRATION] 		= (StateMachine_Transition_T)Stop_InputCalibration,

//	[MSM_TRANSITION_RUN] 			= (StateMachine_Transition_T)TransitionRun,
//	[MSM_TRANSITION_CALIBRATION] 	= (StateMachine_Transition_T)TransitionCalibration,
//	[MSM_TRANSITION_FREEWHEEL] 		= (StateMachine_Transition_T)TransitionFreeWheel,
//	[MSM_INPUT_DECELERATE] 			= (StateMachine_Transition_T)StopTransitionDecelerate,
//	[MSM_TRANSITION_OPEN_LOOP] 			= {&MOTOR_STATE_OPEN_LOOP,		0U},
//	[MSM_TRANSITION_ALIGN] 				= {&MOTOR_STATE_ALIGN, 			0U},
//	[MSM_TRANSITION_FAULT] 				= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_INIT] 				= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_STOP] 				= {&MOTOR_STATE_FAULT, 			0U},
};

static void Stop_Entry(Motor_T * p_motor)
{
	Motor_Stop(p_motor);
	Motor_StartIdle(p_motor);
}

static void Stop_Proc(Motor_T * p_motor)
{
	if(Motor_GetSpeed(p_motor) > 0U)
	{
//		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_FREEWHEEL);
		StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL);
	}
	else
	{
		Motor_ProcIdle(p_motor);
	}
}

static const StateMachine_State_T MOTOR_STATE_STOP =
{
	.P_TRANSITION_TABLE 	= STOP_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Stop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Stop_Proc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T ALIGN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_RUN] 		= (StateMachine_Transition_T)TransitionRun,
//	[MSM_TRANSITION_OPEN_LOOP] 	= (StateMachine_Transition_T)TransitionOpenLoop,

//	[MSM_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_INIT] 			= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_STOP] 			= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
};

static void Align_Entry(Motor_T * p_motor)
{
	Motor_StartAlign(p_motor);
}

static void Align_Proc(Motor_T * p_motor)
{
	if(Motor_ProcAlign(p_motor) == true)
	{
		if((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP))
		{
//			StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_OPEN_LOOP);
			StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_OPEN_LOOP);
		}
		else
		{
//			StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_RUN);
			StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_RUN);
		}
	}
}

static const StateMachine_State_T MOTOR_STATE_ALIGN =
{
	.P_TRANSITION_TABLE 	= ALIGN_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Align_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Align_Proc,
};

/******************************************************************************/
/*!
    @brief  State OpenLoop
*/
/******************************************************************************/
static StateMachine_State_T * OpenLoop_InputStop(Motor_T * p_motor)
{
	return &MOTOR_STATE_FREEWHEEL; //No resume from OpenLoop, freewheel state check stop
}

static const StateMachine_Transition_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_RUN] 		= TransitionRun,
//	[MSM_TRANSITION_FREEWHEEL] 	= {&MOTOR_STATE_FREEWHEEL, 		0U},
//
//	[MSM_TRANSITION_STOP] 		= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_FAULT] 		= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_INIT] 		= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_ALIGN] 		= {&MOTOR_STATE_FAULT, 			0U},

//	[MSM_INPUT_ACCELERATE] 		= (StateMachine_Transition_T) ,
	[MSM_INPUT_DECELERATE] 		= (StateMachine_Transition_T)OpenLoop_InputStop,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)OpenLoop_InputStop,
};

static void OpenLoop_Entry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_StartOpenProc(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseControl(p_motor);
	}
}

static void OpenLoop_Proc(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//			Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcOpenLoop(p_motor);

		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
		{
			if (Motor_SixStep_GetBemfReliable(p_motor) == true)
			{
//				StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_RUN);
				StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_RUN);
			}
		}
	}
}

static const StateMachine_State_T MOTOR_STATE_OPEN_LOOP =
{
	.P_TRANSITION_TABLE 	= OPEN_LOOP_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)OpenLoop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)OpenLoop_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
//static StateMachine_State_T * RunTransitionAccelerate(Motor_T * p_motor)
//{
//	//check over currnet time
//
//	return 0U;
//}
//
//static StateMachine_State_T * RunTransitionDecelerate(Motor_T * p_motor)
//{
////	return	( ) ? &MOTOR_STATE_ALIGN : &MOTOR_STATE_RUN;
//
//	return 0U;
//}

static StateMachine_State_T * Run_InputDisable(Motor_T * p_motor)
{
	return &MOTOR_STATE_FREEWHEEL;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_FREEWHEEL] 	= (StateMachine_Transition_T)TransitionFreeWheel,

//	[MSM_TRANSITION_STOP] 		= {&MOTOR_STATE_FREEWHEEL, 		0U},
//	[MSM_TRANSITION_RUN] 		= {&MOTOR_STATE_RUN, 			0U},
//
//	[MSM_TRANSITION_FAULT] 		= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_INIT] 		= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_TRANSITION_ALIGN] 		= {&MOTOR_STATE_FAULT, 			0U},
//	[MSM_INPUT_ACCELERATE] 		= (StateMachine_Transition_T)RunTransitionAccelerate,
//	[MSM_INPUT_DECELERATE] 		= (StateMachine_Transition_T)RunTransitionDecelerate,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)Run_InputDisable,
};

static void Run_Entry(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		if(Motor_GetSpeed(p_motor) == 0U)
		{
			Motor_FOC_StartAngleControl(p_motor);
		}
		else
		{
			Motor_FOC_ResumeAngleControl(p_motor);
		}
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		if(Motor_GetSpeed(p_motor) == 0U)
		{
			Motor_SixStep_StartPhaseControl(p_motor);
		}
		else
		{
			Motor_SixStep_ResumePhaseControl(p_motor);
		}
	}
}

static void Run_Proc(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcPhaseControl(p_motor);

		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
		{
//			if (Motor_SixStep_GetBemfReliable(p_motor) == false)
//			{
//				StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_FREEWHEEL);
//			StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL);
//			}
		}
	}
}

static const StateMachine_State_T MOTOR_STATE_RUN =
{
	.P_TRANSITION_TABLE 	= RUN_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Run_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Run_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static StateMachine_State_T * FreeWheel_InputAccelDecel(Motor_T * p_motor)
{
	StateMachine_State_T * p_newState;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		p_newState = &MOTOR_STATE_RUN;
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		p_newState = (Motor_SixStep_CheckResumePhaseControl(p_motor) == true) ? &MOTOR_STATE_RUN : 0U; //openloop does not resume
	}

	return p_newState;
}

static const StateMachine_Transition_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_STOP] 		= (StateMachine_Transition_T)TransitionStop,

	[MSM_INPUT_ACCELERATE] 		=  (StateMachine_Transition_T)FreeWheel_InputAccelDecel,
	[MSM_INPUT_DECELERATE] 		=  (StateMachine_Transition_T)FreeWheel_InputAccelDecel,
};

static void FreewheelEntry(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor); //todo resume ramp cmd

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_StartAngleObserve(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseObserve(p_motor);
	}
}

static void FreewheelProc(Motor_T * p_motor)
{
	if(Motor_GetSpeed(p_motor) == 0U)
	{
//		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_STOP);
		StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
	}
	else
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
			//		Motor_FOC_ProcAngleObserve(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
			Motor_SixStep_ProcPhaseObserve(p_motor);
		}
	}
}

static const StateMachine_State_T MOTOR_STATE_FREEWHEEL =
{
	.P_TRANSITION_TABLE 	= FREEWHEEL_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)FreewheelEntry,
	.OUTPUT 				= (StateMachine_Output_T)FreewheelProc,
};

/******************************************************************************/
/*!
    @brief  State shared calibration
*/
/******************************************************************************/
static const StateMachine_Transition_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_STOP] 	= (StateMachine_Transition_T)TransitionStop,
	//cancel on decel?

//	[MSM_TRANSITION_FAULT] 	= {&MOTOR_STATE_FAULT, 	0U},
//	[MSM_TRANSITION_INIT] 	= {&MOTOR_STATE_FAULT, 	0U},
//	[MSM_TRANSITION_ALIGN] 	= {&MOTOR_STATE_FAULT, 	0U},
//	[MSM_TRANSITION_RUN] 		= {&MOTOR_STATE_FAULT, 	0U},
};

static void CalibrationEntry(Motor_T * p_motor)
{
	switch (p_motor->CalibrationState)
	{
		case MOTOR_CALIBRATION_STATE_ADC:
			if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
			{
		//			Motor_FOC_StartCalibrateAdc(p_motor);
			}
			else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
			{
				Motor_StartCalibrateAdc(p_motor);
			}
			break;

		case MOTOR_CALIBRATION_STATE_HALL:
			Motor_StartCalibrateHall(p_motor);
			break;

		case MOTOR_CALIBRATION_STATE_ENCODER:
			Motor_StartCalibrateEncoder(p_motor);
			break;

		default:
			break;
	}
}

static void CalibrationProc(Motor_T * p_motor)
{
	bool isComplete = false;

	switch (p_motor->CalibrationState)
	{
		case MOTOR_CALIBRATION_STATE_ADC:		isComplete = Motor_CalibrateAdc(p_motor); 		break;
		case MOTOR_CALIBRATION_STATE_HALL:		isComplete = Motor_CalibrateHall(p_motor); 		break;
		case MOTOR_CALIBRATION_STATE_ENCODER:	isComplete = Motor_CalibrateEncoder(p_motor); 	break;
		default: break;
	}

	if(isComplete == true)
	{
//		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_TRANSITION_STOP);
		StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
	}
}

static const StateMachine_State_T MOTOR_STATE_CALIBRATION  =
{
	.P_TRANSITION_TABLE 	= CALIBRATION_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)CalibrationEntry,
	.OUTPUT 				= (StateMachine_Output_T)CalibrationProc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
//	[MSM_TRANSITION_STOP] 	= (StateMachine_Transition_T)TransitionStop,
};

static void Fault_Entry(Motor_T * p_motor)
{

}

static void Fault_Proc(Motor_T * p_motor)
{
	//if fault clears	StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
}

static const StateMachine_State_T MOTOR_STATE_FAULT =
{
	.P_TRANSITION_TABLE 	= FAULT_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
