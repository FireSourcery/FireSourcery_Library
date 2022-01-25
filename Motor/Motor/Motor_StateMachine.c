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

static StateMachine_State_T * TransitionFault(Motor_T * p_motor) 		{return &MOTOR_STATE_FAULT;}

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
};

static void Init_Entry(Motor_T * p_motor)
{
//	Motor_InitReboot(p_motor);
}

static void Init_Proc(Motor_T * p_motor)
{
//	StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
	Motor_SetCalibrationStateAdc(p_motor);
	StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_CALIBRATION);
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
	return (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL) ? &MOTOR_STATE_RUN : &MOTOR_STATE_ALIGN;
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
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_ACCELERATE] 		= (StateMachine_Transition_T)Stop_InputAccelerate,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)Stop_InputCalibration,

//	[MSM_INPUT_DETECT_FREEWHEEL] 	= (StateMachine_Transition_T)TransitionFreeWheel,
//	[MSM_INPUT_DECELERATE] 			= (StateMachine_Transition_T)StopTransitionDecelerate,
};

static void Stop_Entry(Motor_T * p_motor)
{
	Motor_Stop(p_motor);
}

static void Stop_Proc(Motor_T * p_motor)
{
	//poll speedfeedback or use bemf
//	if(Motor_GetSpeed(p_motor) > 0U)
//	{
//		StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL);
//	}
//	else
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
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
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
			StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_OPEN_LOOP);
		}
		else
		{
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
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_DECELERATE] 		= (StateMachine_Transition_T)OpenLoop_InputStop,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)OpenLoop_InputStop,
};

static void OpenLoop_Entry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_StartAngleControl(p_motor);
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
		Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcPhaseControl(p_motor);

		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
		{
			if (Motor_SixStep_GetBemfReliable(p_motor) == true)
			{
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
//static StateMachine_State_T * Run_InputAccelerate(Motor_T * p_motor)
//{
//	//check over current time
//
//	return 0U;
//}
//
//static StateMachine_State_T * Run_InputDecelerate(Motor_T * p_motor)
//{
// check regen over volt
//	return 0U;
//}

static StateMachine_State_T * Run_InputFloat(Motor_T * p_motor)
{
	return &MOTOR_STATE_FREEWHEEL;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
//	[MSM_INPUT_ACCELERATE] 		= (StateMachine_Transition_T)Run_InputAccelerate,
//	[MSM_INPUT_DECELERATE] 		= (StateMachine_Transition_T)Run_InputDecelerate,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)Run_InputFloat,
};

static void Run_Entry(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		if(Motor_GetSpeed(p_motor) == 0U) //move to module?
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
//				StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_FREEWHEEL);
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
	StateMachine_State_T * p_newState = &MOTOR_STATE_RUN;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		p_newState = &MOTOR_STATE_RUN;
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
		{
			if (Motor_SixStep_GetBemfReliable(p_motor) == false)
			{
				p_newState = 0U;
			}
		}

//		p_newState = (Motor_SixStep_CheckResumePhaseControl(p_motor) == true) ? &MOTOR_STATE_RUN : 0U; //openloop does not resume
	}

	return p_newState;
}

static const StateMachine_Transition_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_ACCELERATE] 		= (StateMachine_Transition_T)FreeWheel_InputAccelDecel,
	[MSM_INPUT_DECELERATE] 		= (StateMachine_Transition_T)FreeWheel_InputAccelDecel,
};

static void Freewheel_Entry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_StartAngleObserve(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseObserve(p_motor);
	}
}

static void Freewheel_Proc(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);

	if(Motor_GetSpeed(p_motor) == 0U)
	{
		StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
	}
	else
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
			Motor_FOC_ProcAngleObserve(p_motor);
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
	.ON_ENTRY 				= (StateMachine_Output_T)Freewheel_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Freewheel_Proc,
};

/******************************************************************************/
/*!
    @brief  State shared calibration
*/
/******************************************************************************/
static StateMachine_State_T * Calibration_InputStop(Motor_T * p_motor)
{
	return &MOTOR_STATE_STOP;
}

static const StateMachine_Transition_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_ACCELERATE] 		= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_DECELERATE] 		= (StateMachine_Transition_T)Calibration_InputStop,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)Calibration_InputStop,
};

static void Calibration_Entry(Motor_T * p_motor)
{
	switch (p_motor->CalibrationState)
	{
		case MOTOR_CALIBRATION_STATE_ADC:		Motor_StartCalibrateAdc(p_motor);		break;
		case MOTOR_CALIBRATION_STATE_HALL:		Motor_StartCalibrateHall(p_motor);		break;
		case MOTOR_CALIBRATION_STATE_ENCODER:	Motor_StartCalibrateEncoder(p_motor);	break;
		default: break;
	}
}

static void Calibration_Proc(Motor_T * p_motor)
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
		StateMachine_ProcTransition(&p_motor->StateMachine, &MOTOR_STATE_STOP);
	}
}

static const StateMachine_State_T MOTOR_STATE_CALIBRATION  =
{
	.P_TRANSITION_TABLE 	= CALIBRATION_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Calibration_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Calibration_Proc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
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
