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

static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_STOP;
static const StateMachine_State_T STATE_ALIGN;
static const StateMachine_State_T STATE_OPEN_LOOP;
static const StateMachine_State_T STATE_RUN;
static const StateMachine_State_T STATE_FREEWHEEL;
static const StateMachine_State_T STATE_CALIBRATION;
static const StateMachine_State_T STATE_FAULT;

//MotorController_StateMachine_StateId_T MotorController_StateMachine_GetStateId(MotorController_T * p_mc)
//{
//	MotorController_StateMachine_StateId_T id;
//	if(p_mc->StateMachine.p_StateActive == &STATE_INIT)
//	{
//		id = MCSM_STATE_ID_INIT;
//	}
//	else if(p_mc->StateMachine.p_StateActive == &STATE_STOP)
//	{
//		id = MCSM_STATE_ID_STOP;
//	}
//	else if(p_mc->StateMachine.p_StateActive == &STATE_RUN)
//	{
//		id = MCSM_STATE_ID_RUN;
//	}
//	else if(p_mc->StateMachine.p_StateActive == &STATE_FAULT)
//	{
//		id = MCSM_STATE_ID_FAULT;
//	}
//	return id;
//}

/******************************************************************************/
/*!
    @brief State Machine
*/
/******************************************************************************/
const StateMachine_Machine_T MSM_MACHINE =
{
	.P_STATE_INITIAL 			= &STATE_INIT,
	.TRANSITION_TABLE_LENGTH 	= MSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(Motor_T * p_motor) 		{return &STATE_FAULT;}

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
	_StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_STOP);
}

static const StateMachine_State_T STATE_INIT =
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
static StateMachine_State_T * Stop_TransitionRun(Motor_T * p_motor)
{
	StateMachine_State_T * p_nextState;

	if (p_motor->UserDirection != p_motor->Direction)
	{
//		p_nextState = &STATE_FAULT; //direction set was called during another mode
		p_nextState = 0U;
	}
	else
	{
		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
		{
			if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
			{
				Motor_FOC_StartAngleControl(p_motor);
			}
			else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
			{

			}

			p_nextState = &STATE_RUN;
		}
		else
		{
			p_nextState = &STATE_ALIGN;
		}
	}

	return p_nextState;
}

static StateMachine_State_T * Stop_InputCalibration(Motor_T * p_motor)
{
	return &STATE_CALIBRATION;
}

static StateMachine_State_T * Stop_InputDirection(Motor_T * p_motor)
{
	StateMachine_State_T * p_nextState;

	if (p_motor->Speed_RPM == 0U)
	{
		Motor_SetDirection(p_motor, p_motor->UserDirection);
		p_nextState = 0U;
	}
	else
	{
		p_nextState = &STATE_FAULT;
	}

	return p_nextState;
}


static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL_MODE] 	= (StateMachine_Transition_T)Stop_TransitionRun,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)Stop_InputCalibration,
	[MSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Stop_InputDirection,
//	[MSM_INPUT_GROUND] 			= (StateMachine_Transition_T)Stop_InputGround, //only ground in stop mode?
};

/*
 * Enters upon reaching 0 Speed
 */
static void Stop_Entry(Motor_T * p_motor)
{
	Motor_EnterStop(p_motor);

	p_motor->ControlModeFlags.Update = 1U; /* This way next cmd call will set to run state, share state input */

	if (p_motor->UserDirection != p_motor->Direction)
	{
		_StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_FAULT); //direction was set during another mode
	}

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_ResetOutput( p_motor); //set on resume transition
	}
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
	{

	}
}

static void Stop_Proc(Motor_T * p_motor)
{
	//poll speedfeedback or use bemf
//	if(Motor_GetSpeed(p_motor) > 0U)
//	{
//		StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_FREEWHEEL);
//	}
//	else
	{
		Motor_ProcStop(p_motor);
	}
}

static const StateMachine_State_T STATE_STOP =
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
/*
 * run if cmd mode changed
 * change mode must check state, freewheel to run and run self transition match different output
 * change during run mode, output voltage is known, match to vq
 */
static StateMachine_State_T * Run_TransitionRun(Motor_T * p_motor)
{
	StateMachine_State_T * p_newState = 0;

//	if (p_motor->UserDirection != p_motor->Direction)
//	{
//		p_newState = &STATE_FAULT;
//	}
//	else
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
			Motor_FOC_SetMatchOutputKnown(p_motor);
		}
		else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
		{

		}

//		p_motor->ControlModeFlags.Update = 0U;
	}

	return p_newState;
}


static StateMachine_State_T * Run_InputFloat(Motor_T * p_motor)
{
	return &STATE_FREEWHEEL;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL_MODE]	= (StateMachine_Transition_T)Run_TransitionRun,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)Run_InputFloat,
};

static void Run_Entry(Motor_T * p_motor)
{
//	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
//	{
////		Motor_FOC_EnterRunState(p_motor);
//	}
//	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
//	{
//		if(Motor_GetSpeed(p_motor) == 0U)
//		{
//			Motor_SixStep_StartPhaseControl(p_motor);
//		}
//		else
//		{
//			Motor_SixStep_ResumePhaseControl(p_motor);
//		}
//	}
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

		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SENSORLESS)
		{
//			if (Motor_SixStep_GetBemfReliable(p_motor) == false)
//			{
//				StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_FREEWHEEL);
//			}
		}
	}
}

//static StateMachine_State_T *  Run_TransitionFunction(Motor_T * p_motor, MotorStateMachine_Input_T inputMode, uint32_t var)
//{
//	StateMachine_State_T * p_newState = 0U;
//
//	//jump table smaller/faster than function pointer?
//	switch(inputMode)
//	{
//		case MSM_INPUT_FAULT:
//		case MSM_INPUT_CONTROL_MODE:
//			if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
//			{
//				Motor_FOC_SetMatchOutputKnown(p_motor);
//			}
//			else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
//			{
//
//			}
//		case MSM_INPUT_FLOAT:
//		default:			break;
//	}
//
//	return p_newState;
//}

static const StateMachine_State_T STATE_RUN =
{
	.P_TRANSITION_TABLE 	= RUN_TRANSITION_TABLE,
//	.P_TRANSITION_INPUT 	= (StateMachine_TransitionInput_T)Run_TransitionFunction,
	.ON_ENTRY 				= (StateMachine_Output_T)Run_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Run_Proc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
/*
 * match vq to bemf if available or speed scalar
 */
static StateMachine_State_T * FreeWheel_TransitionRun(Motor_T * p_motor)
{
	StateMachine_State_T * p_newState = 0;

//	if (p_motor->UserDirection != p_motor->Direction)
//	{
//		p_newState = &STATE_FAULT;
//	}
//	else
	if (p_motor->ControlModeFlags.OpenLoop == 1U)
	{
		p_newState = 0U; //openloop does not resume
	}
	else
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
			Motor_FOC_ResumeAngleControl(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
//			Motor_SixStep_ResumePhaseControl(p_motor);
		}

//		p_motor->ControlModeFlags.Update = 0U;
		p_newState = &STATE_RUN;
	}

	return p_newState;
}


static const StateMachine_Transition_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL_MODE]	= (StateMachine_Transition_T)FreeWheel_TransitionRun,
};

static void Freewheel_Entry(Motor_T * p_motor)
{
	p_motor->ControlModeFlags.Update = 1U; //need to update on exit

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

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ProcAngleObserve(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcPhaseObserve(p_motor);
	}

	/* Check after, this way lower priority input cannot proc until stop state  */
	if(Motor_GetSpeed(p_motor) == 0U)
	{
		_StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_STOP);
	}
}

static const StateMachine_State_T STATE_FREEWHEEL =
{
	.P_TRANSITION_TABLE 	= FREEWHEEL_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Freewheel_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Freewheel_Proc,
};


/******************************************************************************/
/*!
    @brief  State Align - todo common start control?
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
		if((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SENSORLESS) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP))
		{
			_StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_OPEN_LOOP);
		}
		else
		{
			if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
			{
				Motor_FOC_StartAngleControl(p_motor);
			}
			else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
			{

			}

			_StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_RUN);
		}
	}
}

static const StateMachine_State_T STATE_ALIGN =
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
	return &STATE_FREEWHEEL; //No resume from OpenLoop, freewheel state check stop
}

static const StateMachine_Transition_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
//	[MSM_INPUT_RUN_CMD] 		= (StateMachine_Transition_T)OpenLoop_InputStop,
	[MSM_INPUT_FLOAT] 			= (StateMachine_Transition_T)OpenLoop_InputStop,
};

static void OpenLoop_Entry(Motor_T * p_motor)
{
//	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
//	{
//		Motor_FOC_StartAngleControl(p_motor);
//	}
//	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
//	{
//		Motor_SixStep_StartPhaseControl(p_motor);
//	}
}

static void OpenLoop_Proc(Motor_T * p_motor)
{
//	Motor_ProcRamp(p_motor);
//
//	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
//	{
//		Motor_FOC_ProcAngleControl(p_motor);
//	}
//	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
//	{
//		Motor_SixStep_ProcPhaseControl(p_motor);
//
//		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SENSORLESS)
//		{
//			if (Motor_SixStep_GetBemfReliable(p_motor) == true)
//			{
//				StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_RUN);
//			}
//		}
//	}
}

static const StateMachine_State_T STATE_OPEN_LOOP =
{
	.P_TRANSITION_TABLE 	= OPEN_LOOP_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)OpenLoop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)OpenLoop_Proc,
};

/******************************************************************************/
/*!
    @brief  State shared calibration
*/
/******************************************************************************/
static StateMachine_State_T * Calibration_InputStop(Motor_T * p_motor)
{
	return &STATE_STOP;
}

static const StateMachine_Transition_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT]			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL_MODE]	= (StateMachine_Transition_T)Calibration_InputStop,
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
		_StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_STOP);
	}
}

static const StateMachine_State_T STATE_CALIBRATION  =
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
/*
 * Manual Check Fault
 */
static StateMachine_State_T * Fault_InputFault(Motor_T * p_motor)
{
	return (p_motor->ErrorFlags.State == 0U) ? &STATE_STOP : 0U;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)Fault_InputFault,
};

static void Fault_Entry(Motor_T * p_motor)
{

}

static void Fault_Proc(Motor_T * p_motor)
{
//	bool errorCleared = false; //need?
	Phase_Float(&p_motor->Phase); /* repeat ok */

	if(Thermistor_GetStatus(&p_motor->Thermistor) == THERMISTOR_THRESHOLD_OK)
	{
		p_motor->ErrorFlags.OverHeat = 0U;
	}

//	if (p_motor->ErrorFlags.OverHeat = 1U)
//	{
//		if(Thermistor_GetStatus(&p_motor->Thermistor) == THERMISTOR_THRESHOLD_OK)
//		{
//			p_motor->ErrorFlags.OverHeat = 0U;
//			errorCleared = true;
//		}
//	}

//	if ((p_motor->ErrorFlags.State == 0U) && (errorCleared == true))
//	{
//		StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_STOP);
//	}
}

static const StateMachine_State_T STATE_FAULT =
{
	.P_TRANSITION_TABLE 	= FAULT_TRANSITION_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
