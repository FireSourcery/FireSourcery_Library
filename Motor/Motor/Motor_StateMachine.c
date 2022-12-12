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
	@file 	MotorStateMachine.c
	@author FireSourcery
	@brief  MotorStateMachine
	@version V0
*/
/******************************************************************************/
#include "Motor_StateMachine.h"
#include "Motor_Calibrate.h"
#include "Motor_FOC.h"
//#include "Motor_SixStep.h"
#include "Motor.h"
#include "Utility/StateMachine/StateMachine.h"

static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_STOP;
static const StateMachine_State_T STATE_RUN;
static const StateMachine_State_T STATE_FREEWHEEL;
static const StateMachine_State_T STATE_ALIGN;
static const StateMachine_State_T STATE_OPEN_LOOP;
static const StateMachine_State_T STATE_CALIBRATION;
static const StateMachine_State_T STATE_FAULT;

/******************************************************************************/
/*!
	@brief State Machine
*/
/******************************************************************************/
const StateMachine_Machine_T MSM_MACHINE =
{
	.P_STATE_INITIAL = &STATE_INIT,
	.TRANSITION_TABLE_LENGTH = MSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(Motor_T * p_motor, uint32_t voidVar) { (void)p_motor; (void)voidVar; return &STATE_FAULT; }
static StateMachine_State_T * TransitionFreewheel(Motor_T * p_motor, uint32_t voidVar) { (void)p_motor;	(void)voidVar;	return &STATE_FREEWHEEL; }
/******************************************************************************/
/*!
	@brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL] 		= 0U,
	[MSM_INPUT_RELEASE] 		= 0U,
	[MSM_INPUT_DIRECTION] 		= 0U,
	[MSM_INPUT_CALIBRATION] 	= 0U,
};

static void Init_Entry(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_SetDirectionForward(p_motor); /* Alleviate circular inclusion form Motor_Init */
	}
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */
	{
		// Motor_SixStep_SetDirectionForward(p_motor);
	}
}

static void Init_Proc(Motor_T * p_motor)
{
	if(Timer_GetElapsed_Millis(&p_motor->ControlTimer) > 5U) /* Wait for thermistor Adc */
	{
		_StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP);
	}
}

static const StateMachine_State_T STATE_INIT =
{
	.ID 					= MSM_STATE_ID_INIT,
	.P_TRANSITION_TABLE 	= &INIT_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Init_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Init_Proc,
};


/******************************************************************************/
/*!
	@brief  State

	Motor is in floating state with 0 speed
*/
/******************************************************************************/
static StateMachine_State_T * Stop_InputControl(Motor_T * p_motor, uint32_t voidVar)
{
	(void)voidVar;
	StateMachine_State_T * p_nextState;

	Motor_FOC_SetOutputMatchStop(p_motor);

	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		Motor_ZeroSensor(p_motor);
		p_nextState = &STATE_RUN;
	}
	else
	{
		p_nextState = &STATE_ALIGN;
	}

	return p_nextState;
}

static StateMachine_State_T * Stop_InputDirection(Motor_T * p_motor, uint32_t direction)
{
	if(p_motor->SpeedFeedback_Frac16 == 0U)
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) 	{ Motor_FOC_SetDirection(p_motor, direction); }
		else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ 	{ Motor_SetDirection(p_motor, direction); }
	}

	return 0U;
}

static StateMachine_State_T * Stop_InputCalibration(Motor_T * p_motor, uint32_t state)
{
	(void)p_motor;
	p_motor->CalibrationState = state;
	return &STATE_CALIBRATION;
}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL] 		= (StateMachine_Transition_T)Stop_InputControl,
	[MSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Stop_InputDirection,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)Stop_InputCalibration,
	[MSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)0U,
	//	[MSM_INPUT_GROUND] 			= (StateMachine_Transition_T)Stop_InputGround, //only ground in stop mode?
};

/*
	Enters upon reaching 0 Speed
*/
static void Stop_Entry(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	p_motor->ControlTimerBase = 0U; /* ok to reset timer */
	p_motor->FeedbackModeFlags.Update = 1U; /* Check outside before MSM_INPUT_CONTROL. Next Motor_User_SetCmdMode call will proc MSM_INPUT_CONTROL to run state. */
}

static void Stop_Proc(Motor_T * p_motor)
{
	//	if(p_motor->SpeedFeedback_Frac16 > 0U) 	// or use bemf
	//	{
	////		_StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_FREEWHEEL);
	//	}
	//	else
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ProcAngleObserve(p_motor); }
		else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ {}
	}
}

static const StateMachine_State_T STATE_STOP =
{
	.ID 					= MSM_STATE_ID_STOP,
	.P_TRANSITION_TABLE 	= &STOP_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Stop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Stop_Proc,
};

/******************************************************************************/
/*!
	@brief  Run State

	Active control, accelerate or decelerate
	UserCmd/RampCmd value is in effect
*/
/******************************************************************************/
static StateMachine_State_T * Run_InputControl(Motor_T * p_motor, uint32_t voidVar)
{
	(void)voidVar;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_SetOutputMatchRun(p_motor); }
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ {}
#endif
	return 0U;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 		= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL] 	= (StateMachine_Transition_T)Run_InputControl,
	[MSM_INPUT_RELEASE] 	= (StateMachine_Transition_T)TransitionFreewheel,
};

static void Run_Entry(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_StartAngleControl(p_motor); }
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ {}
}

static void Run_Proc(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ProcAngleControl(p_motor);
	}
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		//		Motor_SixStep_ProcPhaseControl(p_motor);

		// if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SENSORLESS)
		// {
			//			if (Motor_SixStep_GetBemfReliable(p_motor) == false)
			//			{
			//				StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_FREEWHEEL);
			//			}
		// }
	}
#endif
}

static const StateMachine_State_T STATE_RUN =
{
	.ID 					= MSM_STATE_ID_RUN,
	.P_TRANSITION_TABLE 	= RUN_TRANSITION_TABLE,
	.ENTRY 					= (StateMachine_Output_T)Run_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Run_Proc,
};

/******************************************************************************/
/*!
	@brief  State
*/
/******************************************************************************/
static StateMachine_State_T * Freewheel_InputControl(Motor_T * p_motor, uint32_t voidVar)
{
	(void)voidVar;
	StateMachine_State_T * p_newState = 0U;

	if(p_motor->FeedbackModeFlags.OpenLoop == 1U) /* if need feedback mode */
	{
		p_newState = 0U; /* openloop does not resume */
	}
	else
	{
		if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) 				{ Motor_FOC_SetOutputMatchFreewheel(p_motor); }
		else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ 	{ /* Motor_SixStep_ResumePhaseControl(p_motor); */ }
		p_newState = &STATE_RUN;
	}

	return p_newState;
}

static const StateMachine_Transition_T FREEWHEEL_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_CONTROL] 		= (StateMachine_Transition_T)Freewheel_InputControl,
	[MSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)0U,
};

static void Freewheel_Entry(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	p_motor->FeedbackModeFlags.Update = 1U; /* Control inactive, call MSM_INPUT_CONTROL on SetCmd  */

	// if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) {Motor_FOC_StartAngleObserve(p_motor); }
	// else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP {	Motor_SixStep_StartPhaseObserve(p_motor); }
}

static void Freewheel_Proc(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ProcAngleObserve(p_motor); }
	else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ { /* Motor_SixStep_ProcPhaseObserve(p_motor); */ }

	/* Check after capture speed, this way lower priority input cannot proc in between capture and check */
	if(p_motor->SpeedFeedback_Frac16 == 0U) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP); }
}

static const StateMachine_State_T STATE_FREEWHEEL =
{
	.ID 					= MSM_STATE_ID_FREEWHEEL,
	.P_TRANSITION_TABLE 	= FREEWHEEL_TRANSITION_TABLE,
	.ENTRY 					= (StateMachine_Output_T)Freewheel_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Freewheel_Proc,
};


/******************************************************************************/
/*!
	@brief  State Align - todo common start control?
*/
/******************************************************************************/
// Motor_AlignMode_T GetAlignMode(Motor_T *p_motor)
// {
// 	Motor_AlignMode_T alignMode;

// 	// if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
// 	// {
// 	// 	alignMode = MOTOR_ALIGN_MODE_DISABLE;
// 	// }
// 	// else
// 	{
// 		// alignMode = p_motor->Parameters.AlignMode;
// 		alignMode = MOTOR_ALIGN_MODE_ALIGN;
// 	}

// 	return alignMode;
// }

static const StateMachine_Transition_T ALIGN_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)TransitionFreewheel,
	[MSM_INPUT_CONTROL] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)0U,
};

static void Align_Entry(Motor_T * p_motor)
{
	// switch(p_motor->Parameters.AlignMode)
	// {

	// }
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
	Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U);
	Phase_ActivateSwitchABC(&p_motor->Phase);
}

static void Align_Proc(Motor_T * p_motor)
{
	if(Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
	{
		Motor_ZeroSensor(p_motor);
		p_motor->ElectricalAngle = 0U;

		if((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SENSORLESS) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP))
		{
			_StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_OPEN_LOOP);
		}
		else
		{
			_StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_RUN);
		}
	}
}

static const StateMachine_State_T STATE_ALIGN =
{
	.ID 					= MSM_STATE_ID_ALIGN,
	.P_TRANSITION_TABLE 	= &ALIGN_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Align_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Align_Proc,
};

/******************************************************************************/
/*!
	@brief  State OpenLoop
*/
/******************************************************************************/
static const StateMachine_Transition_T OPEN_LOOP_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_RELEASE] 	= (StateMachine_Transition_T)TransitionFreewheel, /* No resume from OpenLoop, freewheel state check stop */
	[MSM_INPUT_FAULT] 		= (StateMachine_Transition_T)TransitionFault,
};

static void OpenLoop_Entry(Motor_T * p_motor)
{
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_StartAngleControl(p_motor); }
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else /* p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ { Motor_SixStep_StartPhaseControl(p_motor); }
#endif
}

static void OpenLoop_Proc(Motor_T * p_motor)
{
	Motor_ProcRamp(p_motor);

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) { Motor_FOC_ProcAngleControl(p_motor); }
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcPhaseControl(p_motor);

		if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SENSORLESS)
		{
			if(Motor_SixStep_GetBemfReliable(p_motor) == true)
			{
				StateMachine_ProcTransition(&p_motor->StateMachine, &STATE_RUN);
			}
		}
	}
#endif
}

static const StateMachine_State_T STATE_OPEN_LOOP =
{
	.ID 					= MSM_STATE_ID_OPEN_LOOP,
	.P_TRANSITION_TABLE 	= OPEN_LOOP_TRANSITION_TABLE,
	.ENTRY 					= (StateMachine_Output_T)OpenLoop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)OpenLoop_Proc,
};

/******************************************************************************/
/*!
	@brief Calibration State

	Functions defined in Motor_Calibrate.h for readability
*/
/******************************************************************************/
static StateMachine_State_T * Calibration_InputRelease(Motor_T * p_motor, uint32_t voidVar)
{
	(void)p_motor; 	(void)voidVar;
	return &STATE_STOP;
}

static const StateMachine_Transition_T CALIBRATION_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)Calibration_InputRelease,
	[MSM_INPUT_CONTROL] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)0U,
};

static void Calibration_Entry(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	p_motor->CalibrationStateIndex = 0U;
	Phase_Ground(&p_motor->Phase);

	switch(p_motor->CalibrationState)
	{
		case MOTOR_CALIBRATION_STATE_ADC:		Motor_Calibrate_StartAdc(p_motor);		break;
		case MOTOR_CALIBRATION_STATE_HALL:		Motor_Calibrate_StartHall(p_motor);		break;
		case MOTOR_CALIBRATION_STATE_ENCODER:	Motor_Calibrate_StartEncoder(p_motor);	break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_CALIBRATION_STATE_SIN_COS:	Motor_Calibrate_StartSinCos(p_motor);	break;
#endif
		default: break;
	}
}

static void Calibration_Proc(Motor_T * p_motor)
{
	bool isComplete = false;

	switch(p_motor->CalibrationState)
	{
		case MOTOR_CALIBRATION_STATE_ADC:		isComplete = Motor_Calibrate_Adc(p_motor); 			break;
		case MOTOR_CALIBRATION_STATE_HALL:		isComplete = Motor_Calibrate_Hall(p_motor); 		break;
		case MOTOR_CALIBRATION_STATE_ENCODER:	isComplete = Motor_Calibrate_Encoder(p_motor); 		break;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
		case MOTOR_CALIBRATION_STATE_SIN_COS:	isComplete = Motor_Calibrate_SinCos(p_motor);		break;
#endif
		default: break;
	}

	if(isComplete == true) { _StateMachine_ProcStateTransition(&p_motor->StateMachine, &STATE_STOP); }
}

static const StateMachine_State_T STATE_CALIBRATION =
{
	.ID 					= MSM_STATE_ID_CALIBRATION,
	.P_TRANSITION_TABLE 	= CALIBRATION_TRANSITION_TABLE,
	.ENTRY 					= (StateMachine_Output_T)Calibration_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Calibration_Proc,
};


/******************************************************************************/
/*!
	@brief  State
*/
/******************************************************************************/
static StateMachine_State_T * Fault_InputFault(Motor_T * p_motor, uint32_t voidVar)
{
	(void)voidVar;
	bool isClear = true;
	if(Thermistor_GetIsShutdown(&p_motor->Thermistor) == true) { isClear = false; }
	return (isClear == true) ? &STATE_STOP : 0U;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MSM_TRANSITION_TABLE_LENGTH] =
{
	[MSM_INPUT_FAULT] 			= (StateMachine_Transition_T)Fault_InputFault,
	[MSM_INPUT_CONTROL] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)0U,
	[MSM_INPUT_CALIBRATION] 	= (StateMachine_Transition_T)0U,
};

static void Fault_Entry(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

static void Fault_Proc(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase); /* repeat ok */
}

static const StateMachine_State_T STATE_FAULT =
{
	.ID 					= MSM_STATE_ID_FAULT,
	.P_TRANSITION_TABLE 	= FAULT_TRANSITION_TABLE,
	.ENTRY 					= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
