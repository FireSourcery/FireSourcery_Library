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
#include "MotorStateMachine.h"

#include "Utility/StateMachine/StateMachine.h"

#include "Motor.h"
#include "Motor_FOC.h"
#include "Motor_SixStep.h"

static const StateMachine_State_T MOTOR_STATE_INIT;
static const StateMachine_State_T MOTOR_STATE_STOP;
static const StateMachine_State_T MOTOR_STATE_ALIGN;
static const StateMachine_State_T MOTOR_STATE_OPEN_LOOP;
static const StateMachine_State_T MOTOR_STATE_RUN;
static const StateMachine_State_T MOTOR_STATE_FREEWHEEL;
static const StateMachine_State_T MOTOR_STATE_CALIBRATION;
static const StateMachine_State_T MOTOR_STATE_FAULT;

//Temp todo
#include "Motor/MotorController/MotorController.h"
#include "Motor/Utility/MotAnalogUser/MotAnalogUser.h"
extern void MotAnalogUser_Motor_Write(const MotAnalogUser_T * p_user, Motor_T * p_motorDest, uint8_t motorCount);
extern MotorController_T MotorControllerMain;

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
const StateMachine_Machine_T MOTOR_STATE_MACHINE =
{
	.P_STATE_INITIAL 			= &MOTOR_STATE_INIT,
	.TRANSITION_TABLE_LENGTH 	= MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH,
	.OUTPUT_TABLE_LENGTH 		= MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH,
//	.IS_MULTITHREADED 			= 0U,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void InitEntry(Motor_T * p_motor)
{
//	Motor_InitReboot(p_motor);
}

static void InitProc(Motor_T * p_motor)
{
	//temp
	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_CALIBRATE_HALL);
	}
	else
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}

//		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_CALIBRATE_ADC);
}

static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_STOP] 			= {&MOTOR_STATE_STOP, 			0U},

	[MOTOR_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_INIT] 			= {&MOTOR_STATE_INIT, 			0U},
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= {&MOTOR_STATE_CALIBRATION, 	0U},
	[MOTOR_TRANSITION_CALIBRATE_HALL] 	= {&MOTOR_STATE_CALIBRATION, 	(StateMachine_Output_T)Motor_SetCalibrationStateHall},
	[MOTOR_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_RUN] 				= {&MOTOR_STATE_FAULT, 			0U},
};

static const StateMachine_Output_T INIT_OUTPUT_TABLE[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{

};

static const StateMachine_State_T MOTOR_STATE_INIT =
{
	.P_TRANSITION_TABLE		= &INIT_TRANSITION_TABLE[0U],
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)INIT_OUTPUT_TABLE,
	.ON_ENTRY 				= (StateMachine_Output_T)InitEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)InitProc,
};


/******************************************************************************/
/*!
    @brief  State
    todo long term stop
*/
/******************************************************************************/
static void StopEntry(Motor_T * p_motor)
{
	Motor_Float(p_motor);
	p_motor->VPwm = 0U;
	p_motor->ControlTimerBase = 0U; //ok to reset timer
}

static void StopProc(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->MillisTimer) == true)
	{
		MotAnalogUser_Motor_Write(&MotorControllerMain.AnalogUser, p_motor, 1);
	}

	if (p_motor->IsActiveControl == true)
	{
//		if (Motor_GetAlignMode(p_motor) == MOTOR_ALIGN_MODE_DISABLE)
//		{
//			if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
//			{
//				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_OPEN_LOOP);
//			}
//			else
//			{
//				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
//			}
//		}
//		else
//		{
//			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_ALIGN);
//		}

//		if (Motor_GetAlignMode(p_motor) == MOTOR_ALIGN_MODE_DISABLE)
//		{
			if ((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP))
			{
				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_ALIGN);
			}
			else
			{
				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
			}
//		}
//		else
//		{
//			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_ALIGN);
//		}
	}
	else
	{
		Motor_PollAnalogStartAll(p_motor);
	}

	// proc direction
	//		Motor_PollToggleDirectionUpdate(p_motor);

	//proc flash
	//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_LOCK);

}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_ALIGN] 			= {&MOTOR_STATE_ALIGN, 			0U},
	[MOTOR_TRANSITION_RUN] 				= {&MOTOR_STATE_RUN, 			0U},
	[MOTOR_TRANSITION_OPEN_LOOP] 		= {&MOTOR_STATE_OPEN_LOOP,		0U},

	[MOTOR_TRANSITION_CALIBRATE_ADC] 		= {&MOTOR_STATE_CALIBRATION, 	(StateMachine_Output_T)Motor_SetCalibrationStateAdc},
	[MOTOR_TRANSITION_CALIBRATE_HALL] 		= {&MOTOR_STATE_CALIBRATION, 	(StateMachine_Output_T)Motor_SetCalibrationStateHall},
	[MOTOR_TRANSITION_CALIBRATE_ENCODER] 	= {&MOTOR_STATE_CALIBRATION, 	(StateMachine_Output_T)Motor_SetCalibrationStateEncoder},

	[MOTOR_TRANSITION_FAULT] 				= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_INIT] 				= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_STOP] 				= {&MOTOR_STATE_FAULT, 			0U},
};


static const StateMachine_Output_T STOP_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{
//set align on not brake

};

static const StateMachine_State_T MOTOR_STATE_STOP =
{
	.P_TRANSITION_TABLE 	= STOP_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)STOP_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 				= (StateMachine_Output_T)StopEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)StopProc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void AlignEntry(Motor_T * p_motor)
{
	Motor_StartAlign(p_motor);
}

static void AlignProc(Motor_T * p_motor)
{
	if(Motor_ProcAlign(p_motor))
	{
		if((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF) || (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP))
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_OPEN_LOOP);
		}
		else
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
		}
	}
}

static const StateMachine_Transition_T ALIGN_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_RUN] 				= {&MOTOR_STATE_RUN, 			0U},
	[MOTOR_TRANSITION_OPEN_LOOP] 		= {&MOTOR_STATE_OPEN_LOOP, 		0U},

	[MOTOR_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_INIT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_STOP] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
};

static const StateMachine_Output_T ALIGN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{

};

static const StateMachine_State_T MOTOR_STATE_ALIGN =
{
	.P_TRANSITION_TABLE 	= ALIGN_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)ALIGN_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 				= (StateMachine_Output_T)AlignEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)AlignProc,
};

/******************************************************************************/
/*!
    @brief  State OpenProc
*/
/******************************************************************************/
static void OpenProcEntry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_StartOpenProc(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseControl(p_motor, MOTOR_SENSOR_MODE_OPEN_LOOP);
	}

	p_motor->Bemf.ZeroCrossingCounter = 0U;
}

static void OpenLoopProc(Motor_T * p_motor)
{
	if (p_motor->IsActiveControl == false)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FREEWHEEL);
	}
	else
	{
		if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
//			Motor_FOC_ProcAngleControl(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
			Motor_SixStep_ProcPhaseControl(p_motor, MOTOR_SENSOR_MODE_OPEN_LOOP);

			if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
			{
				if (Motor_SixStep_GetBemfReliable(p_motor))
				{
					StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
				}
			}
		}
	}
}

static const StateMachine_Transition_T OPEN_LOOP_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_RUN] 				= {&MOTOR_STATE_RUN, 			0U},
	[MOTOR_TRANSITION_FREEWHEEL] 		= {&MOTOR_STATE_FREEWHEEL, 		0U},

	[MOTOR_TRANSITION_STOP] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_INIT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
};

static const StateMachine_Output_T OPEN_LOOP_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{
//	&OpenProcInputAll,
};

static const StateMachine_State_T MOTOR_STATE_OPEN_LOOP =
{
	.P_TRANSITION_TABLE 	= OPEN_LOOP_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)OPEN_LOOP_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 				= (StateMachine_Output_T)OpenProcEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)OpenLoopProc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void RunEntry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_StartAngleControl(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseControl(p_motor, p_motor->Parameters.SensorMode);

//		if ((p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF) && Motor_SixStep_GetBemfReliable(p_motor) == false)
//		{
//			Motor_SixStep_StartPhaseControl(p_motor, MOTOR_SENSOR_MODE_OPEN_LOOP);
//		}
//		else
//		{
//			Motor_SixStep_StartPhaseControl(p_motor, p_motor->Parameters.SensorMode);
//		}

	}
}

static void RunProc(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->MillisTimer) == true)
	{
		MotAnalogUser_Motor_Write(&MotorControllerMain.AnalogUser, p_motor, 1);
	}


	if (p_motor->IsActiveControl == false)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FREEWHEEL);
	}
	else
	{
		if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
			Motor_FOC_ProcAngleControl(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
			Motor_SixStep_ProcPhaseControl(p_motor, p_motor->Parameters.SensorMode);
		}

//		if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
//		{
//			if (Motor_SixStep_GetBemfReliable(p_motor) == false)
//			{
//				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FREEWHEEL);
//			}
//		}
	}
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_STOP] 			= {&MOTOR_STATE_FREEWHEEL, 		0U},
	[MOTOR_TRANSITION_RUN] 				= {&MOTOR_STATE_RUN, 			0U},
	[MOTOR_TRANSITION_FREEWHEEL] 		= {&MOTOR_STATE_FREEWHEEL, 		0U},

	[MOTOR_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_INIT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
};

static const StateMachine_Output_T RUN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{
//	&RunInputAll,
};

static const StateMachine_State_T MOTOR_STATE_RUN =
{
	.P_TRANSITION_TABLE 		= RUN_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 			= (StateMachine_Output_T *)RUN_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 					= (StateMachine_Output_T)RunEntry,
	.OUTPUT_SYNCHRONOUS 		= (StateMachine_Output_T)RunProc,
};

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void FreewheelEntry(Motor_T * p_motor)
{
	Motor_Float(p_motor);

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
	if (Timer_Poll(&p_motor->MillisTimer) == true)
	{
		MotAnalogUser_Motor_Write(&MotorControllerMain.AnalogUser, p_motor, 1);
	}

	if (Motor_GetSpeed(p_motor) == 0U)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
	else
	{
		if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
	//		Motor_FOC_ProcAngleObserve(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
			Motor_SixStep_ProcPhaseObserve(p_motor, p_motor->Parameters.SensorMode);
		}

		if (p_motor->IsActiveControl == true)
		{
			if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP)
			{
				//			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_OPEN_LOOP);
				//
				//			if (Motor_GetBemf_Frac16(p_motor) < 5U)
				//			{
				//				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
				//			}
				//			//no transition to freewheel from open loop

				//openloop sensor mode must wait for stop detect
			}
			else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_BEMF)
			{
				if (Motor_SixStep_GetBemfReliable(p_motor))
				{
					StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
				}
				else
				{
					//				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_OPEN_LOOP);

					//bemf sensor mode use last capture to determine if speed == 0, or missed zcd count
					//				if (Motor_GetBemf_Frac16(p_motor) < 5U)
					//				{
					//					StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
					//				}

				}
			}
			else //hall or encoder
			{
				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
			}
		}

	}

}

static const StateMachine_Transition_T FREEWHEEL_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_STOP] 			= {&MOTOR_STATE_STOP, 			0U},
	[MOTOR_TRANSITION_RUN] 				= {&MOTOR_STATE_RUN, 			0U},
//	[MOTOR_TRANSITION_OPEN_LOOP] 		= {&MOTOR_STATE_OPEN_LOOP, 		0U},

	[MOTOR_TRANSITION_FAULT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_INIT] 			= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= {&MOTOR_STATE_FAULT, 			0U},
	[MOTOR_TRANSITION_ALIGN] 			= {&MOTOR_STATE_FAULT, 			0U},
};

static const StateMachine_Output_T FREEWHEEL_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{

};

static const StateMachine_State_T MOTOR_STATE_FREEWHEEL =
{
	.P_TRANSITION_TABLE 	= FREEWHEEL_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)FREEWHEEL_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 				= (StateMachine_Output_T)FreewheelEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)FreewheelProc,
};

/******************************************************************************/
/*!
    @brief  State shared calibration
*/
/******************************************************************************/
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
	}

	if (isComplete == true) {StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);}
}

static const StateMachine_Transition_T CALIBRATION_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_STOP] 	= {&MOTOR_STATE_STOP, 	0U},

	[MOTOR_TRANSITION_FAULT] 	= {&MOTOR_STATE_FAULT, 	0U},
	[MOTOR_TRANSITION_INIT] 	= {&MOTOR_STATE_FAULT, 	0U},
	[MOTOR_TRANSITION_ALIGN] 	= {&MOTOR_STATE_FAULT, 	0U},
	[MOTOR_TRANSITION_RUN] 		= {&MOTOR_STATE_FAULT, 	0U},
};

static const StateMachine_Output_T CALIBRATION_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{

};

static const StateMachine_State_T MOTOR_STATE_CALIBRATION  =
{
	.P_TRANSITION_TABLE 	= CALIBRATION_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)CALIBRATION_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 				= (StateMachine_Output_T)CalibrationEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)CalibrationProc,
};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static void FaultEntry(Motor_T * p_motor)
{

}

static void FaultProc(Motor_T * p_motor)
{
	//if fault clears	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MOTOR_STATE_MACHINE_TRANSITION_TABLE_LENGTH] =
{
	[MOTOR_TRANSITION_INIT] = {&MOTOR_STATE_INIT, 0U},
};

static const StateMachine_Output_T FAULT_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_OUTPUT_TABLE_LENGTH] =
{

};

static const StateMachine_State_T MOTOR_STATE_FAULT =
{
	.P_TRANSITION_TABLE 	= FAULT_TRANSITION_TABLE,
	.P_OUTPUT_TABLE 		= (StateMachine_Output_T *)FAULT_OUTPUT_FUNCTION_MAP,
	.ON_ENTRY 				= (StateMachine_Output_T)FaultEntry,
	.OUTPUT_SYNCHRONOUS 	= (StateMachine_Output_T)FaultProc,
};
