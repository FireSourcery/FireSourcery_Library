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
    @file 	MotorStateMachine.c
    @author FireSoucery
    @brief  MotorStateMachine
    @version V0
*/
/*******************************************************************************/

#include "MotorStateMachine.h"

#include "System/StateMachine/StateMachine.h"

#include "Motor.h"
#include "Motor_FOC.h"
#include "Motor_SixStep.h"

#define MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH 			(10)
#define MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH 				(2) //input output map

extern const State_T MOTOR_STATE_INIT;
extern const State_T MOTOR_STATE_STOP;
extern const State_T MOTOR_STATE_ALIGN;
extern const State_T MOTOR_STATE_OPEN_LOOP;
extern const State_T MOTOR_STATE_SPIN;
extern const State_T MOTOR_STATE_FREEWHEEL;
extern const State_T MOTOR_STATE_FAULT;
extern const State_T MOTOR_STATE_CALIBRATE_ADC;
extern const State_T MOTOR_STATE_CALIBRATE_HALL;
//extern const State_T MOTOR_STATE_CALIBRATE;

//const State_T * const P_INIT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] = 	{[MOTOR_TRANSITION_FAULT] = &MOTOR_STATE_FAULT, 	[MOTOR_TRANSITION_INIT] = &MOTOR_STATE_INIT,	[MOTOR_TRANSITION_STOP] = &MOTOR_STATE_STOP,	[MOTOR_TRANSITION_CALIBRATE_ADC] = &MOTOR_STATE_CALIBRATE_ADC, 	[MOTOR_TRANSITION_CALIBRATE_HALL] = &MOTOR_STATE_CALIBRATE_HALL,	[MOTOR_TRANSITION_ALIGN] = &MOTOR_STATE_FAULT, 	[MOTOR_TRANSITION_SPIN] = &MOTOR_STATE_FAULT};


/*******************************************************************************/
/*!
    @brief Init
*/
/*******************************************************************************/
void MotorStateMachine_Init(Motor_T * p_motor)
{
	//move to motor removes circular dependency
	StateMachine_Init
	(
		&(p_motor->StateMachine),
		&MOTOR_STATE_INIT,
		MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH,
		MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH,
		p_motor
	);

//	StateMachine_Init
//	(
//		&(p_motor->StateMachine),
//		&MOTOR_STATE_FAULT,
//		MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH,
//		MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH,
//		p_motor
//	);
//
//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}


/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void InitEntry(Motor_T * p_motor)
{
	Motor_InitReboot(p_motor);
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

const State_T * const P_INIT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_INIT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_CALIBRATE_ADC,
	[MOTOR_TRANSITION_CALIBRATE_HALL] 	= &MOTOR_STATE_CALIBRATE_HALL,

	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_FAULT,
};

//void (* const INIT_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 			= 0U,
//	[MOTOR_TRANSITION_INIT] 			= 0U,
//	[MOTOR_TRANSITION_STOP] 			= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

//void (* const INIT_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
//{
//
//};

const State_T MOTOR_STATE_INIT =
{
	.PP_INPUT_TRANSITION_STATE_MAP 			= P_INIT_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T*)INIT_TRANSITION_FUNCTION_MAP,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 			= (StateMachine_StateFunction_T*)INIT_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 						= (StateMachine_StateFunction_T)InitEntry,
	.OUTPUT 								= (StateMachine_StateFunction_T)InitProc,
};


/*******************************************************************************/
/*!
    @brief  State long term stop?
*/
/*******************************************************************************/
static void StopEntry(Motor_T * p_motor)
{
	Motor_Float(p_motor);
	p_motor->VPwm = 0U;
	p_motor->ControlTimerBase = 0U; //ok to reset timer
}

static void StopProc(Motor_T * p_motor)
{
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
//				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
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
				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
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

const State_T * const P_STOP_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_ALIGN,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
	[MOTOR_TRANSITION_OPEN_LOOP] 		= &MOTOR_STATE_OPEN_LOOP,

	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_CALIBRATE_ADC,
	[MOTOR_TRANSITION_CALIBRATE_HALL] 	= &MOTOR_STATE_CALIBRATE_HALL,

	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_FAULT,
};

//void (* const STOP_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 			= 0U,
//	[MOTOR_TRANSITION_INIT] 			= 0U,
//	[MOTOR_TRANSITION_STOP] 			= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const STOP_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
{
//set align on not brake


};

const State_T MOTOR_STATE_STOP =
{
	.PP_INPUT_TRANSITION_STATE_MAP 			= P_STOP_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)STOP_TRANSITION_FUNCTION_MAP,
	.P_INPUT_OUTPUT_FUNCTION_MAP 			= (StateMachine_StateFunction_T *)STOP_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 						= (StateMachine_StateFunction_T)StopEntry,
	.OUTPUT 								= (StateMachine_StateFunction_T)StopProc,
};


/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
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
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
		}
	}
}

const State_T * const P_ALIGN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
	[MOTOR_TRANSITION_OPEN_LOOP] 		= &MOTOR_STATE_OPEN_LOOP,

	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
};

//void (* const ALIGN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

//void (* const ALIGN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
//{
//
//};

const State_T MOTOR_STATE_ALIGN =
{
	.PP_INPUT_TRANSITION_STATE_MAP 			= P_ALIGN_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)ALIGN_TRANSITION_FUNCTION_MAP,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 			= (StateMachine_StateFunction_T *)ALIGN_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 						= (StateMachine_StateFunction_T)AlignEntry,
	.OUTPUT 								= (StateMachine_StateFunction_T)AlignProc,
};

/*******************************************************************************/
/*!
    @brief  State OpenProc
*/
/*******************************************************************************/
static void OpenProcEntry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_StartOpenProc(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseControlActive(p_motor, MOTOR_SENSOR_MODE_OPEN_LOOP);
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
					StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
				}
			}
		}
	}
}

const State_T * const P_OPEN_LOOP_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
	[MOTOR_TRANSITION_FREEWHEEL] 		= &MOTOR_STATE_FREEWHEEL,

	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
};

//void (* const SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 			= 0U,
//	[MOTOR_TRANSITION_INIT] 			= 0U,
//	[MOTOR_TRANSITION_STOP] 			= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_OPEN_LOOP] 		= 0U,
//};

//void (*const SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	&OpenProcInputAll,
//};

const State_T MOTOR_STATE_OPEN_LOOP =
{
	.PP_INPUT_TRANSITION_STATE_MAP 		= P_OPEN_LOOP_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)0U,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)0U,
	.TRANSITION_ENTRY 					= (StateMachine_StateFunction_T)OpenProcEntry,
	.OUTPUT 							= (StateMachine_StateFunction_T)OpenLoopProc,
};

/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void SpinEntry(Motor_T * p_motor)
{
//	Motor_SetRamp(p_motor);

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_StartAngleControl(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseControlActive(p_motor, p_motor->Parameters.SensorMode);

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

static void SpinProc(Motor_T * p_motor)
{
	if (p_motor->IsActiveControl == false)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FREEWHEEL);
	}
	else
	{
	//	if (Motor_GetCommutationMode(p_motor) == MOTOR_COMMUTATION_MODE_FOC)
		if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
			Motor_FOC_ProcAngleControl(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
			Motor_SixStep_ProcPhaseControl(p_motor, p_motor->Parameters.SensorMode);
		}
	}
}

//static void SpinInputAll(Motor_T * p_motor)
//{
//
//}

const State_T * const P_SPIN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_FREEWHEEL,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
	[MOTOR_TRANSITION_FREEWHEEL] 		= &MOTOR_STATE_FREEWHEEL,

	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
};

//void (* const SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

//void (*const SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	&SpinInputAll,
//};

const State_T MOTOR_STATE_SPIN =
{
	.PP_INPUT_TRANSITION_STATE_MAP 		= P_SPIN_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)SPIN_TRANSITION_FUNCTION_MAP,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)SPIN_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 					= (StateMachine_StateFunction_T)SpinEntry,
	.OUTPUT 							= (StateMachine_StateFunction_T)SpinProc,
};



/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void FreewheelEntry(Motor_T * p_motor)
{


	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_StartPhaseControlPassive(p_motor);
	}
}

static void FreewheelProc(Motor_T * p_motor)
{
	if (Motor_GetSpeed(p_motor) == 0U)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
	else if (p_motor->IsActiveControl == true)
	{
		if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP)
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
				StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
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
		else
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
		}
	}
	else //(p_motor->IsActiveControl == false)
	{
		if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
		{
	//		Motor_FOC_ProcAngleControl(p_motor);
		}
		else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
		{
			Motor_SixStep_ProcPhaseControl(p_motor, p_motor->Parameters.SensorMode);
		}
	}
}

const State_T * const P_FREEWHEEL_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
//	[MOTOR_TRANSITION_OPEN_LOOP] 		= &MOTOR_STATE_OPEN_LOOP,

	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
};

//void (* const FREEWHEEL_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

const State_T MOTOR_STATE_FREEWHEEL =
{
	.PP_INPUT_TRANSITION_STATE_MAP 		= P_FREEWHEEL_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)FREEWHEEL_TRANSITION_FUNCTION_MAP,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)FREEWHEEL_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 					= (StateMachine_StateFunction_T)FreewheelEntry,
	.OUTPUT 							= (StateMachine_StateFunction_T)FreewheelProc,
};


/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void FaultEntry(Motor_T * p_motor)
{

}

static void FaultProc(Motor_T * p_motor)
{
	//if fault clear
//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}

const State_T * const P_FAULT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_FAULT] 		= 0U,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_INIT,
	[MOTOR_TRANSITION_STOP] 		= 0U,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
	[MOTOR_TRANSITION_ALIGN] 		= 0U,
	[MOTOR_TRANSITION_SPIN] 			= 0U,

};

//void (* const FAULT_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const FAULT_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_FAULT =
{
	.PP_INPUT_TRANSITION_STATE_MAP 			= P_FAULT_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)FAULT_TRANSITION_FUNCTION_MAP,
	.P_INPUT_OUTPUT_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)FAULT_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 		= (StateMachine_StateFunction_T)FaultEntry,
	.OUTPUT 	= (StateMachine_StateFunction_T)FaultProc,
};



///*******************************************************************************/
///*!
//    @brief  State shared calibration
//*/
///*******************************************************************************/
//static void CalibrationEntry(Motor_T * p_motor)
//{
//	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
//	{
//		switch(p_motor->CalibrationParameter)
//		{
//		case adc:
//			Motor_FOC_StartCalibrateAdc(p_motor);
//		}
//
//	}
//	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
//	{
//		Motor_StartCalibration(p_motor);
//	}
//}
//
//static void CalibrationProc(Motor_T * p_motor)
//{
//	switch(p_motor->CalibrationParameter)
//	{
//	case adc:
//		if (Motor_CalibrateAdc(p_motor) == true)
//		{
//			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
//		}
//	}
//
//}
//
//const State_T * const P_CALIBRATION_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
//{
//	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
////	[MOTOR_TRANSITION_CALIBRATION] 	= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_FAULT,
//};
//
////void (* const CALIBRATION_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
////{
////	[MOTOR_TRANSITION_FAULT] 		= 0U,
////	[MOTOR_TRANSITION_INIT] 		= 0U,
////	[MOTOR_TRANSITION_STOP] 		= 0U,
////	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
////	[MOTOR_TRANSITION_ALIGN] 		= 0U,
////	[MOTOR_TRANSITION_SPIN] 			= 0U,
////};
//
////void (* const CALIBRATION_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
////{
////
////};
//
//const State_T MOTOR_STATE_CALIBRATION  =
//{
//	.PP_INPUT_TRANSITION_STATE_MAP 			= P_CALIBRATION_TRANSITION_STATE_MAP,
////	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)CALIBRATION_TRANSITION_FUNCTION_MAP,
////	.P_INPUT_OUTPUT_FUNCTION_MAP 			= (StateMachine_StateFunction_T *)CALIBRATION_OUTPUT_FUNCTION_MAP,
//	.TRANSITION_ENTRY 						= (StateMachine_StateFunction_T)CalibrationEntry,
//	.OUTPUT 								= (StateMachine_StateFunction_T)CalibrationProc,
//};



/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void CalibrateAdcEntry(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_StartCalibrateAdc(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_StartCalibrateAdc(p_motor);
	}
}

static void CalibrateAdcProc(Motor_T * p_motor)
{
	if (Motor_CalibrateAdc(p_motor) == true)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
}

const State_T * const P_CALIBRATE_ADC_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_FAULT,
};

//void (* const CALIBRATE_ADC_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const CALIBRATE_ADC_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_CALIBRATE_ADC  =
{
	.PP_INPUT_TRANSITION_STATE_MAP 			= P_CALIBRATE_ADC_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)CALIBRATE_ADC_TRANSITION_FUNCTION_MAP,
	.P_INPUT_OUTPUT_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)CALIBRATE_ADC_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 				= (StateMachine_StateFunction_T)CalibrateAdcEntry,
	.OUTPUT 						= (StateMachine_StateFunction_T)CalibrateAdcProc,
};


/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void CalibrateHallEntry(Motor_T * p_motor)
{
	Motor_StartCalibrateHall(p_motor);
}

static void CalibrateHallProc(Motor_T * p_motor)
{
	if (Motor_CalibrateHall(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
}

const State_T * const P_CALIBRATE_HALL_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_FAULT,
};

//void (* const CALIBRATE_HALL_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 			= 0U,
//	[MOTOR_TRANSITION_INIT] 			= 0U,
//	[MOTOR_TRANSITION_STOP] 			= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 				= 0U,
//};

void (* const CALIBRATE_HALL_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_CALIBRATE_HALL  =
{
	.PP_INPUT_TRANSITION_STATE_MAP 		= P_CALIBRATE_HALL_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)CALIBRATE_HALL_TRANSITION_FUNCTION_MAP,
	.P_INPUT_OUTPUT_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)CALIBRATE_HALL_OUTPUT_FUNCTION_MAP,
	.TRANSITION_ENTRY 		= (StateMachine_StateFunction_T)CalibrateHallEntry,
	.OUTPUT 				= (StateMachine_StateFunction_T)CalibrateHallProc,
};


/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void Flash(Motor_T * p_motor)
{

}


///*******************************************************************************/
///*!
//    @brief  State
//*/
///*******************************************************************************/
//static void FocSpinEntry(Motor_T * p_motor)
//{
//	Motor_FOC_SetAngleControl(p_motor);
//}
//
//static void FocSpinProc(Motor_T * p_motor)
//{
//	Motor_FOC_ProcAngleControl(p_motor);
//	Motor_SixStep_ProcPhaseControl(p_motor);
//
////	only check flags in pwm loop. set flags in main loop
//	if(Motor_PollFaultFlag(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
//	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}
//
//}
//const State_T * const P_FOC_SPIN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
//{
//	[MOTOR_TRANSITION_FAULT] 					= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 					= &MOTOR_STATE_STOP,
//};
//
//void (* const FOC_SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};
//
//void (* const FOC_SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
//{
//
//};
//
//const State_T MOTOR_STATE_FOC_SPIN =
//{
//	.PP_INPUT_TRANSITION_STATE_MAP 			= P_FOC_SPIN_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)FOC_SPIN_TRANSITION_FUNCTION_MAP,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)FOC_SPIN_OUTPUT_FUNCTION_MAP,
//	.TRANSITION_ENTRY = (StateMachine_StateFunction_T)FocSpinEntry,
//	.OUTPUT = (StateMachine_StateFunction_T)FocSpinProc,
//};
//
//
//
///*******************************************************************************/
///*!
//    @brief  State
//*/
///*******************************************************************************/
//static void SixStepSpinEntry(Motor_T * p_motor)
//{
//	Motor_SixStep_SetPhaseControl(p_motor);
//}
//
//static void SixStepSpinProc(Motor_T * p_motor)
//{
//	Motor_SixStep_ProcPhaseControl(p_motor);
//
////	only check flags in pwm loop. set flags in main loop
////	if(Motor_PollFaultFlag(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
////	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}
//}
//const State_T * const P_SIX_STEP_SPIN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH] =
//{
//	[MOTOR_TRANSITION_FAULT] 					= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 					= &MOTOR_STATE_STOP,
//};
//
//void (* const SIX_STEP_SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_CALIBRATE_ADC_COMPLETE] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= 0U,
//};
//
//void (* const SIX_STEP_SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_LENGTH])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};
//
//const State_T MOTOR_STATE_SIX_STEP_SPIN =
//{
//	.PP_INPUT_TRANSITION_STATE_MAP 			= P_SIX_STEP_SPIN_TRANSITION_STATE_MAP,
//	.P_INPUT_TRANSITION_FUNCTION_MAP 		= (StateMachine_StateFunction_T *)SIX_STEP_SPIN_TRANSITION_FUNCTION_MAP,
//	.P_INPUT_OUTPUT_FUNCTION_MAP 	= (StateMachine_StateFunction_T *)SIX_STEP_SPIN_OUTPUT_FUNCTION_MAP,
//	.TRANSITION_ENTRY = (StateMachine_StateFunction_T)SixStepSpinEntry,
//	.OUTPUT = (StateMachine_StateFunction_T)SixStepSpinProc,
//};





/*******************************************************************************/
/*!
    @brief  StateMachine Transition Input Parse
*/
/*******************************************************************************/
//static inline bool IsInputSpin(Motor_T * p_motor)
//{
//	return (p_motor->IsActiveControl == true);
////	return (p_motor->IsDirectionNeutral == false); //&& (p_motor->UserCmd > 0U);
//}
//
//static inline bool IsInputFreewheel(Motor_T * p_motor)
//{
//	return (p_motor->IsActiveControl == false);
////	return (p_motor->IsDirectionNeutral == true); //|| (passive brake &&(p_motor->UserCmd == 0));
//}
//
//static inline bool IsInputStop(Motor_T * p_motor)
//{
//	return (Motor_GetSpeed(p_motor) == 0U);
//}
