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

#define MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES 			(10)
#define MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES 				(2) //input output map


extern const State_T MOTOR_STATE_INIT;
extern const State_T MOTOR_STATE_STOP;
extern const State_T MOTOR_STATE_ALIGN;
extern const State_T MOTOR_STATE_SPIN;
extern const State_T MOTOR_STATE_FREEWHEEL;
extern const State_T MOTOR_STATE_FAULT;
extern const State_T MOTOR_STATE_CALIBRATE_ADC;
extern const State_T MOTOR_STATE_CALIBRATE_HALL;

//extern const State_T MOTOR_STATE_FOC_INIT;
//extern const State_T MOTOR_STATE_FOC_CALIBRATE;
//extern const State_T MOTOR_STATE_FOC_ALIGN;

//extern const State_T MOTOR_STATE_FOC_SPIN;
//extern const State_T MOTOR_STATE_SIX_STEP_SPIN;



/*******************************************************************************/
/*!
    @brief  Transition Conditions
*/
/*******************************************************************************/
static inline bool Motor_PollStopToSpin(Motor_T * p_motor)
{
	bool transition = true;

	transition &= (p_motor->InputSwitchBrake == false) || ((p_motor->InputSwitchBrake == true) && (Motor_GetSpeed(p_motor) > 0));
	transition &= (p_motor->IsDirectionNeutral == false);
	transition &= (p_motor->UserCmd > 0U);
	transition &= (p_motor->IsThrottleRelease == false);

	return transition;
}

static inline bool Motor_PollSpinToFreewheel(Motor_T * p_motor)
{
	bool transition = false;

	transition |= (p_motor->IsDirectionNeutral == true);
	transition |= (p_motor->UserCmd == 0); 	//brake 0 and throttle 0
	transition |= (p_motor->IsThrottleRelease == true);
	transition |= ((p_motor->InputSwitchBrake == true) && (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE));

	//poll stall
	return transition;
}

static inline bool Motor_PollFreewheelToSpin(Motor_T * p_motor)
{
	bool transition = true;

	transition &= (p_motor->InputSwitchBrake == false) || ((p_motor->InputSwitchBrake == true) && (p_motor->Parameters.BrakeMode != MOTOR_BRAKE_MODE_PASSIVE) && (Motor_GetSpeed(p_motor) > 0));
	transition &= (p_motor->UserCmd > 0U);
	transition &= (p_motor->IsDirectionNeutral == false);
	transition &= (p_motor->IsThrottleRelease == false);

	return transition;
}

static inline bool Motor_PollFreewheelToStop(Motor_T * p_motor)
{
	if (Motor_GetSpeed(p_motor) == 0U)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void FaultEntry(Motor_T * p_motor)
{

}

static void FaultLoop(Motor_T * p_motor)
{
	//if fault clear
//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}

const State_T * const P_FAULT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= 0U,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_INIT,
	[MOTOR_TRANSITION_STOP] 		= 0U,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
	[MOTOR_TRANSITION_ALIGN] 		= 0U,
	[MOTOR_TRANSITION_SPIN] 			= 0U,

};

//void (* const FAULT_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const FAULT_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_FAULT =
{
	.pp_TransitionStateMap 			= P_FAULT_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_TransitionFunction_T *)FAULT_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_TransitionFunction_T *)FAULT_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)FaultEntry,
	.Output 	= (StateMachine_StateFunction_T)FaultLoop,
};

/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void InitEntry(Motor_T * p_motor)
{
	Motor_InitReboot(p_motor);
}

static void InitLoop(Motor_T * p_motor)
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

const State_T * const P_INIT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_INIT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_CALIBRATE_ADC,
	[MOTOR_TRANSITION_CALIBRATE_HALL] 	= &MOTOR_STATE_CALIBRATE_HALL,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 				= &MOTOR_STATE_FAULT,
};

//void (* const INIT_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const INIT_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_INIT =
{
	.pp_TransitionStateMap 			= P_INIT_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_TransitionFunction_T *)INIT_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_TransitionFunction_T *)INIT_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_TransitionFunction_T)InitEntry,
	.Output 	= (StateMachine_TransitionFunction_T)InitLoop,
};


/*******************************************************************************/
/*!
    @brief  State long term stop?
*/
/*******************************************************************************/
static void StopEntry(Motor_T * p_motor)
{
	Motor_Float(p_motor);

	//set zeros Motor_SetZero
	p_motor->VPwm = 0U;
}

static void StopLoop(Motor_T * p_motor)
{

	//proc direction
	//		Motor_PollToggleDirectionUpdate(p_motor);


	if (Motor_PollStopToSpin(p_motor))
	{
		if (Motor_GetAlignMode(p_motor) == MOTOR_ALIGN_MODE_DISABLE)
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
		}
		else
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_ALIGN);
		}
	}

	//proc flash
	//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_LOCK);

}

const State_T * const P_STOP_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_CALIBRATE_ADC,
	[MOTOR_TRANSITION_CALIBRATE_HALL] 	= &MOTOR_STATE_CALIBRATE_HALL,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_ALIGN,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
};

//void (* const STOP_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const STOP_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{
//set align on not brake


};

const State_T MOTOR_STATE_STOP =
{
	.pp_TransitionStateMap 			= P_STOP_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)STOP_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)STOP_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)StopEntry,
	.Output 	= (StateMachine_StateFunction_T)StopLoop,
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

static void AlignLoop(Motor_T * p_motor)
{
	if(Motor_ProcAlign(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
	}
}

const State_T * const P_ALIGN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
};

//void (* const ALIGN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const ALIGN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_ALIGN =
{
	.pp_TransitionStateMap 			= P_ALIGN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)ALIGN_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)ALIGN_OUTPUT_FUNCTION_MAP,
	.Entry = (StateMachine_StateFunction_T)AlignEntry,
	.Output = (StateMachine_StateFunction_T)AlignLoop,
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
		Motor_SixStep_StartSpin(p_motor);
	}
}

static void SpinLoop(Motor_T * p_motor)
{
//	if (Motor_GetCommutationMode(p_motor) == MOTOR_COMMUTATION_MODE_FOC)
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcPhaseControl(p_motor);
	}

	if (Motor_PollSpinToFreewheel(p_motor))
	{
		//resume spin function
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FREEWHEEL);
	}

}

static void SpinInputAll(Motor_T * p_motor)
{

}

const State_T * const P_SPIN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,
	[MOTOR_TRANSITION_FREEWHEEL] 		= &MOTOR_STATE_FREEWHEEL,

	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
};

//void (* const SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (*const SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{
	&SpinInputAll,
};

const State_T MOTOR_STATE_SPIN =
{
	.pp_TransitionStateMap 			= P_SPIN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)SPIN_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)SPIN_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)SpinEntry,
	.Output 	= (StateMachine_StateFunction_T)SpinLoop,
};



/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
static void FreewheelEntry(Motor_T * p_motor)
{
	Motor_SixStep_StartFreewheel(p_motor);
}

static void FreewheelLoop(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
//		Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcPhaseControl(p_motor);
	}

	if (Motor_PollFreewheelToStop(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
	else if (Motor_PollFreewheelToSpin(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_SPIN);
	}

}

const State_T * const P_FREEWHEEL_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_SPIN,

	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
};

//void (* const SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};



const State_T MOTOR_STATE_FREEWHEEL =
{
	.pp_TransitionStateMap 			= P_FREEWHEEL_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)FREEWHEEL_TRANSITION_FUNCTION_MAP,
//	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)FREEWHEEL_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)FreewheelEntry,
	.Output 	= (StateMachine_StateFunction_T)FreewheelLoop,
};


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

	}
}

static void CalibrateAdcLoop(Motor_T * p_motor)
{
	bool isComplete;

	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		isComplete = Motor_FOC_CalibrateAdc(p_motor);
	}
	else //p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{

	}

	if (isComplete == true)
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
}

const State_T * const P_CALIBRATE_ADC_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 		= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_FAULT,
};

//void (* const CALIBRATE_ADC_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};

void (* const CALIBRATE_ADC_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_CALIBRATE_ADC  =
{
	.pp_TransitionStateMap 			= P_CALIBRATE_ADC_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_TransitionFunction_T *)CALIBRATE_ADC_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_TransitionFunction_T *)CALIBRATE_ADC_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)CalibrateAdcEntry,
	.Output 	= (StateMachine_StateFunction_T)CalibrateAdcLoop,
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

static void CalibrateHallLoop(Motor_T * p_motor)
{
	if (Motor_CalibrateHall(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
}

const State_T * const P_CALIBRATE_HALL_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 			= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 			= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_SPIN] 			= &MOTOR_STATE_FAULT,
};

//void (* const CALIBRATE_HALL_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 			= 0U,
//	[MOTOR_TRANSITION_INIT] 			= 0U,
//	[MOTOR_TRANSITION_STOP] 			= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 			= 0U,
//	[MOTOR_TRANSITION_SPIN] 				= 0U,
//};

void (* const CALIBRATE_HALL_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_CALIBRATE_HALL  =
{
	.pp_TransitionStateMap 			= P_CALIBRATE_HALL_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_TransitionFunction_T *)CALIBRATE_HALL_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_TransitionFunction_T *)CALIBRATE_HALL_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)CalibrateHallEntry,
	.Output 	= (StateMachine_StateFunction_T)CalibrateHallLoop,
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
//static void FocSpinLoop(Motor_T * p_motor)
//{
//	Motor_FOC_ProcAngleControl(p_motor);
//	Motor_SixStep_ProcPhaseControl(p_motor);
//
////	only check flags in pwm loop. set flags in main loop
//	if(Motor_PollFaultFlag(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
//	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}
//
//}
//const State_T * const P_FOC_SPIN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_TRANSITION_FAULT] 					= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 					= &MOTOR_STATE_STOP,
//};
//
//void (* const FOC_SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATE_ADC] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_SPIN] 			= 0U,
//};
//
//void (* const FOC_SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//
//};
//
//const State_T MOTOR_STATE_FOC_SPIN =
//{
//	.pp_TransitionStateMap 			= P_FOC_SPIN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)FOC_SPIN_TRANSITION_FUNCTION_MAP,
//	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)FOC_SPIN_OUTPUT_FUNCTION_MAP,
//	.Entry = (StateMachine_StateFunction_T)FocSpinEntry,
//	.Output = (StateMachine_StateFunction_T)FocSpinLoop,
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
//static void SixStepSpinLoop(Motor_T * p_motor)
//{
//	Motor_SixStep_ProcPhaseControl(p_motor);
//
////	only check flags in pwm loop. set flags in main loop
////	if(Motor_PollFaultFlag(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
////	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}
//}
//const State_T * const P_SIX_STEP_SPIN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_TRANSITION_FAULT] 					= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 					= &MOTOR_STATE_STOP,
//};
//
//void (* const SIX_STEP_SPIN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_CALIBRATE_ADC_COMPLETE] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= 0U,
//};
//
//void (* const SIX_STEP_SPIN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
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
//	.pp_TransitionStateMap 			= P_SIX_STEP_SPIN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)SIX_STEP_SPIN_TRANSITION_FUNCTION_MAP,
//	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)SIX_STEP_SPIN_OUTPUT_FUNCTION_MAP,
//	.Entry = (StateMachine_StateFunction_T)SixStepSpinEntry,
//	.Output = (StateMachine_StateFunction_T)SixStepSpinLoop,
//};


/*******************************************************************************/
/*!
    @brief
*/
/*******************************************************************************/
void MotorStateMachine_Init(Motor_T * p_motor)
{
	StateMachine_Init
	(
		&(p_motor->StateMachine),
		&MOTOR_STATE_INIT,
		MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES,
		MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES,
		p_motor
	);

//	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}
