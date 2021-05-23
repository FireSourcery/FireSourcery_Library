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

#include "../Motor.h"
#include "../Motor_FOC.h"
#include "../Motor_SixStep.h"

#define MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES 			(10)
#define MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES 				(2) //input output map

extern const State_T MOTOR_STATE_INIT;
extern const State_T MOTOR_STATE_FAULT;
extern const State_T MOTOR_STATE_CALIBRATE_ADC;
extern const State_T MOTOR_STATE_ALIGN;
extern const State_T MOTOR_STATE_RUN;
extern const State_T MOTOR_STATE_STOP;

//extern const State_T MOTOR_STATE_FOC_INIT;
//extern const State_T MOTOR_STATE_FOC_CALIBRATE;
//extern const State_T MOTOR_STATE_FOC_ALIGN;

//extern const State_T MOTOR_STATE_FOC_RUN;
//extern const State_T MOTOR_STATE_SIX_STEP_RUN;

/*******************************************************************************/
/*!
    @brief  State
*/
/*******************************************************************************/
void FaultEntry(Motor_T * p_motor)
{

}

void FaultLoop(Motor_T * p_motor)
{
	//if fault clear
	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}

const State_T * const P_FAULT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= 0U,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_INIT,
	[MOTOR_TRANSITION_STOP] 		= 0U,
	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
	[MOTOR_TRANSITION_ALIGN] 		= 0U,
	[MOTOR_TRANSITION_RUN] 			= 0U,

};

//void (* const FAULT_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
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
	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_CALIBRATION);
}

const State_T * const P_INIT_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATION] 	= &MOTOR_STATE_CALIBRATE_ADC,
	[MOTOR_TRANSITION_ALIGN] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_RUN] 			= &MOTOR_STATE_FAULT,
};

//void (* const INIT_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
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
    @brief  State
*/
/*******************************************************************************/
static void CalibrateAdcEntry(Motor_T * p_motor)
{
	Motor_FOC_StartCalibrateAdc(p_motor);
}

static void CalibrateAdcLoop(Motor_T * p_motor)
{
	if (Motor_FOC_CalibrateAdc(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);
	}
}

const State_T * const P_CALIBRATE_ADC_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 		= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATION] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_RUN] 			= &MOTOR_STATE_FAULT,
};

//void (* const CALIBRATE_ADC_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
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
    @brief  State long term stop?
*/
/*******************************************************************************/
static void StopEntry(Motor_T * p_motor)
{

}

static void StopLoop(Motor_T * p_motor)
{

//	//wait for user inpu
//
//if(brake == 0)
	{

		if (Motor_GetAlignMode(p_motor) == MOTOR_ALIGN_MODE_DISABLE)
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
		}
		else
		{
			StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_ALIGN);
		}
	}

//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, PIN_FUNCTION_FORWARD);}
//	if(GetSwitchEdge(DirectionButton)) {StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, PIN_FUNCTION_REVERSE);}
//
//	if(Bounce_GetSwitchEdge(DirectionButton) == 1) { SetDirection(forward);}
//	else if(Bounce_GetSwitchEdge(DirectionButton) == 1) { SetDirection(Reverse);}
//	else if (Bounce_GetState(DirectionButton) ==0 && GetEdge == 1) set neutral
}

const State_T * const P_STOP_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATION] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 		= &MOTOR_STATE_ALIGN,
	[MOTOR_TRANSITION_RUN] 			= &MOTOR_STATE_RUN,
};

//void (* const STOP_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
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
//	Motor_ActivateAlign(p_motor);
}

static void AlignLoop(Motor_T * p_motor)
{
	if(Motor_WaitAlign(p_motor))
	{
		StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_RUN);
	}
}

const State_T * const P_ALIGN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_CALIBRATION] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_RUN] 			= &MOTOR_STATE_RUN,
};

//void (* const ALIGN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
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
static void RunEntry(Motor_T * p_motor)
{
	if (p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_SetAngleControl(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_SetCommutationControl(p_motor);
	}
}

static void RunLoop(Motor_T * p_motor)
{
	//if brake toggle stop { VReq = 0,	} //start deceleration

	if (p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		Motor_FOC_ProcAngleControl(p_motor);
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{
		Motor_SixStep_ProcCommutationControl(p_motor);
	}

//	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}

}
const State_T * const P_RUN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
{
	[MOTOR_TRANSITION_FAULT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_INIT] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_STOP] 		= &MOTOR_STATE_STOP,
	[MOTOR_TRANSITION_CALIBRATION] 	= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_ALIGN] 		= &MOTOR_STATE_FAULT,
	[MOTOR_TRANSITION_RUN] 			= &MOTOR_STATE_RUN,
};

//void (* const RUN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
//};

void (* const RUN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
{

};

const State_T MOTOR_STATE_RUN =
{
	.pp_TransitionStateMap 			= P_RUN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)RUN_TRANSITION_FUNCTION_MAP,
	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)RUN_OUTPUT_FUNCTION_MAP,
	.Entry 		= (StateMachine_StateFunction_T)RunEntry,
	.Output 	= (StateMachine_StateFunction_T)RunLoop,
};


///*******************************************************************************/
///*!
//    @brief  State
//*/
///*******************************************************************************/
//static void FocRunEntry(Motor_T * p_motor)
//{
//	Motor_FOC_SetAngleControl(p_motor);
//}
//
//static void FocRunLoop(Motor_T * p_motor)
//{
//	Motor_FOC_ProcAngleControl(p_motor);
//	Motor_SixStep_ProcCommutationControl(p_motor);
//
////	only check flags in pwm loop. set flags in main loop
//	if(Motor_PollFaultFlag(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
//	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}
//
//}
//const State_T * const P_FOC_RUN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_TRANSITION_FAULT] 					= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 					= &MOTOR_STATE_STOP,
//};
//
//void (* const FOC_RUN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
//};
//
//void (* const FOC_RUN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//
//};
//
//const State_T MOTOR_STATE_FOC_RUN =
//{
//	.pp_TransitionStateMap 			= P_FOC_RUN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)FOC_RUN_TRANSITION_FUNCTION_MAP,
//	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)FOC_RUN_OUTPUT_FUNCTION_MAP,
//	.Entry = (StateMachine_StateFunction_T)FocRunEntry,
//	.Output = (StateMachine_StateFunction_T)FocRunLoop,
//};
//
//
//
///*******************************************************************************/
///*!
//    @brief  State
//*/
///*******************************************************************************/
//static void SixStepRunEntry(Motor_T * p_motor)
//{
//	Motor_SixStep_SetCommutationControl(p_motor);
//}
//
//static void SixStepRunLoop(Motor_T * p_motor)
//{
//	Motor_SixStep_ProcCommutationControl(p_motor);
//
////	only check flags in pwm loop. set flags in main loop
////	if(Motor_PollFaultFlag(p_motor))	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_FAULT);	}
////	if(Motor_GetSpeed(p_motor) == 0)	{ StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_STOP);	}
//}
//const State_T * const P_SIX_STEP_RUN_TRANSITION_STATE_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES] =
//{
//	[MOTOR_TRANSITION_FAULT] 					= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= &MOTOR_STATE_FAULT,
//	[MOTOR_TRANSITION_STOP] 					= &MOTOR_STATE_STOP,
//};
//
//void (* const SIX_STEP_RUN_TRANSITION_FUNCTION_MAP[MOTOR_STATE_MACHINE_TRANSITION_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_CALIBRATION_COMPLETE] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN_COMPLETE] 			= 0U,
//};
//
//void (* const SIX_STEP_RUN_OUTPUT_FUNCTION_MAP[MOTOR_STATE_MACHINE_INPUT_MAP_ENTRIES])(Motor_T * p_motor) =
//{
//	[MOTOR_TRANSITION_FAULT] 		= 0U,
//	[MOTOR_TRANSITION_INIT] 		= 0U,
//	[MOTOR_TRANSITION_STOP] 		= 0U,
//	[MOTOR_TRANSITION_CALIBRATION] 	= 0U,
//	[MOTOR_TRANSITION_ALIGN] 		= 0U,
//	[MOTOR_TRANSITION_RUN] 			= 0U,
//};
//
//const State_T MOTOR_STATE_SIX_STEP_RUN =
//{
//	.pp_TransitionStateMap 			= P_SIX_STEP_RUN_TRANSITION_STATE_MAP,
//	.p_TransitionFunctionMap 		= (StateMachine_StateFunction_T *)SIX_STEP_RUN_TRANSITION_FUNCTION_MAP,
//	.p_SelfTransitionFunctionMap 	= (StateMachine_StateFunction_T *)SIX_STEP_RUN_OUTPUT_FUNCTION_MAP,
//	.Entry = (StateMachine_StateFunction_T)SixStepRunEntry,
//	.Output = (StateMachine_StateFunction_T)SixStepRunLoop,
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

	StateMachine_Semisynchronous_ProcTransition(&p_motor->StateMachine, MOTOR_TRANSITION_INIT);
}
