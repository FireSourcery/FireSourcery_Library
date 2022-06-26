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
	@file 	MotorController_StateMachine.c
	@author FireSoucery
	@brief  MotorController_StateMachine
			States for input mode
			User perspective, input filter
			input acceptance using input state and motor state
			motor state machine uses only motor state
	@version V0
*/
/******************************************************************************/
#include "MotorController_StateMachine.h"
#include "Utility/StateMachine/StateMachine.h"
#include <string.h>

static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_STOP;
static const StateMachine_State_T STATE_RUN;
// static const StateMachine_State_T STATE_NEUTRAL;
// static const StateMachine_State_T STATE_BRAKING;
// static const StateMachine_State_T STATE_MOTORTING;
static const StateMachine_State_T STATE_FAULT;

/******************************************************************************/
/*!
	@brief
*/
/******************************************************************************/
const StateMachine_Machine_T MCSM_MACHINE =
{
	.P_STATE_INITIAL = &STATE_INIT,
	.TRANSITION_TABLE_LENGTH = MCSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(MotorController_T * p_mc) { (void)p_mc; return &STATE_FAULT; }

/******************************************************************************/
/*!
	@brief  State

	Initially Fault flags are set by adc, but init does not tranisition to fault
*/
/******************************************************************************/
static StateMachine_State_T * Init_InputDirection(MotorController_T * p_mc)
{
	StateMachine_State_T * p_nextState;

	if(MotorController_ProcDirection(p_mc) == true)
	{
		// bool isThrottleOn; //check and isthrottle off
		p_nextState = &STATE_STOP;
	}
	else
	{
		p_nextState = &STATE_FAULT;
	}

	return p_nextState;
}

static StateMachine_State_T * Init_InputReleaseThrottle(MotorController_T * p_mc)
{
	StateMachine_State_T * p_nextState = 0U;
	if(p_mc->Parameters.BuzzerFlagsEnable.BeepThrottleOnInit == true)
	{
		MotorController_BeepShort(p_mc);
		p_nextState = &STATE_STOP; //check direction is not forwrad or rev
	}
	return p_nextState;
}

static StateMachine_State_T * Init_InputThrottle(MotorController_T * p_mc)
{
	if((p_mc->Parameters.BuzzerFlagsEnable.BeepThrottleOnInit == true) && (p_mc->BuzzerFlagsActive.BeepThrottleOnInit == 0U))
	{
		p_mc->BuzzerFlagsActive.BeepThrottleOnInit = 1U;
		MotorController_BeepShort(p_mc);
	}

	return 0U;
}

static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	// [MCSM_INPUT_FAULT]				= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_DIRECTION] 			= (StateMachine_Transition_T)Init_InputDirection,
	[MCSM_INPUT_RELEASE_THROTTLE] 	= (StateMachine_Transition_T)Init_InputReleaseThrottle,
	[MCSM_INPUT_THROTTLE] 			= (StateMachine_Transition_T)Init_InputThrottle,
};

static void Init_Entry(MotorController_T * p_mc)
{
	// (void)p_mc;
	Blinky_Blink(&p_mc->Buzzer, 200U);
}

static void Init_Exit(MotorController_T * p_mc)
{
	p_mc->FaultFlags.State = 0U; /* Clear initial ADC readings */
}

static void Init_Proc(MotorController_T * p_mc)
{
	(void)p_mc;
// _StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_STOP);
// if(Timer_GetBase(&p_mc->TimerMillis) > 50U) 	//wait 50ms for adc , may need preinit
// {

// }
}

static const StateMachine_State_T STATE_INIT =
{
	.ID 					= MCSM_STATE_ID_INIT,
	.P_TRANSITION_TABLE 	= &INIT_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Init_Entry,
	.EXIT 					= (StateMachine_Output_T)Init_Exit,
	.OUTPUT 				= (StateMachine_Output_T)Init_Proc,
};


//TODO SYnc protocol direction


/******************************************************************************/
/*!
	@brief  Stop State

	Enters upon all motors reading 0 speed,
		motor may be in run(active brake) or freewheel state

	bug: if power on into run state, while in reverse, will start forward.
*/
/******************************************************************************/
static StateMachine_State_T * Stop_InputThrottle(MotorController_T * p_mc)
{
	StateMachine_State_T * p_nextState;

	if(p_mc->MainDirection == p_mc->UserDirection)
	{
		MotorController_ProcUserCmdThrottle(p_mc);
		p_nextState = &STATE_RUN;
	}
	else
	{
		p_nextState = 0U;
	}

	return p_nextState;
}

static StateMachine_State_T * Stop_InputBrake(MotorController_T * p_mc) //change to brake edge
{
	MotorController_GroundMotorAll(p_mc); /* repeat set is okay for brake */
	return 0U;
}

static StateMachine_State_T * Stop_InputRelease(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc);
	return 0U;
}

static StateMachine_State_T * Stop_InputDirection(MotorController_T * p_mc)
{
	// StateMachine_State_T * p_nextState;
	bool isSucess = MotorController_ProcDirection(p_mc); //fix need beep source
	/*
		Motor Freewheel check stop should be atomic relative to this function
		this runs before motor freewheel checks speed => goto fault state.
	*/
	if(isSucess == false) { p_mc->FaultFlags.StopStateSync = 1U; }
	return (isSucess == true) ? 0U : &STATE_FAULT;
}

static StateMachine_State_T * Stop_InputSaveParams(MotorController_T * p_mc)
{
	/* Disable PWM interrupt to disable Motor_StateMachine */
	Motor_DisablePwm(&p_mc->CONFIG.P_MOTORS[0U]);

	switch(p_mc->StopSubstate)
	{
		case MOTOR_CONTROLLER_NVM_PARAMS_ALL: p_mc->NvmStatus = MotorController_SaveParameters_Blocking(p_mc); break;
		case MOTOR_CONTROLLER_NVM_BOOT: p_mc->NvmStatus = MotorController_SaveBootReg_Blocking(p_mc); break;
		case MOTOR_CONTROLLER_NVM_ONCE: p_mc->NvmStatus = MotorController_SaveOnce_Blocking(p_mc); break;
		default: break;
	}

	Motor_EnablePwm(&p_mc->CONFIG.P_MOTORS[0U]);

	return 0U;
}

static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 				= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_DIRECTION] 			= (StateMachine_Transition_T)Stop_InputDirection,
	[MCSM_INPUT_THROTTLE] 			= (StateMachine_Transition_T)Stop_InputThrottle,
	[MCSM_INPUT_BRAKE] 				= (StateMachine_Transition_T)Stop_InputBrake,
	[MCSM_INPUT_RELEASE_THROTTLE] 	= (StateMachine_Transition_T)Stop_InputRelease,
	[MCSM_INPUT_RELEASE_BRAKE] 		= (StateMachine_Transition_T)0U, //release into neutral, different from release into no input
	[MCSM_INPUT_NULL] 				= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_NEUTRAL] 			= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_SET_NEUTRAL] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_SAVE_PARAMS] 		= (StateMachine_Transition_T)Stop_InputSaveParams, //todo share with calibration
};

static void Stop_Entry(MotorController_T * p_mc)
{
	//	if (p_mc->Parameters.StopMode == MOTOR_CONTROLLER_STOP_MODE_GROUND)
	//	{
	////		MotorController_GroundMotorAll(p_mc);
	//	}
	//	else
	//	{
	MotorController_DisableMotorAll(p_mc); //entry on brake
//	}
}

static void Stop_Proc(MotorController_T * p_mc)
{
	(void)p_mc;
}

static const StateMachine_State_T STATE_STOP =
{
	.ID 					= MCSM_STATE_ID_STOP,
	.P_TRANSITION_TABLE 	= &STOP_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Stop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Stop_Proc,
};

/******************************************************************************/
/*!
	@brief  State

	During Run State Motor Controller accepts speed inputs
	motors may be in active control or freewheel

	release/edge inputs implicitly tracks previous state.
	or use brake, throttle, neutral states, also accounts for entry function

	while in neutral,
	brake release detected, throttle release ignore,
	is implemented via dependency on input priority, may need to change
*/
/******************************************************************************/
static StateMachine_State_T * Run_InputDirection(MotorController_T * p_mc)
{
	if(p_mc->MainDirection != p_mc->UserDirection)
	{
		MotorController_BeepShort(p_mc); //abstrat not allow direction chagen
	}

	return 0U;
}

/*
	Do not allow for run state including both throttle and brake.
	Alternatively, need additional brake state. allow for throttle, ignore for brake.
*/
static StateMachine_State_T * Run_InputNeutral(MotorController_T * p_mc)
{
	//	return &STATE_NEUTRAL;
	return (MotorController_CheckStopMotorAll(p_mc) == true) ? &STATE_STOP : 0U;
}

static StateMachine_State_T * Run_InputNeutralStart(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc); /* will cause discontinuity during brake, todo brake flag or state */
	return 0U;
}

static StateMachine_State_T * Run_InputThrottle(MotorController_T * p_mc)
{
	//or use separate neutral state to prevent return to throttle state

	if(p_mc->MainDirection == p_mc->UserDirection)
	{
		MotorController_ProcUserCmdThrottle(p_mc);
	}

	return 0U;
}

//if using 2 part set/proc
// static StateMachine_State_T * Run_InputThrottleStart(MotorController_T * p_mc)
// {
// 	(void)p_mc;
// 	return 0U;
// }

/*
	When brake is released Coast mode will run, and check stop
	or go directly to stop so direction can change
*/
static StateMachine_State_T * Run_InputBrake(MotorController_T * p_mc)
{
	MotorController_ProcUserCmdBrake(p_mc);
	return (MotorController_CheckStopMotorAll(p_mc) == true) ? &STATE_STOP : 0U;
}

//if using 2 part set/proc
// static StateMachine_State_T * Run_InputBrakeStart(MotorController_T * p_mc)
// {
// 	return 0U;
// }

/*
 * input release
 */
static StateMachine_State_T * Run_InputCoast(MotorController_T * p_mc)
{
	if(p_mc->Parameters.CoastMode == MOTOR_CONTROLLER_COAST_MODE_REGEN)
	{
		// MotorController_ProcUserCmdVoltageBrake(p_mc);
	}
	else
	{
		//motor already disabled
	}

	return (MotorController_CheckStopMotorAll(p_mc) == true) ? &STATE_STOP : 0U;
}

/*
	only release into null/coast
*/
static StateMachine_State_T * Run_InputCoastStart(MotorController_T * p_mc)
{
	if(p_mc->Parameters.CoastMode == MOTOR_CONTROLLER_COAST_MODE_REGEN)
	{
		//		MotorController_StartRegenMode(p_mc); //if using 2 part set/proc
	}
	else
	{
		MotorController_DisableMotorAll(p_mc);
	}

	return 0U;
}

/*
	release into neutral, release into throttle/coast satisfy? todo neutral state
*/
static StateMachine_State_T * Run_InputReleaseBrake(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc);
	return 0U;
}

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 				= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_DIRECTION] 			= (StateMachine_Transition_T)Run_InputDirection,
	[MCSM_INPUT_THROTTLE] 			= (StateMachine_Transition_T)Run_InputThrottle,
	[MCSM_INPUT_BRAKE] 				= (StateMachine_Transition_T)Run_InputBrake,
	[MCSM_INPUT_RELEASE_THROTTLE] 	= (StateMachine_Transition_T)Run_InputCoastStart,
	[MCSM_INPUT_RELEASE_BRAKE] 		= (StateMachine_Transition_T)Run_InputReleaseBrake,
	[MCSM_INPUT_NULL] 				= (StateMachine_Transition_T)Run_InputCoast,
	[MCSM_INPUT_NEUTRAL] 			= (StateMachine_Transition_T)Run_InputNeutral,
	[MCSM_INPUT_SET_NEUTRAL] 		= (StateMachine_Transition_T)Run_InputNeutralStart,
	[MCSM_INPUT_SAVE_PARAMS] 		= (StateMachine_Transition_T)0U,
};

static void Run_Entry(MotorController_T * p_mc)
{
	(void)p_mc;
}

static void Run_Proc(MotorController_T * p_mc)
{
	(void)p_mc;
}

static const StateMachine_State_T STATE_RUN =
{
	.ID 					= MCSM_STATE_ID_RUN,
	.P_TRANSITION_TABLE 	= &RUN_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Run_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Run_Proc,
};


/******************************************************************************/
/*!
	@brief  State

	Active throttle will remain active during
	Alternatively use extra brake state, or track is neutral set once to avoid repeat
*/
/******************************************************************************/
//static StateMachine_State_T * Neutral_InputDirection(MotorController_T * p_mc)
//{
//	StateMachine_State_T * p_nextState;
//	/* in neutral state, Motor not stopped yet */
//	if(p_mc->MainDirection != p_mc->UserDirection)
//	{
////		MotorController_Beep(p_mc);
//		p_nextState = 0U;
//	}
//	else
//	{
//		/* fault is transition to the same direction fails */
////		p_nextState = (MotorController_ProcDirection(p_mc) == true) ? &STATE_RUN : &STATE_FAULT;
//		p_nextState = &STATE_RUN;
//	}
//
//	return p_nextState;
//}
//
//static StateMachine_State_T * Neutral_InputBrake(MotorController_T * p_mc)
//{
//	MotorController_ProcUserCmdBrake(p_mc);
//	return 0U;
//	/*
//	 * When brake is released, will enter neutral mode again
//	 */
//	//	return &STATE_RUN;
//}
//
//
//static StateMachine_State_T * Neutral_InputRelease(MotorController_T * p_mc)
//{
//	MotorController_DisableMotorAll(p_mc);
//	return 0U;
//}
//
//static const StateMachine_Transition_T NEUTRAL_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
//{
//	[MCSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
//	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Neutral_InputDirection,
//	[MCSM_INPUT_THROTTLE] 		= (StateMachine_Transition_T)0U,
//	[MCSM_INPUT_BRAKE]  		= (StateMachine_Transition_T)Neutral_InputBrake,
//	[MCSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)Neutral_InputRelease,
//	[MCSM_INPUT_NULL] 			= (StateMachine_Transition_T)0U,
//	[MCSM_INPUT_NEUTRAL] 		= (StateMachine_Transition_T)0U,
//	[MCSM_INPUT_SAVE_PARAMS] 	= (StateMachine_Transition_T)0U,
//};
//
//static void Neutral_Entry(MotorController_T * p_mc)
//{
//	MotorController_DisableMotorAll(p_mc); // entry while braking, will experience discontinuity
//}
//
//static void Neutral_Proc(MotorController_T * p_mc)
//{
//	if(MotorController_CheckStopMotorAll(p_mc) == true)
//	{
//		StateMachine_ProcTransition(&p_mc->StateMachine, &STATE_STOP);
//	}
//}
//
//static const StateMachine_State_T STATE_NEUTRAL =
//{
//	.P_TRANSITION_TABLE 	= &NEUTRAL_TRANSITION_TABLE[0U],
//	.ENTRY 				= (StateMachine_Output_T)Neutral_Entry,
//	.OUTPUT 				= (StateMachine_Output_T)Neutral_Proc,
//};


/******************************************************************************/
/*!
	@brief  State
*/
/******************************************************************************/

/* Sensor faults only clear on user input */
static StateMachine_State_T * Fault_InputFault(MotorController_T * p_mc)
{
	p_mc->FaultFlags.Motors 				= (MotorController_ClearFaultMotorAll(p_mc) == false);
	p_mc->FaultFlags.VSenseLimit 			= VMonitor_GetIsStatusLimit(&p_mc->VMonitorSense);
	p_mc->FaultFlags.VAccLimit 				= VMonitor_GetIsStatusLimit(&p_mc->VMonitorAcc);
	p_mc->FaultFlags.VPosLimit 				= VMonitor_GetIsStatusLimit(&p_mc->VMonitorPos);
	p_mc->FaultFlags.PcbOverHeat 			= Thermistor_GetIsShutdown(&p_mc->ThermistorPcb);
	p_mc->FaultFlags.MosfetsTopOverHeat 	= Thermistor_GetIsShutdown(&p_mc->ThermistorMosfetsTop);
	p_mc->FaultFlags.MosfetsBotOverHeat 	= Thermistor_GetIsShutdown(&p_mc->ThermistorMosfetsBot);
	p_mc->FaultFlags.User 					= 0U;
	if(p_mc->FaultFlags.Motors == 0U) {p_mc->FaultFlags.MotorOverHeat = 0U;}
	return 0U;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 				= (StateMachine_Transition_T)Fault_InputFault,
	[MCSM_INPUT_DIRECTION] 			= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_THROTTLE] 			= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_BRAKE] 				= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_RELEASE_THROTTLE] 	= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_RELEASE_BRAKE] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_NULL] 				= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_NEUTRAL] 			= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_SET_NEUTRAL] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_SAVE_PARAMS] 		= (StateMachine_Transition_T)0U,
};

static void Fault_Entry(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc);
	memcpy((void *)&p_mc->FaultAnalogRecord, (void *)&p_mc->AnalogResults, sizeof(MotAnalog_Results_T));
	Blinky_StartPeriodic(&p_mc->Buzzer, 500U, 500U);
}

static void Fault_Proc(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc);

	switch(p_mc->Parameters.InputMode)
	{
		case MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL: /* Protocol Rx Lost use auto recover, without user input */
			p_mc->FaultFlags.RxLost = Protocol_CheckRxLost(&p_mc->CONFIG.P_PROTOCOLS[0U]);
			break;
		case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
		default:  break;
	}

	if(p_mc->FaultFlags.State == 0U)
	{
		Blinky_Stop(&p_mc->Buzzer);
		_StateMachine_ProcStateTransition(&p_mc->StateMachine, &STATE_STOP);
	}
}

static const StateMachine_State_T STATE_FAULT =
{
	.ID 					= MCSM_STATE_ID_FAULT,
	.P_TRANSITION_TABLE 	= &FAULT_TRANSITION_TABLE[0U],
	.ENTRY 					= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
