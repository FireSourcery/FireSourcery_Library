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
    @file 	MCStateMachine.c
    @author FireSoucery
    @brief  MCStateMachine
    		States for input mode
        	User perspective, input filter
        	input acceptance using input state and motor state
        	motor state machine uses only motor state
    @version V0
*/
/******************************************************************************/
#include "MotorController_StateMachine.h"
#include "Utility/StateMachine/StateMachine.h"

static const StateMachine_State_T STATE_INIT;
static const StateMachine_State_T STATE_STOP;
static const StateMachine_State_T STATE_RUN;
static const StateMachine_State_T STATE_NEUTRAL;
static const StateMachine_State_T STATE_FAULT;

MotorController_StateMachine_StateId_T MotorController_StateMachine_GetStateId(MotorController_T * p_mc)
{
	MotorController_StateMachine_StateId_T id;

	if(p_mc->StateMachine.p_StateActive == &STATE_INIT)
	{
		id = MCSM_STATE_ID_INIT;
	}
	else if(p_mc->StateMachine.p_StateActive == &STATE_STOP)
	{
		id = MCSM_STATE_ID_STOP;
	}
	else if(p_mc->StateMachine.p_StateActive == &STATE_RUN)
	{
		id = MCSM_STATE_ID_RUN;
	}
	else if(p_mc->StateMachine.p_StateActive == &STATE_FAULT)
	{
		id = MCSM_STATE_ID_FAULT;
	}
	else
	{
		id = -1;
	}

	return id;
}

/******************************************************************************/
/*!
    @brief
*/
/******************************************************************************/
const StateMachine_Machine_T MCSM_MACHINE =
{
	.P_STATE_INITIAL 			= &STATE_INIT,
	.TRANSITION_TABLE_LENGTH 	= MCSM_TRANSITION_TABLE_LENGTH,
};

static StateMachine_State_T * TransitionFault(MotorController_T * p_mc) 		{return &STATE_FAULT;}

/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/
static const StateMachine_Transition_T INIT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
//	[MCSM_INPUT_TRANSITION_FAULT]		= (StateMachine_Transition_T)TransitionFault,
};

static void Init_Entry(MotorController_T * p_mc)
{
//	MotorController_InitReboot(p_mc);
}

static void Init_Proc(MotorController_T * p_mc)
{
	_StateMachine_ProcTransition(&p_mc->StateMachine, &STATE_STOP);
}

static const StateMachine_State_T STATE_INIT =
{
//	.ID 					= MCSM_STATE_ID_INIT,
	.P_TRANSITION_TABLE		= &INIT_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Init_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Init_Proc,
};


/******************************************************************************/
/*!
    @brief  Stop State

    Enters upon all motors reading 0 speed, may still be in run(active brake) or freewheel state
*/
/******************************************************************************/
static StateMachine_State_T * Stop_InputThrottle(MotorController_T * p_mc)
{
	StateMachine_State_T * p_nextState;

	if (p_mc->MainDirection == p_mc->UserDirection)
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
//	MotorController_DisableMotorAll(p_mc);
	return 0U;
}

static StateMachine_State_T * Stop_InputDirection(MotorController_T * p_mc)
{
//	/*
//	 * if this runs before motor freewheel checks speed, will beep
//	 */
//	if(MotorController_ProcDirection(p_mc) == false)
//	{
//		MotorController_Beep(p_mc);
//	}

//	return 0U;
	/*
	 * Motor Freewheel check stop should be atomic relative to this function
	 *  this runs before motor freewheel checks speed? goto fault state.
	 */
	return (MotorController_ProcDirection(p_mc) == true) ? 0U : &STATE_FAULT;
}

static StateMachine_State_T * Stop_InputSaveParams(MotorController_T * p_mc)
{
	MotorController_SaveParameters_Blocking(p_mc);
	return 0U;
}



static const StateMachine_Transition_T STOP_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Stop_InputDirection,
	[MCSM_INPUT_THROTTLE] 		= (StateMachine_Transition_T)Stop_InputThrottle,
	[MCSM_INPUT_BRAKE]  		= (StateMachine_Transition_T)Stop_InputBrake,
	[MCSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)0U, //Stop_InputRelease,
	[MCSM_INPUT_NULL] 			= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_NEUTRAL] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_SAVE_PARAMS] 	= (StateMachine_Transition_T)Stop_InputSaveParams,//todo share with calibration
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

}

static const StateMachine_State_T STATE_STOP =
{
	.P_TRANSITION_TABLE 	= &STOP_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Stop_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Stop_Proc,
};

/******************************************************************************/
/*!
    @brief  State

    During Run State Motor Controller accepts speed inputs
    motors may be in active control or freewheel
*/
/******************************************************************************/
static StateMachine_State_T * Run_InputDirection(MotorController_T * p_mc)
{
	if (p_mc->MainDirection != p_mc->UserDirection)
	{
		MotorController_Beep(p_mc);
	}

	return 0U;
}

/*
 * Do not allow for run state including both throttle and brake.
 * Alternatively, need additional brake state. allow for throttle, ignore for brake.
 */
static StateMachine_State_T * Run_InputNeutral(MotorController_T * p_mc)
{
//	return &STATE_NEUTRAL;
	return (MotorController_CheckMotorStopAll(p_mc) == true) ? &STATE_STOP : 0U;
}

static StateMachine_State_T * Run_InputNeutralStart(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc); //discontinuity during brake, todo brake flag or state
	return 0U;
}

static StateMachine_State_T * Run_InputThrottle(MotorController_T * p_mc)
{
	//or use separate neutral state to prevent return to throttle state

	if (p_mc->MainDirection == p_mc->UserDirection)
	{
		MotorController_ProcUserCmdThrottle(p_mc);
	}

	return 0U;
}


static StateMachine_State_T * Run_InputThrottleStart(MotorController_T * p_mc)
{
	//if using 2 part set/proc
	return 0U;
}

/*
 * When brake is released Coast mode will run, and check stop
 *
 * or go directly to stop so direction can change
 */
static StateMachine_State_T * Run_InputBrake(MotorController_T * p_mc)
{
	MotorController_ProcUserCmdBrake(p_mc);
	return (MotorController_CheckMotorStopAll(p_mc) == true) ? &STATE_STOP : 0U;
}

static StateMachine_State_T * Run_InputBrakeStart(MotorController_T * p_mc)
{
	//if using 2 part set/proc
	return 0U;
}

/*
 * input release
 */
static StateMachine_State_T * Run_InputCoast(MotorController_T * p_mc)
{
 	if(p_mc->Parameters.CoastMode == MOTOR_CONTROLLER_COAST_MODE_REGEN)
 	{
// 		Motor_UserN_SetRegenCmd(p_mc->CONFIG.P_MOTORS, p_mc->CONFIG.MOTOR_COUNT);
 		MotorController_ProcUserCmdRegen(p_mc);
 	}
 	else
 	{
 		//motor already disabled
 	}

	return (MotorController_CheckMotorStopAll(p_mc) == true) ? &STATE_STOP : 0U;
}

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

static const StateMachine_Transition_T RUN_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 			= (StateMachine_Transition_T)TransitionFault,
	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)Run_InputDirection,
	[MCSM_INPUT_THROTTLE] 		= (StateMachine_Transition_T)Run_InputThrottle,
	[MCSM_INPUT_BRAKE]  		= (StateMachine_Transition_T)Run_InputBrake,
	[MCSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)Run_InputCoastStart,
	[MCSM_INPUT_NULL] 			= (StateMachine_Transition_T)Run_InputCoast,
	[MCSM_INPUT_NEUTRAL] 		= (StateMachine_Transition_T)Run_InputNeutral,
	[MCSM_INPUT_NEUTRAL_START]	= (StateMachine_Transition_T)Run_InputNeutralStart,
	[MCSM_INPUT_SAVE_PARAMS] 	= (StateMachine_Transition_T)0U,
};

static void Run_Entry(MotorController_T * p_mc)
{

}

static void Run_Proc(MotorController_T * p_mc)
{

}

static const StateMachine_State_T STATE_RUN =
{
	.P_TRANSITION_TABLE 	= &RUN_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Run_Entry,
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
//	if(MotorController_CheckMotorStopAll(p_mc) == true)
//	{
//		StateMachine_ProcTransition(&p_mc->StateMachine, &STATE_STOP);
//	}
//}
//
//static const StateMachine_State_T STATE_NEUTRAL =
//{
//	.P_TRANSITION_TABLE 	= &NEUTRAL_TRANSITION_TABLE[0U],
//	.ON_ENTRY 				= (StateMachine_Output_T)Neutral_Entry,
//	.OUTPUT 				= (StateMachine_Output_T)Neutral_Proc,
//};


/******************************************************************************/
/*!
    @brief  State
*/
/******************************************************************************/

/*
 * Manual Check fault
 */
static StateMachine_State_T * Fault_InputFault(MotorController_T * p_mc)
{
	return (p_mc->ErrorFlags.State == 0U) ? &STATE_STOP : 0U;
}

static const StateMachine_Transition_T FAULT_TRANSITION_TABLE[MCSM_TRANSITION_TABLE_LENGTH] =
{
	[MCSM_INPUT_FAULT] 			= (StateMachine_Transition_T)Fault_InputFault,
	[MCSM_INPUT_DIRECTION] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_THROTTLE] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_BRAKE]  		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_RELEASE] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_NULL] 			= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_NEUTRAL] 		= (StateMachine_Transition_T)0U,
	[MCSM_INPUT_SAVE_PARAMS] 	= (StateMachine_Transition_T)0U,
};

static void Fault_Entry(MotorController_T * p_mc)
{
	MotorController_Beep(p_mc); //todo set periodic
	Timer_StartPeriod(&p_mc->StateTimer, 1000U);
	MotorController_SetFaultRecord(p_mc);
}

static void Fault_Proc(MotorController_T * p_mc)
{
	MotorController_DisableMotorAll(p_mc);

	if(Thermistor_GetStatus(&p_mc->ThermistorPcb) == THERMISTOR_THRESHOLD_OK)
	{
		p_mc->ErrorFlags.PcbOverHeat = 0U;
	}

	if(Thermistor_GetStatus(&p_mc->ThermistorMosfetsTop) == THERMISTOR_THRESHOLD_OK)
	{
		p_mc->ErrorFlags.MosfetsTopOverHeat = 0U;
	}

	if(Thermistor_GetStatus(&p_mc->ThermistorMosfetsBot) == THERMISTOR_THRESHOLD_OK)
	{
		p_mc->ErrorFlags.MosfetsBotOverHeat = 0U;
	}

	if(VMonitor_CheckLimits(&p_mc->VMonitorPos, p_mc->AnalogResults.VPos_ADCU) == VMONITOR_LIMITS_OK)
	{
		p_mc->ErrorFlags.VPosLimit = 0U;
	}

	if(VMonitor_CheckLimits(&p_mc->VMonitorSense, p_mc->AnalogResults.VSense_ADCU) == VMONITOR_LIMITS_OK)
	{
		p_mc->ErrorFlags.VSenseLimit = 0U;
	}

	if(VMonitor_CheckLimits(&p_mc->VMonitorAcc, p_mc->AnalogResults.VAcc_ADCU) == VMONITOR_LIMITS_OK)
	{
		p_mc->ErrorFlags.VAccLimit = 0U;
	}

//	bool errorCleared = false;
//	if (p_mc->ErrorFlags.PcbOverHeat = 1U)
//	{
//		if(Thermistor_GetStatus(&p_mc->ThermistorPcb) == THERMISTOR_THRESHOLD_OK)
//		{
//			p_mc->ErrorFlags.PcbOverHeat = 0U;
//			errorCleared = true;
//		}
//	}
//
//	if (p_mc->ErrorFlags.MosfetsTopOverHeat = 1U)
//	{
//		if(Thermistor_GetStatus(&p_mc->ThermistorMosfetsTop) == THERMISTOR_THRESHOLD_OK)
//		{
//			p_mc->ErrorFlags.MosfetsTopOverHeat = 0U;
//			errorCleared = true;
//		}
//	}
//	if ((p_mc->ErrorFlags.State == 0U) && (errorCleared == true))// and motor error flags
//	{
////		StateMachine_ProcTransition(&p_mc->StateMachine, &STATE_STOP);
//	}
//	else
//	{
////		MotorController_ProcErrorOutput(p_mc); //alarm etc
//	}
}

static const StateMachine_State_T STATE_FAULT =
{
	.P_TRANSITION_TABLE 	= &FAULT_TRANSITION_TABLE[0U],
	.ON_ENTRY 				= (StateMachine_Output_T)Fault_Entry,
	.OUTPUT 				= (StateMachine_Output_T)Fault_Proc,
};
