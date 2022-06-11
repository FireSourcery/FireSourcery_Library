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
	@file 	MotorController_Thread.h
	@author FireSoucery
	@brief  MotorController module functions must be placed into corresponding user app threads.
			Most outer functions to call from MCU app.
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_CONTROLLER_THREAD_H
#define MOTOR_CONTROLLER_THREAD_H

#include "MotorController_User.h"
#include "Motor/Motor/Motor_Thread.h"

/******************************************************************************/
/*
	Mapped to Thread - Proc in All States
*/
/******************************************************************************/

static inline void _MotorController_ProcAnalogUser(MotorController_T * p_mc)
{
	MotAnalogUser_Cmd_T cmd = MotAnalogUser_PollCmd(&p_mc->AnalogUser);
	MotAnalogUser_CaptureInput(&p_mc->AnalogUser, p_mc->AnalogResults.Throttle_Adcu, p_mc->AnalogResults.Brake_Adcu);

	AnalogN_Group_PauseQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_USER);
	AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_THROTTLE);
	AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_BRAKE);
	AnalogN_Group_ResumeQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_USER);

	/* Assume no input cmd priority level, although implmented */
	switch(cmd)
	{
		case MOT_ANALOG_USER_CMD_SET_BRAKE:					MotorController_User_SetCmdBrake(p_mc, MotAnalogUser_GetBrakeValue(&p_mc->AnalogUser));			break;
		case MOT_ANALOG_USER_CMD_SET_THROTTLE:				MotorController_User_SetCmdThrottle(p_mc, MotAnalogUser_GetThrottleValue(&p_mc->AnalogUser));	break;
		case MOT_ANALOG_USER_CMD_SET_THROTTLE_RELEASE:		MotorController_User_SetReleaseThrottle(p_mc);													break;
		case MOT_ANALOG_USER_CMD_SET_BRAKE_RELEASE:			MotorController_User_SetReleaseBrake(p_mc);														break;
		case MOT_ANALOG_USER_CMD_SET_NEUTRAL:				MotorController_User_SetNeutral(p_mc); 															break;
		case MOT_ANALOG_USER_CMD_PROC_NEUTRAL:				MotorController_User_ProcNeutral(p_mc);															break;
		case MOT_ANALOG_USER_CMD_SET_DIRECTION_FORWARD: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_FORWARD);					break;
		case MOT_ANALOG_USER_CMD_SET_DIRECTION_REVERSE: 	MotorController_User_SetDirection(p_mc, MOTOR_CONTROLLER_DIRECTION_REVERSE);	 				break;
		case MOT_ANALOG_USER_CMD_PROC_NO_INPUT:				MotorController_User_ProcRelease(p_mc);															break;
		default: break;
	}
}

/*
	Optional Din
*/
static inline void _MotorController_ProcOptDin(MotorController_T * p_mc)
{
	if(p_mc->Parameters.OptDinFunction != MOTOR_CONTROLLER_OPT_DIN_DISABLE)
	{
		if(Debounce_CaptureState(&p_mc->OptDin) == true)
		{
			if(Debounce_PollRisingEdge(&p_mc->OptDin) == true) /* Compiler optimize to Debounce_GetState */
			{
				switch(p_mc->Parameters.OptDinFunction)
				{
					case MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT: MotorController_SetSpeedLimitMotorAll(p_mc, p_mc->Parameters.OptDinSpeedLimit_Frac16); break;
					default: break;
				}
			}
			else if(Debounce_PollFallingEdge(&p_mc->OptDin) == true)
			{
				switch(p_mc->Parameters.OptDinFunction)
				{
					case MOTOR_CONTROLLER_OPT_DIN_SPEED_LIMIT: MotorController_ClearSpeedLimitMotorAll(p_mc); break;
					default: break;
				}
			}
		}
	}
}

/* Monitor Threads only set fault flags. Do not clear until user input */

static inline void _MotorController_ProcHeatMonitor(MotorController_T * p_mc)
{
	bool isFault = false;
	bool isWarning = false;

	AnalogN_Group_PauseQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_HEAT);
	AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT_PCB);
	if(Thermistor_GetIsMonitorEnable(&p_mc->ThermistorMosfetsTop))
	{
		AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT_MOSFETS_TOP);
	}
	if(Thermistor_GetIsMonitorEnable(&p_mc->ThermistorMosfetsBot))
	{
		AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_HEAT_MOSFETS_BOT);
	}
	AnalogN_Group_ResumeQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_HEAT);

	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++)
	{
		Motor_Heat_Thread(&p_mc->CONFIG.P_MOTORS[iMotor]);
		if		(Thermistor_GetIsShutdown(&p_mc->CONFIG.P_MOTORS[iMotor].Thermistor) == true) 	{ p_mc->FaultFlags.MotorOverHeat = 1U; isFault = true; }
		else if	(Thermistor_GetIsWarning(&p_mc->CONFIG.P_MOTORS[iMotor].Thermistor) == true) 	{ isWarning = true; }
	}

	Thermistor_PollMonitor(&p_mc->ThermistorPcb, p_mc->AnalogResults.HeatPcb_Adcu);
	Thermistor_PollMonitor(&p_mc->ThermistorMosfetsTop, p_mc->AnalogResults.HeatMosfetsTop_Adcu);
	Thermistor_PollMonitor(&p_mc->ThermistorMosfetsBot, p_mc->AnalogResults.HeatMosfetsBot_Adcu);

	/* Shutdown repeat set ok */
	if(Thermistor_GetIsShutdown(&p_mc->ThermistorPcb) == true) 			{ p_mc->FaultFlags.PcbOverHeat = 1U; isFault = true; }
	if(Thermistor_GetIsShutdown(&p_mc->ThermistorMosfetsTop) == true) 	{ p_mc->FaultFlags.MosfetsTopOverHeat = 1U; isFault = true; }
	if(Thermistor_GetIsShutdown(&p_mc->ThermistorMosfetsBot) == true) 	{ p_mc->FaultFlags.MosfetsBotOverHeat = 1U; isFault = true; }

	if(isFault == true)
	{
		MotorController_User_SetFault(p_mc);
	}
	else
	{
		/* Warning behaviors edge triggered */
		isWarning =
			Thermistor_GetIsWarning(&p_mc->ThermistorPcb) ||
			Thermistor_GetIsWarning(&p_mc->ThermistorMosfetsTop) ||
			Thermistor_GetIsWarning(&p_mc->ThermistorMosfetsBot);

		if(isWarning == true)
		{
			/*
				Thermistor Adcu is roughly linear in Warning region
				Assume HeatMostfetTop as highest heat
				constantly compares lowest active limit, alternatively, check and restore prev on clear limit
			*/
			MotorController_SetILimitMotorAll
			(
				p_mc,
				Linear_Function(&p_mc->HeatILimitRate, p_mc->AnalogResults.HeatMosfetsTop_Adcu),
				MOTOR_CONTROLLER_I_LIMIT_ACTIVE_HEAT
			);

			if(p_mc->WarningFlags.Heat == false)
			{
				Blinky_BlinkN(&p_mc->Buzzer, 250U, 500U, 2U);
				p_mc->WarningFlags.Heat = true;
			}
		}
		else
		{
			if(p_mc->WarningFlags.Heat == true)
			{
				MotorController_ClearILimitMotorAll(p_mc, MOTOR_CONTROLLER_I_LIMIT_ACTIVE_HEAT);
				p_mc->WarningFlags.Heat = false;
			}
		}
	}
}

static inline void _MotorController_ProcVoltageMonitor(MotorController_T * p_mc)
{
	bool isFault = false;

	AnalogN_Group_PauseQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);
	AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VACC);
	AnalogN_Group_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VSENSE);
	AnalogN_Group_ResumeQueue(p_mc->CONFIG.P_ANALOG_N, p_mc->CONFIG.ANALOG_CONVERSIONS.ADCS_GROUP_V);

	VMonitor_PollStatus(&p_mc->VMonitorSense, p_mc->AnalogResults.VSense_Adcu);
	VMonitor_PollStatus(&p_mc->VMonitorAcc, p_mc->AnalogResults.VAcc_Adcu);
	if(VMonitor_GetIsStatusLimit(&p_mc->VMonitorSense) == true) { p_mc->FaultFlags.VSenseLimit = 1U; isFault = true; }
	if(VMonitor_GetIsStatusLimit(&p_mc->VMonitorAcc) == true) 	{ p_mc->FaultFlags.VAccLimit = 1U; isFault = true; }

	if(isFault == true) { MotorController_User_SetFault(p_mc); } 	/* Sensors checks fault only */
}

/*
	High Freq, Low Priority, Main
*/
static inline void MotorController_Main_Thread(MotorController_T * p_mc)
{
	/* Med Freq, Low Priority, 1 ms */
	if(Timer_Poll(&p_mc->TimerMillis) == true)
	{
		StateMachine_Semi_ProcOutput(&p_mc->StateMachine);

		switch(p_mc->Parameters.InputMode)
		{
			case MOTOR_CONTROLLER_INPUT_MODE_ANALOG: _MotorController_ProcAnalogUser(p_mc);	break;
			case MOTOR_CONTROLLER_INPUT_MODE_PROTOCOL:
				if(MotAnalogUser_PollBrakePinRisingEdge(&p_mc->AnalogUser) == true) { MotorController_User_DisableControl(p_mc); }
				if(Protocol_CheckRxLost(&p_mc->CONFIG.P_PROTOCOLS[0U]) == true)
				{
					MotorController_User_DisableControl(p_mc);
					MotorController_User_SetFault(p_mc);
					p_mc->FaultFlags.RxLost = 1U;
				}
				break;

			case MOTOR_CONTROLLER_INPUT_MODE_CAN: break;
			default:  break;
		}

		/* Bypass Isr */
		// for(uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++)
		// {
		// 	Serial_PollRxData(&p_mc->CONFIG.P_SERIALS[iSerial]);
		// 	Serial_PollTxData(&p_mc->CONFIG.P_SERIALS[iSerial]);
		// }

		for(uint8_t iProtocol = 0U; iProtocol < p_mc->CONFIG.PROTOCOL_COUNT; iProtocol++) { Protocol_Proc(&p_mc->CONFIG.P_PROTOCOLS[iProtocol]); }
		if(p_mc->Parameters.IsCanEnable == true) { CanBus_ProcServices(p_mc->CONFIG.P_CAN_BUS); }
	}

	/* Low Freq, Low Priority, 10ms */
	if(Timer_Poll(&p_mc->TimerMillis10) == true)
	{
		Shell_Proc(&p_mc->Shell);
		Blinky_Proc(&p_mc->Buzzer);
	}

	/* Low Freq, Low Priority, 1s */
	if(Timer_Poll(&p_mc->TimerSeconds) == true)
	{
		/* In case of Serial Rx Overflow Timeout */
		for(uint8_t iSerial = 0U; iSerial < p_mc->CONFIG.SERIAL_COUNT; iSerial++) { Serial_PollRestartRxIsr(&p_mc->CONFIG.P_SERIALS[iSerial]); }

		/* Can use low priority check, as motor is already in fault state */
		if(MotorController_CheckFaultMotorAll(p_mc) != 0U) {  p_mc->FaultFlags.Motors = 1U; MotorController_User_SetFault(p_mc); }

		_MotorController_ProcOptDin(p_mc);
	}
}


/*
	Med Freq, Med-High Priority

	Thread Safety: Async Fault proc is okay?
*/
static inline void MotorController_Timer1Ms_Thread(MotorController_T * p_mc)
{
//	BrakeThread(p_mc);
#if !defined(CONFIG_MOTOR_V_SENSORS_ISOLATED) && defined(CONFIG_MOTOR_V_SENSORS_ADC)
	VMonitor_Status_T statusVPos = VMonitor_PollStatus(&p_mc->VMonitorPos, p_mc->AnalogResults.VPos_Adcu);

	AnalogN_EnqueueConversion(p_mc->CONFIG.P_ANALOG_N, &p_mc->CONFIG.ANALOG_CONVERSIONS.CONVERSION_VPOS);

	switch(statusVPos)
	{
		case VMONITOR_LIMIT_UPPER: p_mc->FaultFlags.VPosLimit = 1U; MotorController_User_SetFault(p_mc); break;
		case VMONITOR_LIMIT_LOWER: p_mc->FaultFlags.VPosLimit = 1U;	MotorController_User_SetFault(p_mc); break;
		case VMONITOR_WARNING_UPPER: break;
		case VMONITOR_WARNING_LOWER:
			if(p_mc->WarningFlags.LowV == false)
			{
				MotorController_SetILimitMotorAll(p_mc, p_mc->Parameters.ILimitScalarOnLowV_Frac16, MOTOR_CONTROLLER_I_LIMIT_ACTIVE_LOW_V);
				Blinky_BlinkN(&p_mc->Buzzer, 250U, 500U, 2U);
				p_mc->WarningFlags.LowV = true;
			}
			break;
		case VMONITOR_STATUS_OK:
			if(p_mc->WarningFlags.LowV == true)
			{
				MotorController_ClearILimitMotorAll(p_mc, MOTOR_CONTROLLER_I_LIMIT_ACTIVE_LOW_V);
				p_mc->WarningFlags.LowV = false;
			}
			break;
		default: break;
	}
#endif

	if(Timer_Poll(&p_mc->TimerIsrDividerSeconds) == true)
	{
		_MotorController_ProcHeatMonitor(p_mc);
		_MotorController_ProcVoltageMonitor(p_mc);
	}
}

/*
	High Freq, High Priority
*/
static inline void MotorController_PWM_Thread(MotorController_T * p_mc)
{
	for(uint8_t iMotor = 0U; iMotor < p_mc->CONFIG.MOTOR_COUNT; iMotor++) { Motor_PWM_Thread(&p_mc->CONFIG.P_MOTORS[iMotor]); }
}

#endif
