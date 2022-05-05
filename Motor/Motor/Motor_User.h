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
	@file 	Motor_User.h
	@author FireSoucery
	@brief  User Interface. Motor module public functions.
			Functions include error checking.
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor_StateMachine.h"
#include "Motor_FOC.h"

#include <stdint.h>
#include <stdbool.h>
 
extern void Motor_User_DisableControl(Motor_T * p_motor);
extern void Motor_User_Ground(Motor_T * p_motor);
extern bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern bool Motor_User_SetDirectionForward(Motor_T * p_motor);
extern bool Motor_User_SetDirectionReverse(Motor_T * p_motor);

extern void Motor_User_SetCmdSigned(Motor_T * p_motor, int32_t cmd);
extern void Motor_User_SetCmdUnsigned(Motor_T * p_motor, uint16_t cmd); // user match to mode
extern void Motor_User_SetCmd(Motor_T * p_motor, int32_t userCmd);
extern void Motor_User_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode);

/******************************************************************************/
/*!
	Control Mode Wrappers
*/
/******************************************************************************/
static inline void Motor_User_SetVoltageMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE);
}

/*! 
	@param[in] voltage [-65536:65535]
*/
static inline void Motor_User_SetVoltageCmd(Motor_T * p_motor, int32_t voltage)
{
	Motor_User_SetVoltageMode(p_motor);
	Motor_User_SetCmdSigned(p_motor, voltage);
}

static inline void Motor_User_SetTorqueMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_CURRENT);
}

/*! 
	@param[in] torque [-65536:65535]
*/
static inline void Motor_User_SetTorqueCmd(Motor_T * p_motor, int32_t torque)
{
	Motor_User_SetTorqueMode(p_motor);
	Motor_User_SetCmdSigned(p_motor, torque);
}

/*
	Default speed mode is speed torque mode
*/
static inline void Motor_User_SetSpeedMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT);
}

/*! 
	@param[in] speed [0:65535]
*/
static inline void Motor_User_SetSpeedCmd(Motor_T * p_motor, uint16_t speed)
{
	Motor_User_SetSpeedMode(p_motor);
	Motor_User_SetCmdUnsigned(p_motor, speed); 
}

/*! 
	todo
	@param[in] angle [0:65535]
*/
// static inline void Motor_User_SetPositionCmd(Motor_T * p_motor, uint16_t angle)
// { 
//	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_POSITION);
//	Motor_User_SetCmd(p_motor, angle); 
// }

static inline void Motor_User_SetThrottleMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, p_motor->Parameters.ControlMode);
}

/*!
	@param[in] throttle [0:65535]
*/
static inline void Motor_User_SetThrottleCmd(Motor_T * p_motor, uint16_t throttle)
{ 
	Motor_User_SetThrottleMode(p_motor); 
	Motor_User_SetCmd(p_motor, throttle);
}

/*!
	Always request opposite direction current
	req opposite iq, bound vq to 0 for no plugging brake

	transition from accelerating to decelerating, 
	use signed ramp to transition through 0 without discontinuity
	ramp from in-direction torque to 0 to counter-direction torque

	@param[in] brake [0:65535]
*/
static inline void Motor_User_SetBrakeCmd(Motor_T * p_motor, uint16_t brake)
{
	int32_t torque = (int32_t)0 - (int32_t)brake;

	// if(p_motor->ControlModeFlags.Hold == 0U)
	// {
	// 	if(Motor_GetSpeed_RPM(p_motor) > 10U)
	// 	{
			Motor_User_SetTorqueCmd(p_motor, torque);
	// 	}
	// 	else
	// 	{
	// 		p_motor->ControlModeFlags.Hold = 1U;  //clears on throttle
	// 		Phase_Ground(&p_motor->Phase);
	// 	}
	// }
}

/* 
	ramped speed match provides smoother change 
	alternatively, V/F mode
*/
static inline void Motor_User_SetVoltageBrakeCmd(Motor_T * p_motor)
{
	// if(Motor_GetSpeed_RPM(p_motor) > 30U)
	// {
		Motor_User_SetVoltageCmd(p_motor, p_motor->SpeedFeedback_Frac16 * 3 / 4); 
	// }
	// else
	// {
	// 	Motor_User_DisableControl(p_motor);   //fix repeat
	// }
}
 
/*
	Calibration State - Run Calibration functions
*/
static inline void Motor_User_ActivateCalibrationHall(Motor_T * p_motor)
{
	p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_HALL;
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor)
{
	p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ENCODER;
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor)
{
	p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ADC;
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationSinCos(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_SIN_COS)
	{
		p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_SIN_COS;
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
	}
}

/*
	Fault State - checks exit
	Other States - user initated tranisition to fault state
*/
static inline void Motor_User_ProcFault(Motor_T * p_motor)
{
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
}
 
static inline bool Motor_User_ClearFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) == MSM_STATE_ID_FAULT)
	{
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
	}

	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT);
}

static inline void Motor_User_SetFault(Motor_T * p_motor)
{
	if(StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT)
	{
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
	}
}

static inline bool Motor_User_CheckFault(Motor_T * p_motor)
{
	return (StateMachine_GetActiveStateId(&p_motor->StateMachine) != MSM_STATE_ID_FAULT);
}

//static inline void Motor_User_ActivatePhase(Motor_T * p_motor)
//{
//
//} 

/******************************************************************************/
/*
	RAM Variable RW
	Conversion Function only called if called by the user. Not called regularly
*/
/******************************************************************************/ 
static inline Motor_StateMachine_StateId_T Motor_User_GetStateId(Motor_T * p_motor) 						{ return StateMachine_GetActiveStateId(&p_motor->StateMachine); }
static inline uint16_t Motor_User_GetMotorAdcu(Motor_T * p_motor, MotorAnalog_Channel_T adcChannel) 		{ return p_motor->AnalogResults.Channels[adcChannel]; }
static inline uint16_t Motor_User_GetMotorAdcu_Msb8(Motor_T * p_motor, MotorAnalog_Channel_T adcChannel) 	{ return Motor_User_GetMotorAdcu(p_motor, adcChannel) >> (ADC_BITS - 8U); }

static inline uint16_t Motor_User_GetSpeed_RPM(Motor_T * p_motor) 					{ return p_motor->SpeedFeedback_Frac16 * p_motor->Parameters.SpeedRefMax_RPM / 65536U; }
static inline int32_t Motor_User_GetHeat_DegC(Motor_T * p_motor, uint16_t scalar) 	{ return Thermistor_ConvertToDegC_Int(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, scalar); }
static inline float Motor_User_GetHeat_DegCFloat(Motor_T * p_motor) 				{ return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU); } 

// static inline uint16_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)		{ return Linear_Voltage_CalcFractionUnsigned16(&p_motor->UnitVabc, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf)); }
// static inline uint32_t Motor_User_GetBemf_V(Motor_T * p_motor)			{ return Linear_Voltage_CalcV(&p_motor->UnitVabc, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf)); } 
// static inline Motor_ErrorFlags_T Motor_User_GetErrorFlags(Motor_T * p_motor) { return p_motor->ErrorFlags; }
/*
	User output only
*/
typedef union Motor_User_StatusFlags_T
{
	struct
	{
		uint32_t ILimitFault 		: 1U;
		uint32_t ILimitActive 		: 1U;
		uint32_t IWarning 			: 1U;
		// uint32_t HeatOverLimit 		: 1U;
		// uint32_t HeatOverThreshold 	: 1U;
		// uint32_t HeatOverWarning 	: 1U;

		Thermistor_Status_T HeatStatus 	: 2U;
	};
	uint32_t State;
} 
Motor_User_StatusFlags_T;

static inline Motor_User_StatusFlags_T Motor_User_GetStatusFlags(Motor_T * p_motor) 
{
	Motor_User_StatusFlags_T status ;

	status.HeatStatus = Thermistor_GetStatus(&p_motor->Thermistor);  
	status.IWarning = p_motor->RunStateFlags.IWarningActive;
	status.ILimitActive = p_motor->RunStateFlags.ILimitActive;
	
	// status.ILimitFault = p_motor->RunStateFlags.ILimitActive;

	return status;
}

/*
	supported modes only, or implement function for 
*/
static inline uint16_t Motor_User_GetMechanialAngle(Motor_T * p_motor)
{
	uint16_t angle;
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_SENSORLESS: 	angle = 0; 	break;
		case MOTOR_SENSOR_MODE_HALL: 		angle = 0; 	break;
		case MOTOR_SENSOR_MODE_ENCODER: 	angle = Encoder_Motor_GetMechanicalAngle(&p_motor->Encoder);	break;
		case MOTOR_SENSOR_MODE_SIN_COS: 	angle = SinCos_GetMechanicalAngle(&p_motor->SinCos); 			break;
		default: 	break;
	}
	return angle;
}

static inline uint16_t Motor_User_GetIPhase_Frac16(Motor_T * p_motor)
{
	uint16_t iPhase;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		iPhase = Motor_FOC_GetIMagnitude_Frac16(p_motor);
	}
	else
	{

	}

	return iPhase;
}

static inline uint16_t Motor_User_GetIPhase_Amp(Motor_T * p_motor)
{ 
	return (uint32_t)Motor_User_GetIPhase_Frac16(p_motor) * p_motor->Parameters.IRefMax_Amp / 65536U; 
}

/******************************************************************************/
/*
	Nvm Param function
*/
/******************************************************************************/

/* Persistent Control Mode */
static inline void Motor_User_SetControlModeParam(Motor_T * p_motor, Motor_ControlMode_T mode) { p_motor->Parameters.ControlMode = mode; }
/* Persistent ILimit */
static inline void Motor_User_SetILimitParam_Amp(Motor_T * p_motor, uint16_t motoring_Amp, uint16_t generating_Amp)
{
	uint16_t limit_Frac16 = (uint32_t)limit_Amp * 65536U / p_motor->Parameters.IRefMax_Amp;
	if(limit_Frac16 > 65535U) { limit_Frac16 = 65535U; }

	// p_motor->Parameters.ILimitMotoring_Frac16 = limit_Frac16;
	// p_motor->Parameters.ILimitGenerating_Frac16 = limit_Frac16; 

	// p_motor->Parameters.IBusLimit_Frac16 = limit_Frac16;
	// p_motor->Parameters.IqLimit = limit_Frac16 / 2U;
}

/* effective speed control mode only */
static inline void Motor_User_SetSpeedLimitParam_RPM(Motor_T * p_motor, uint16_t forward_Rpm, uint16_t reverse_Rpm)
{
	uint32_t forward_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, forward_Rpm);
	uint32_t reverse_Frac16 = Motor_ConvertToSpeedFrac16(p_motor, reverse_Rpm); //saturate

	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		p_motor->Parameters.SpeedLimitCcw_Frac16 = forward_Frac16;
		p_motor->Parameters.SpeedLimitCw_Frac16 = reverse_Frac16;
	}
	else
	{
		p_motor->Parameters.SpeedLimitCcw_Frac16 = reverse_Frac16;
		p_motor->Parameters.SpeedLimitCw_Frac16 = forward_Frac16;
	}
}

/* 
	Active speed limit  
	MotorRef -> UserParam -> Direction -> Divider
*/
static inline void Motor_User_SetSpeedLimitActive_Factor(Motor_T * p_motor, uint16_t factor_frac16)
{
	uint32_t limit = factor_frac16 * p_motor->Parameters.SpeedLimitParam_Frac16 / 65536U;
	PID_SetOutputLimits(&p_motor->PidSpeed, 0 - limit, limit);
	// p_motor->SpeedLimitActive = factor_frac16 * p_motor->Parameters.SpeedLimitParam_Frac16 / 65536U;
}


/* SpeedRef match to motor voltage max */
static inline void Motor_User_SpeedRefMax_RPM(Motor_T * p_motor, uint16_t rpm) { p_motor->Parameters.SpeedRefMax_RPM = rpm; } 


static inline void Motor_User_SetIRefMax_Amp(Motor_T * p_motor, uint16_t amp)
{
	p_motor->Parameters.IRefMax_Amp = amp;
}

static inline void Motor_User_SetIRefZeroToPeak_ADCU(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IRefPeak_ADCU = adcu;
}

static inline void Motor_User_SetIRefZeroToPeak_MilliV(Motor_T * p_motor, uint16_t min_MilliV, uint16_t max_MilliV)
{
	uint16_t adcuZero = (uint32_t)(max_MilliV + min_MilliV) * ADC_MAX / 2U / _Motor_GetAdcVRef();
	uint16_t adcuRef = (uint32_t)max_MilliV * ADC_MAX / _Motor_GetAdcVRef();

	p_motor->Parameters.IRefPeak_ADCU = adcuRef - adcuZero;
}

static inline void Motor_User_SetIaZero_ADCU(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IaRefZero_ADCU = adcu; }
static inline void Motor_User_SetIbZero_ADCU(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IbRefZero_ADCU = adcu; }
static inline void Motor_User_SetIcZero_ADCU(Motor_T * p_motor, uint16_t adcu) { p_motor->Parameters.IcRefZero_ADCU = adcu; }
static inline void Motor_User_SetIaMax_ADCU(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IaRefMax_ADCU = adcu;
}
static inline void Motor_User_SetIbMax_ADCU(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IbRefMax_ADCU = adcu;
}
static inline void Motor_User_SetIcMax_ADCU(Motor_T * p_motor, uint16_t adcu)
{
	p_motor->Parameters.IcRefMax_ADCU = adcu;
}


static inline void Motor_User_SetRef(Motor_T * p_motor, uint16_t vSupply, uint16_t vMotor, uint16_t speedVMotor_Rpm)
{
	p_motor->Parameters.VSupply = vSupply;
	p_motor->Parameters.SpeedRefMax_RPM = speedVMotor_Rpm * vSupply / vMotor; 
	p_motor->Parameters.SpeedRefVoltage_RPM = speedVMotor_Rpm * vSupply / vMotor;
}


static inline void Motor_User_SetSpeedRefMax_VRatio(Motor_T * p_motor, uint16_t vSupply, uint16_t vMotor, uint16_t speedVMotor)
{
	p_motor->Parameters.SpeedRefMax_RPM = speedVMotor * vSupply / vMotor;

	p_motor->Parameters.SpeedRefVoltage_RPM = speedVMotor * vSupply / vMotor;
}

static inline void Motor_User_SetSpeedRefVoltage_VScale(Motor_T * p_motor, uint16_t vSupply, uint16_t vMotor, uint16_t speedVMotor)
{
	p_motor->Parameters.SpeedRefVoltage_RPM = speedVMotor * vSupply / vMotor;
}
static inline void Motor_User_SetVSupplyVMotorScale(Motor_T * p_motor, uint16_t vSupply, uint16_t vMotor)
{

}

static inline void Motor_User_SetSpeedRefVoltage_RPM(Motor_T * p_motor, uint16_t rpm)
{
	p_motor->Parameters.SpeedRefVoltage_RPM = rpm;
} 


static inline void Motor_User_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode) 						{ p_motor->Parameters.CommutationMode = mode; }
static inline void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode) 								{ p_motor->Parameters.SensorMode = mode; }
static inline void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T mode) 			{ p_motor->Parameters.DirectionCalibration = mode; }
static inline Motor_CommutationMode_T Motor_User_GetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode) 	{ return p_motor->Parameters.CommutationMode; }
static inline Motor_SensorMode_T Motor_User_GetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode) 					{ return p_motor->Parameters.SensorMode; }
static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T * p_motor) 						{ return p_motor->Parameters.DirectionCalibration; }



 
/*
	Additional module wrapper
*/
static inline Hall_Sensors_T Motor_User_ReadHall(Motor_T * p_motor) { return Hall_ReadSensors(&p_motor->Hall); }
static inline bool Motor_User_GetHallA(Motor_T * p_motor) { return Hall_GetSensorA(&p_motor->Hall);; }
static inline bool Motor_User_GetHallB(Motor_T * p_motor) { return Hall_GetSensorB(&p_motor->Hall); }
static inline bool Motor_User_GetHallC(Motor_T * p_motor) { return Hall_GetSensorC(&p_motor->Hall); }
static inline uint16_t Motor_User_GetHallRotorAngle(Motor_T * p_motor) { return Hall_GetRotorAngle_Degrees16(&p_motor->Hall); }
//static inline void Motor_User_SetPidIqKp(Motor_T * p_motor, uint16_t kpFactor)
//{
//	p_motor->CONFIG.P_MOTORS[motorIndex].PidIq.Params.KpFactor = kpFactor;
//}

/******************************************************************************/
/*!
	extern
*/
/******************************************************************************/
extern void Motor_UserN_SetCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd);
extern void Motor_UserN_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle);
extern void Motor_UserN_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake);
extern void Motor_UserN_SetVoltageBrakeCmd(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount);
extern void Motor_UserN_DisableControl(Motor_T * p_motor, uint8_t motorCount);
extern void Motor_UserN_Ground(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_CheckErrorFlags(Motor_T * p_motor, uint8_t motorCount);
extern bool Motor_UserN_ClearFault(Motor_T * p_motor, uint8_t motorCount);

#endif
