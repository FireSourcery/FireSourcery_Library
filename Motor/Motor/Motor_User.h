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
    @brief  User Interface. Functions include error checking.
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_USER_H
#define MOTOR_USER_H

#include "Motor_StateMachine.h"
#include "Motor.h"
#include "Utility/StateMachine/StateMachine.h"

#include <stdint.h>
#include <stdbool.h>

/*
 *
 * +/- Sign indicates along or against direction selected. NOT absolute CW/CCW
 */


/******************************************************************************/
/*!
 * State Machine Error checked inputs
 */
/******************************************************************************/
/*
 * Motor State Machine Thread Safety
 *
 * SemiSync Mode -
 * State Proc in PWM thread. User Input in Main Thread. May need critical section during input.
 * Sync error only occurs in way of running new states function, using false validated function of old state.
 * Can always recover? Stop to Run transition always at 0 speed.

 * Set async to proc, sync issue can recover?,
 * control proc may overwrite pid state set, but last to complete is always user input

 * Sync Mode
 * Must check input flags every pwm cycle
 */

/*
 * Disable control, motor may remain spinning
 */
static inline void Motor_User_DisableControl(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
}

static inline void Motor_User_DisableControlForce(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FLOAT);
}

static inline void Motor_User_Ground(Motor_T * p_motor)
{
	Phase_Ground(&p_motor->Phase);

	//alternatively only ground in stop state
//	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_GROUND);
}


/*
 * set buffered direction, check on state machine
 */
static inline bool Motor_User_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->UserDirection = direction; //pass to state machine
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION);
//	return StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_DIRECTION, direction);

	return (p_motor->UserDirection == p_motor->Direction);
}

static inline bool Motor_User_SetDirectionForward(Motor_T * p_motor)
{
	bool isSet;

	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CCW) ? true : false;
	}
	else
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CW) ? true : false;
	}

	return isSet;
}

static inline bool Motor_User_SetDirectionReverse(Motor_T * p_motor)
{
	bool isSet;

	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CW) ? true : false;
	}
	else
	{
		isSet = Motor_User_SetDirection(p_motor, MOTOR_DIRECTION_CCW) ? true : false;
	}

	return isSet;
}


static inline void SetMotorUserCmdSigned(Motor_T * p_motor, int32_t cmd)
{
	int32_t rampTarget;

	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC)
	{
		rampTarget = cmd >> 1U;
		if(p_motor->Direction == MOTOR_DIRECTION_CW) {rampTarget = 0 - rampTarget;}
	}
	else //p_motor->CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP
	{

	}

	Motor_SetRamp(p_motor, rampTarget);
}

/*
 * 	Call regularly to update cmd value
 *  Cmd Updater called 1/Millis. check direction here.
 *
 *  sign indicates along or against set direction
 *
 *	Cmd can set without checking state machine
 *
 *  input [-65536:65535]
 */
static inline void Motor_User_SetCmd(Motor_T * p_motor, int32_t cmd)
{
//	Critical_Enter();
	if(p_motor->ControlModeFlags.Speed == 0U)
	{
		SetMotorUserCmdSigned(p_motor, cmd);
	}
	else
	{
		Motor_SetRamp(p_motor, cmd);
	}
//	Critical_Exit();
}

/******************************************************************************/
/*!
 * User control modes, torque/speed/position, above foc/sixstep commutation layer
 */
/******************************************************************************/
static inline void Motor_User_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
{
	Critical_Enter();

	Motor_SetControlModeFlags(p_motor, mode);

	if(p_motor->ControlModeFlags.Update == 1U)
	{
		StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CONTROL_MODE);
		p_motor->ControlModeFlags.Update = 0U;
	}

	Critical_Exit();
}
/*
 * Start functions sets mode only
 */
static inline void Motor_User_StartVoltageMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE);
}

/*
 * Set Cmd functions sets mode and cmd value
 *
 * input [-65536:65535]
 */
static inline void Motor_User_SetVoltageCmd(Motor_T * p_motor, int32_t voltage)
{
	Motor_User_StartVoltageMode(p_motor);
	SetMotorUserCmdSigned(p_motor, voltage);
}

static inline void Motor_User_StartTorqueMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_CURRENT);
}

/*
 * Set Cmd functions sets mode and cmd value
 *
 * input [-65536:65535]
 */
static inline void Motor_User_SetTorqueCmd(Motor_T * p_motor, int32_t torque)
{
	Motor_User_StartTorqueMode(p_motor);
	SetMotorUserCmdSigned(p_motor, torque);
}


/*
 * Default speed mode is speed torque mode
 */
static inline void Motor_User_StartSpeedMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT);
}

/*
 *
 * input [0:65535]
 */
static inline void Motor_User_SetSpeedCmd(Motor_T * p_motor, uint16_t speed)
{
	Motor_User_StartSpeedMode(p_motor);
	Motor_SetRamp(p_motor, speed); /* commutation mode common */
}

/*
 *
 * input [0:65535]
 */
//static inline void Motor_User_SetCmdPosition(Motor_T * p_motor, uint16_t angle)
//{
////	Motor_SetControlModeFlags(p_motor, MOTOR_CONTROL_MODE_POSITION);
//	//	Motor_User_SetCmd(p_motor, angle);
////	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_RUN_CMD);
//}

static inline void Motor_User_StartThrottleMode(Motor_T * p_motor)
{
	Motor_User_SetControlMode(p_motor, p_motor->Parameters.ControlMode);
}

/*
 *
 * input [0:65535]
 */
static inline void Motor_User_SetThrottleCmd(Motor_T * p_motor, uint16_t throttle)
{
//	Critical_Enter();
	Motor_User_StartThrottleMode(p_motor);
	Motor_User_SetCmd(p_motor, throttle);
//	Critical_Exit();
}

/*
	Always request opposite direction current
	if directly from accelerating to decelerating, unsigned ramp cannot transition through 0 without discontinuity
	from in-direction torque skips to zero to ramped counter-direction torque

	req opposite iq, vq approach towards 0 but not past 0, no plugging

	input [0:65535]

 */
static inline void Motor_User_SetBrakeCmd(Motor_T * p_motor, uint16_t brake)
{
	//use minimal brake to compensate for PID integral, or small values will increase speed as integral dominates proportional
	int32_t torque = brake;

	if (p_motor->Speed_RPM > 60U)
	{
//			Motor_User_SetTorqueCmd(p_motor, (int32_t)0 - (int32_t)brake);
//		if (torque < 65536/10) {torque = 65536/10;}
		Motor_User_SetTorqueCmd(p_motor, 0 - torque);
	}
	else
	{
		Phase_Ground(&p_motor->Phase); //fix repeat
	}
}


static inline void Motor_User_ProcRegenCmd(Motor_T * p_motor)
{
	SetMotorUserCmdSigned(p_motor, p_motor->Speed_Frac16 / 2); /* ramped speed match provides smoother change. alternatively, VFreqMode */
}

static inline void Motor_User_SetRegenCmd(Motor_T * p_motor)
{
	Motor_User_SetVoltageCmd(p_motor, p_motor->Speed_Frac16 / 2);

//	if (p_motor->Speed_RPM > 20U)
//	{
//		Motor_User_SetCmd(p_motor, p_motor->Speed_Frac16 / 2); /* ramped speed match provides smoother change */
//	}
//	else
//	{
//		Motor_User_DisableControl(p_motor);
//	}

}

/*
 * Checks exit in fault state, will tranisition to fault state in other states
 */
static inline void Motor_User_CheckFault(Motor_T * p_motor)
{
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_FAULT);
}

/*
 * Run Calibration functions
 */
static inline void Motor_User_ActivateCalibrationHall(Motor_T * p_motor)
{
	Motor_SetCalibrationStateHall(p_motor);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationEncoder(Motor_T * p_motor)
{
	Motor_SetCalibrationStateEncoder(p_motor);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

static inline void Motor_User_ActivateCalibrationAdc(Motor_T * p_motor)
{
	Motor_SetCalibrationStateAdc(p_motor);
	StateMachine_Semisynchronous_ProcInput(&p_motor->StateMachine, MSM_INPUT_CALIBRATION);
}

/******************************************************************************/
/*
 * Nvm Param function
 */
/******************************************************************************/
static inline void Motor_User_SetILimit_Amp(Motor_T * p_motor, uint16_t limit_Amp)
{
	uint16_t limit_Frac16 = (uint32_t)limit_Amp * 65536U /  p_motor->Parameters.IRefMax_Amp;

	if(limit_Frac16 > 65535U)
	{
		limit_Frac16 = 65535U;
	}

	p_motor->Parameters.IBusLimit_Frac16 = limit_Frac16;
	p_motor->Parameters.IqLimit = limit_Frac16 / 2U;
}

/*
 * Must use speed control mode
 */
static inline void Motor_User_SetSpeedLimit(Motor_T * p_motor, uint16_t rpm)
{
	p_motor->Parameters.SpeedRefMax_RPM = rpm;
}

//// 250Amp => 4.08V => 3341 ADCU
//.IRefMax_Amp = 270U,
//.IaRefMax_ADCU = 3445U,
//.IbRefMax_ADCU = 3445U,
//.IcRefMax_ADCU = 3445U,
//.IaRefZero_ADCU = 1990U,
//.IbRefZero_ADCU = 1990U,
//.IcRefZero_ADCU = 1990U,

static inline void Motor_User_SetSensorMode(Motor_T * p_motor, Motor_SensorMode_T mode) 			{p_motor->Parameters.SensorMode = mode;}
static inline void Motor_User_SetUserControlMode(Motor_T * p_motor, Motor_ControlMode_T mode) 			{p_motor->Parameters.ControlMode = mode;}
static inline void Motor_User_SetCommutationMode(Motor_T * p_motor, Motor_CommutationMode_T mode) 	{p_motor->Parameters.CommutationMode = mode;}

static inline void Motor_User_SetIaZero_ADCU(Motor_T * p_motor, uint16_t adcu) {p_motor->Parameters.IaRefZero_ADCU = adcu;}
static inline void Motor_User_SetIbZero_ADCU(Motor_T * p_motor, uint16_t adcu) {p_motor->Parameters.IbRefZero_ADCU = adcu;}
static inline void Motor_User_SetIcZero_ADCU(Motor_T * p_motor, uint16_t adcu) {p_motor->Parameters.IcRefZero_ADCU = adcu;}
static inline void Motor_User_SetIaMax_ADCU(Motor_T * p_motor, uint16_t adcu) {p_motor->Parameters.IaRefMax_ADCU = adcu;}
static inline void Motor_User_SetIbMax_ADCU(Motor_T * p_motor, uint16_t adcu) {p_motor->Parameters.IbRefMax_ADCU = adcu;}
static inline void Motor_User_SetIcMax_ADCU(Motor_T * p_motor, uint16_t adcu) {p_motor->Parameters.IcRefMax_ADCU = adcu;}
static inline void Motor_User_SetIaRef_ADCU(Motor_T * p_motor, uint16_t zero) {p_motor->Parameters.IaRefZero_ADCU = zero;}
static inline void Motor_User_SetIbRef_ADCU(Motor_T * p_motor, uint16_t zero) {p_motor->Parameters.IbRefZero_ADCU = zero;}
static inline void Motor_User_SetIcRef_ADCU(Motor_T * p_motor, uint16_t zero) {p_motor->Parameters.IcRefZero_ADCU = zero;}


static inline void Motor_User_SetSpeedRefVoltage_Scale(Motor_T * p_motor, uint16_t vSupply, uint16_t vMotor, uint16_t speedVMotor)
{
	p_motor->Parameters.SpeedRefVoltage_RPM = speedVMotor * vSupply / vMotor;
}

static inline void Motor_User_SetSpeedVoltageRef(Motor_T * p_motor, uint16_t speedRef)
{
	p_motor->Parameters.SpeedRefVoltage_RPM = speedRef;
}

static inline void Motor_User_SetDirectionCalibration(Motor_T * p_motor, Motor_DirectionCalibration_T calibration) 	{p_motor->Parameters.DirectionCalibration = calibration;}

static inline Motor_DirectionCalibration_T Motor_User_GetDirectionCalibration(Motor_T *p_motor) 	{return p_motor->Parameters.DirectionCalibration;}

static inline void Motor_User_SetVSupplyVMotorScale(Motor_T * p_motor, uint16_t vSupply, uint16_t vMotor)
{

}

/******************************************************************************/
/*
 * RAM Variable RW
 */
/******************************************************************************/
/*
 * Conversion Function only called if called by the user. Not called regularly
 */
//static inline uint16_t Motor_User_GetBemf_Frac16(Motor_T * p_motor)	{return Linear_Voltage_CalcFractionUnsigned16(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
//static inline uint32_t Motor_User_GetBemf_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));}
//static inline uint32_t Motor_User_GetVPos_MilliV(Motor_T * p_motor)	{return Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults.VPos_ADCU);}


//static inline float Motor_User_GetHeat_DegCFloat(Motor_T *p_motor)						{return Thermistor_ConvertToDegC_Float(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
//static inline uint16_t Motor_User_GetHeat_DegCRound(Motor_T *p_motor)						{return Thermistor_ConvertToDegC_Round(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}
//static inline uint16_t Motor_User_GetHeat_DegCNDecimal(Motor_T *p_motor, uint8_t n) 		{return Thermistor_ConvertToDegC_NDecimal(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, n);}
//static inline uint16_t Motor_User_GetHeat_DegCScalar(Motor_T * p_motor, uint16_t scalar) 	{return Thermistor_ConvertToDegC_Scalar(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU, scalar);}
//static inline uint16_t Motor_User_GetHeat_Fixed32(Motor_T *p_motor) 						{return Thermistor_ConvertToDegC_Fixed32(&p_motor->Thermistor, p_motor->AnalogResults.Heat_ADCU);}

static inline Hall_Sensors_T Motor_User_GetHall(Motor_T * p_motor) 		{return Hall_GetSensors(&p_motor->Hall);}
static inline uint16_t Motor_User_GetHallRotorAngle(Motor_T * p_motor) 	{return Hall_GetRotorAngle_Degrees16(&p_motor->Hall);}
static inline uint16_t Motor_User_GetSpeed_RPM(Motor_T * p_motor) 		{return p_motor->Speed_RPM;}
static inline Motor_ErrorFlags_T Motor_User_GetErrorFlags(Motor_T * p_motor) 	{return  p_motor->ErrorFlags;}


//static inline void Motor_User_ActivatePhase(Motor_T * p_motor)
//{
//
//}

/******************************************************************************/
/*
 * N Motor Array functions
 */
/******************************************************************************/
//static inline void ProcMotorUserN(Motor_T * p_motor, uint8_t motorCount, uint32_t (*op)(Motor_T*, uint32_t), uint16_t var)
//{
//	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
//	{
//		op(&p_motor[iMotor], var);
//	}
//}


static inline void Motor_UserN_SetCmd(Motor_T * p_motor, uint8_t motorCount, int32_t cmd)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetCmd(&p_motor[iMotor], cmd);
	}
}

static inline void Motor_UserN_SetThrottleCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t throttle)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetThrottleCmd(&p_motor[iMotor], throttle);
	}
}

static inline void Motor_UserN_SetBrakeCmd(Motor_T * p_motor, uint8_t motorCount, uint16_t brake)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetBrakeCmd(&p_motor[iMotor], brake);
	}
}

static inline void Motor_UserN_SetRegenCmd(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_SetRegenCmd(&p_motor[iMotor]);
	}
}

static inline bool Motor_UserN_SetDirectionForward(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionForward(&p_motor[iMotor]) == false)
		{
			isSet = false;
		}
	}

	return isSet;
}

static inline bool Motor_UserN_SetDirectionReverse(Motor_T * p_motor, uint8_t motorCount)
{
	bool isSet = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_SetDirectionReverse(&p_motor[iMotor]) == false)
		{
			isSet = false;
		}
	}

	return isSet;
}

static inline void Motor_UserN_DisableControl(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_DisableControl(&p_motor[iMotor]);
	}
}


static inline void Motor_UserN_Ground(Motor_T * p_motor, uint8_t motorCount)
{
	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		Motor_User_Ground(&p_motor[iMotor]);
	}
}

static inline bool Motor_UserN_CheckStop(Motor_T * p_motor, uint8_t motorCount)
{
	bool isStop = true;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_GetSpeed_RPM(&p_motor[iMotor]) > 0U)
		{
			isStop = false;
			break;
		}
	}

	return isStop;
}

static inline bool Motor_UserN_CheckErrorFlags(Motor_T * p_motor, uint8_t motorCount)
{
	bool isError = false;

	for(uint8_t iMotor = 0U; iMotor < motorCount; iMotor++)
	{
		if(Motor_User_GetErrorFlags(&p_motor[iMotor]).State != 0U)
		{
			isError = true;
			break;
		}
	}

	return isError;
}

#endif
