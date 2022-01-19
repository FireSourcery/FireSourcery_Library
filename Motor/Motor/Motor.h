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
    @file 	Motor.h
    @author FireSoucery
    @brief  Per Motor State Control.
    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "Config.h"

#include "MotorAnalog.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/BEMF/BEMF.h"

#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_DeltaD.h"
#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"

#include "Math/Q/Q.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * All modules independently conform to same ID
 */
typedef enum
{
	MOTOR_SECTOR_ID_0 = 0U,
	MOTOR_SECTOR_ID_1 = 1U,
	MOTOR_SECTOR_ID_2 = 2U,
	MOTOR_SECTOR_ID_3 = 3U,
	MOTOR_SECTOR_ID_4 = 4U,
	MOTOR_SECTOR_ID_5 = 5U,
	MOTOR_SECTOR_ID_6 = 6U,
	MOTOR_SECTOR_ID_7 = 7U,
	MOTOR_SECTOR_ERROR_000 = 0U,
	MOTOR_SECTOR_ERROR_111 = 7U,

	MOTOR_PHASE_ERROR_0 = 0U,
	MOTOR_PHASE_AC = 1U,
	MOTOR_PHASE_BC = 2U,
	MOTOR_PHASE_BA = 3U,
	MOTOR_PHASE_CA = 4U,
	MOTOR_PHASE_CB = 5U,
	MOTOR_PHASE_AB = 6U,
	MOTOR_PHASE_ERROR_7 = 7U,
} Motor_SectorId_T;

/*
 * Commutation Direction
 */
typedef enum
{
	MOTOR_DIRECTION_CW = 0U,
	MOTOR_DIRECTION_CCW = 1U,
} Motor_Direction_T;

typedef enum
{
	MOTOR_FORWARD_IS_CW,
	MOTOR_FORWARD_IS_CCW,
} Motor_DirectionCalibration_T;

//typedef enum
//{
//	MOTOR_STATUS_OKAY,
//	MOTOR_STATUS_ERROR_1,
//} Motor_Status_T;

/*
 *
 */
typedef enum
{
	MOTOR_COMMUTATION_MODE_FOC,
	MOTOR_COMMUTATION_MODE_SIX_STEP,
} Motor_CommutationMode_T;

/*
 * Feedback Control Variable Mode
 */
typedef enum
{
	MOTOR_CONTROL_MODE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,
} Motor_ControlMode_T;

typedef enum
{
	MOTOR_SENSOR_MODE_OPEN_LOOP,
	MOTOR_SENSOR_MODE_HALL,
	MOTOR_SENSOR_MODE_ENCODER,
	MOTOR_SENSOR_MODE_BEMF,
} Motor_SensorMode_T;

/*
	Motor run modes
	Mode			Feedback
	Openloop		None
	Voltage			Position
	Current			Position Current
	VoltageFreq		Position 					 (Scalar)
	SpeedVoltage	Position 			Speed
	SpeedCurrent	Position Current	Speed
	sensorless..
 */

//typedef struct
//{
// uint8_t Position: 1; 	//0 -> Openloop, 1 -> position feedback encoder or bemf
// uint8_t Sensorless: 1; 	//0 -> encoder, 1 -> bemf
// uint8_t Current: 1; 		//0 -> voltage, 1-> current feedback PID loop
// uint8_t Speed: 1; 		//0 -> const voltage or current, 1 -> speed feedback
// uint8_t SpeedPID: 1; 	//0 -> speed scalar, 1-> speed feedback PID loop
//} Motor_ConfigMode_T;

/*
 * All except hall
 */
typedef enum
{
	MOTOR_ALIGN_MODE_DISABLE,
	MOTOR_ALIGN_MODE_ALIGN,
	MOTOR_ALIGN_MODE_HFI,
} Motor_AlignMode_T;

typedef enum
{
	MOTOR_BRAKE_MODE_PASSIVE,
	MOTOR_BRAKE_MODE_CONSTANT,
	MOTOR_BRAKE_MODE_PROPRTIONAL,
	MOTOR_BRAKE_MODE_SCALAR,

} Motor_BrakeMode_T;

/*
 * Calibration Substate Flag
 */
typedef enum
{
	MOTOR_CALIBRATION_STATE_DISABLE,
	MOTOR_CALIBRATION_STATE_ADC,
	MOTOR_CALIBRATION_STATE_HALL,
	MOTOR_CALIBRATION_STATE_ENCODER,
} Motor_CalibrationState_T;

/*!
	@brief Motor Parameters
	Runtime variable configuration

	load from flash
 */
typedef struct __attribute__ ((aligned (4U)))
{
    uint8_t PolePairs;

	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_ControlMode_T 		ControlMode;
	Motor_BrakeMode_T 			BrakeMode;
	Motor_AlignMode_T 			AlignMode;

	Motor_DirectionCalibration_T DirectionCalibration;

	//	uint8_t BrakeCoeffcient;
	//	uint32_t SpeedControlPeriod;
	//	qfrac16_t FocOpenLoopVq;
	//	qfrac16_t FocAlignVd;

	uint16_t OpenLoopVPwmMin;
	uint16_t OpenLoopVPwmMax;
	uint16_t OpenLoopSpeedStart;
	uint16_t OpenLoopSpeedFinal;
	uint16_t OpenLoopAccel;
	//	uint16_t OpenLoopVHzGain; //vhz scale
	//	uint16_t OpenLoopZcdTransition;

	uint16_t SpeedMax_RPM;
	uint16_t Imax_Amp;
	uint16_t IaZero_ADCU;
	uint16_t IbZero_ADCU;
	uint16_t IcZero_ADCU;
	uint16_t IaRefMax_ADCU;
	uint16_t IbRefMax_ADCU;
	uint16_t IcRefMax_ADCU;

	uint16_t IBusLimit_Frac16;
	qfrac16_t IqLimit;

//	bool CalibrateHallOnStart;
//	uint32_t RampAcceleration; //fraction16

	//todo account for copy to ram twice
	Phase_Mode_T				PhasePwmMode;
}
Motor_Params_T;

/*
	@brief Motor Init
	Compile time const configuration
 */
typedef const struct Motor_Init_Tag
{
 	const Motor_Params_T * const P_PARAMETERS; //eeprom copy

//#ifdef CONFIG
// 	const Motor_Params_T * const P_PARAMS_DEFAULT;
//#endif

	//const using fixed resistor values
	const Linear_T UNIT_V_POS;
	const Linear_T UNIT_V_ABC; 	//Bemf V and mV conversion todo may need zero calibration

	AnalogN_T * const P_ANALOG_N;
//	const AnalogN_Conversion_T CONVERSION_VPOS_PWM_ON;
	const AnalogN_AdcFlags_T ADCS_ACTIVE_PWM_THREAD;
	const AnalogN_Conversion_T CONVERSION_VPOS;
	const AnalogN_Conversion_T CONVERSION_VA;
	const AnalogN_Conversion_T CONVERSION_VB;
	const AnalogN_Conversion_T CONVERSION_VC;
	const AnalogN_Conversion_T CONVERSION_IA;
	const AnalogN_Conversion_T CONVERSION_IB;
	const AnalogN_Conversion_T CONVERSION_IC;
	const AnalogN_Conversion_T CONVERSION_HEAT;
	const AnalogN_Conversion_T CONVERSION_OPTION_PWM_ON;
	const AnalogN_Conversion_T CONVERSION_OPTION_RESTORE;

	const uint16_t I_MAX_AMP;

//	const AnalogN_Conversion_T CONVERSION_BEMF_A_PWM_OFF;
//	const AnalogN_Conversion_T CONVERSION_BEMF_B_PWM_OFF;
//	const AnalogN_Conversion_T CONVERSION_BEMF_C_PWM_OFF;

//const AnalogN_Conversion_T CONVERSION_PHASE_AC_PWM_OFF;
//const AnalogN_Conversion_T CONVERSION_PHASE_AC_PWM_ON;
//const AnalogN_Conversion_T CONVERSION_PHASE_BC_PWM_OFF;
//const AnalogN_Conversion_T CONVERSION_PHASE_BC_PWM_ON;
//const AnalogN_Conversion_T CONVERSION_PHASE_BA_PWM_OFF;
//const AnalogN_Conversion_T CONVERSION_PHASE_CA_PWM_ON;
//const AnalogN_Conversion_T CONVERSION_PHASE_CA_PWM_OFF;
//const AnalogN_Conversion_T CONVERSION_PHASE_CB_PWM_OFF;
//const AnalogN_Conversion_T CONVERSION_PHASE_CB_PWM_ON;
//const AnalogN_Conversion_T CONVERSION_PHASE_AB_PWM_OFF;
//const AnalogN_Conversion_T CONVERSION_PHASE_AB_PWM_ON;

//	const AnalogN_Conversion_T CONVERSION_BEMF_REMAINDER;
//	const AnalogN_Conversion_T CONVERSION_FOC_IABC;
//	const AnalogN_Conversion_T CONVERSION_FOC_REMAINDER;
//	const AnalogN_Conversion_T CONVERSION_IDLE;
}
Motor_Config_T;

typedef struct
{
 	const Motor_Config_T CONFIG;	// compile time const, unique per motor
 	Motor_Params_T Parameters; 		// load from eeprom
	StateMachine_T 	StateMachine;

	Encoder_T Encoder;
	Phase_T Phase;
	Hall_T Hall;
	BEMF_T Bemf;

	volatile MotorAnalog_Results_T AnalogResults;
//		AnalogN_AdcFlags_T SignalBufferBemfA;
//	// 	AnalogN_AdcFlags_T SignalBufferBemfARepeat;
//		AnalogN_AdcFlags_T SignalBufferBemfB;
//		AnalogN_AdcFlags_T SignalBufferBemfC;
//		AnalogN_AdcFlags_T SignalBufferRemainder;
//		AnalogN_AdcFlags_T SignalBufferFocIabc;
//		AnalogN_AdcFlags_T SignalBufferFocRemainder;
//		AnalogN_AdcFlags_T SignalBufferIdle;

	volatile uint32_t ControlTimerBase;	 	/* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
	Timer_T ControlTimer; 					/* State Timer, openloop, Bem */

	Timer_T MillisTimer; //  millis thread

	//not const due to adc calibration
	Linear_T UnitIa; 	//Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;

	Thermistor_T Thermistor;

	Filter_T FilterA;
	Filter_T FilterB;
	Filter_T FilterC;

	Motor_Direction_T Direction; //active spin direction

	Linear_T Ramp;
	uint32_t RampIndex;
	uint16_t RampCmd;			//SetPoint after ramp, VReq/IReq/SpeedReq

	/* Speed Feedback */
	PID_T PidSpeed;
	Timer_T SpeedTimer;			// SpeedCalc Timer
	uint16_t Speed_RPM;			// Common Feedback Variable
	uint16_t SpeedFeedback_Frac16;
	uint16_t SpeedControl; 		/* Store results, updated once per millis */

	/*
	 * Open-loop
	 */
	bool IsOpenLoop;
	Linear_T OpenLoopRamp;
	uint32_t OpenLoopRampIndex;
	uint32_t OpenLoopCommutationPeriod;
	uint16_t OpenLoopSpeed_RPM;
	uint16_t OpenLoopVPwm;

	/*
	 * FOC
	 */
	FOC_T Foc;
	PID_T PidId;
	PID_T PidIq;
	qangle16_t ElectricalAngle;

	/* interpolated angle */
	qangle16_t HallAngle;
//	qangle16_t ElectricalDeltaPrev;
	uint32_t InterpolatedAngleIndex;

	/*
	 * Six-Step
	 */
	PID_T PidIBus; //Six Step
	Motor_SectorId_T NextPhase;		 		//for 6 step openloop/sensorless
	Motor_SectorId_T CommutationPhase;	 	//for 6 step openloop/sensorless
	uint32_t IBus_Frac16;
//	uint32_t IBusPrev_Frac16;
	uint16_t VPwm; 				//Control Variable

	volatile bool IsPwmOn;
	volatile bool IOverLimitFlag;

	//Substates
	Motor_CalibrationState_T CalibrationState; /* Substate, selection for calibration */
	uint8_t CalibrationSubstateStep;

//	uint32_t JogSteps;
//	uint32_t StepCount;

	uint32_t VPos_Debug;
	uint32_t Va_Debug;
}
Motor_T;


/******************************************************************************/
/*!
	 User Input Ramp
*/
/******************************************************************************/
static inline void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->Direction = direction;

	if (direction == MOTOR_DIRECTION_CW)
	{
		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW); //only hall module stores direction
//		BEMF_SetDirection(&p_motor->Bemf, direction); //if bemf module parses rising/falling edge
	}
	else
	{
		Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);
//		BEMF_SetDirection(&p_motor->Bemf, direction);
	}
}

static inline void Motor_ProcRamp(Motor_T * p_motor)
{
	if (p_motor->RampCmd != Linear_Ramp_GetFinal(&p_motor->Ramp))
	{
		p_motor->RampCmd = Linear_Ramp_Proc(&p_motor->Ramp, &p_motor->RampIndex);
	}
}

/*
 * userCmd is final value, fractional value of control mode unit
 */
static inline void Motor_SetRamp(Motor_T * p_motor, uint16_t userCmd)
{
	//	if (userCmd != p_motor->RampCmd)
//	int32_t acceleration = (userCmd >= p_motor->RampCmd) ? p_motor->Parameters.RampAcceleration : -p_motor->Parameters.RampAcceleration; //todo accel param
//	int32_t acceleration = userCmd - p_motor->RampCmd;
	Linear_Ramp_InitMillis(&p_motor->Ramp, 20000U, p_motor->RampCmd, userCmd, 10U);
//	Linear_Ramp_SetSlope(&p_motor->Ramp, 20000U, p_motor->RampCmd, userCmd, acceleration); //accelparam
	//	Linear_Ramp_SetSlope(&p_motor->Ramp, 20000U, p_motor->RampCmd, userCmd, userCmd - p_motor->RampCmd);
	p_motor->RampIndex = 0;
}

/*
 * userCmd is acceleration deceleration
 */
static inline void Motor_SetRamp_Rate(Motor_T * p_motor, uint16_t userCmdFinal, int16_t userCmdRate_Signed)
{
//	if (0U != p_motor->RampCmd)
	{
		//Decel by Brake per 1s
		Linear_Ramp_InitSlope(&p_motor->Ramp, 20000U, p_motor->RampCmd, userCmdFinal, (int32_t)userCmdRate_Signed/2);
//		Linear_Ramp_SetSlope(&p_motor->Ramp, 20000U, p_motor->RampCmd, 0U, -100);
		p_motor->RampIndex = 0;
	}
}

/******************************************************************************/
/*!
*/
/******************************************************************************/


/******************************************************************************/
/*!
	Speed
*/
/******************************************************************************/
static inline bool Motor_PollCaptureSpeed(Motor_T * p_motor)
{
	bool captureSpeed = Timer_Poll(&p_motor->SpeedTimer);

	if (captureSpeed == true)
	{
		p_motor->Speed_RPM = (Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder) + p_motor->Speed_RPM) / 2U;
		p_motor->SpeedFeedback_Frac16 = ((uint32_t)p_motor->Speed_RPM * (uint32_t)65535U / (uint32_t)p_motor->Parameters.SpeedMax_RPM);
	}

	return captureSpeed;
}

static inline void Motor_PollDeltaTStop(Motor_T * p_motor)
{
	if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) //once per millis
	{
		p_motor->Speed_RPM = 0U;
		p_motor->SpeedFeedback_Frac16 = 0U;
	}
}

static inline uint32_t Motor_GetSpeed(Motor_T * p_motor)
{
	return p_motor->Speed_RPM;
}

/*
 * Common Speed PID Feedback Loop
 */
static inline void Motor_ProcSpeedFeedback(Motor_T * p_motor)
{
	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
		p_motor->SpeedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, p_motor->SpeedFeedback_Frac16);
	}
}

static inline bool Motor_PollSpeedFeedback(Motor_T * p_motor)
{
	bool captureSpeed = Motor_PollCaptureSpeed(p_motor);

	if (captureSpeed == true)
	{
 		Motor_ProcSpeedFeedback(p_motor);
	}

	return captureSpeed;
}

static inline bool Motor_ResumeSpeedFeedback(Motor_T * p_motor)
{
	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
		PID_SetIntegral(&p_motor->PidSpeed, p_motor->SpeedFeedback_Frac16);
		p_motor->SpeedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, p_motor->SpeedFeedback_Frac16);
		Timer_Restart(&p_motor->SpeedTimer);
	}
}
/******************************************************************************/
/*!
*/
/******************************************************************************/

/******************************************************************************/
/*!
	Idle Stop State
*/
/******************************************************************************/
static inline void Motor_StartIdle(Motor_T * p_motor)
{
	Timer_SetPeriod(&p_motor->ControlTimer, 2000U);
}

//PollIdleConversion
static inline void Motor_ProcIdle(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VPOS);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_HEAT);
		AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
	}

	Debug_CaptureElapsed(1);

	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU); //debug check conversion

	p_motor->VPos_Debug = Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_POS, p_motor->AnalogResults.VPos_ADCU);
	p_motor->Va_Debug = Linear_Voltage_CalcMilliV(&p_motor->CONFIG.UNIT_V_ABC, p_motor->AnalogResults.Va_ADCU);

}

/******************************************************************************/
/*!
	Wrappers
*/
/******************************************************************************/
static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

static inline void Motor_Stop(Motor_T * p_motor)
{
	Motor_Float(p_motor);
	p_motor->ControlTimerBase = 0U; //ok to reset timer
}



/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);

/*
 * Referenced by State Machine
 */
extern void Motor_SetCalibrationStateAdc(Motor_T * p_motor);
extern void Motor_SetCalibrationStateHall(Motor_T * p_motor);
extern void Motor_SetCalibrationStateEncoder(Motor_T * p_motor);
extern bool Motor_CalibrateAdc(Motor_T * p_motor);
extern bool Motor_CalibrateHall(Motor_T * p_motor);
extern bool Motor_CalibrateEncoder(Motor_T * p_motor);

#endif
