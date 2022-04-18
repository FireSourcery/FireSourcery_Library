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
#include "Transducer/SinCos/SinCos.h"
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

#define MOTOR_LIB_VERSION_OPT		0
#define MOTOR_LIB_VERSION_MAJOR 	0
#define MOTOR_LIB_VERSION_MINOR 	1
#define MOTOR_LIB_VERSION_BUGFIX 	0

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
 *
 */
typedef enum
{
	MOTOR_COMMUTATION_MODE_FOC,
	MOTOR_COMMUTATION_MODE_SIX_STEP,
} Motor_CommutationMode_T;

typedef enum
{
	MOTOR_SENSOR_MODE_OPEN_LOOP,
	MOTOR_SENSOR_MODE_HALL,
	MOTOR_SENSOR_MODE_ENCODER,
	MOTOR_SENSOR_MODE_SIN_COS,
	MOTOR_SENSOR_MODE_SENSORLESS,
} Motor_SensorMode_T;

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
}
Motor_ControlMode_T; //Motor_ControlFeedbackMode_T FeedbackMode


typedef union
{
	struct
	{
		uint32_t OpenLoop 		:1; //0 -> position feedback, 1 -> Openloop, control mode live toggle
		uint32_t Speed 			:1;	//0 -> const voltage or current, 1 -> speed feedback
		uint32_t Current 		:1;	//0 -> voltage, 1-> current
		uint32_t VFreqScalar 	:1; //Use v scalar

		uint32_t Hold			:1;
//		uint32_t Regen 			:1;
		uint32_t Update 		:1;
//		uint32_t Direction :1;
	};
	uint32_t State;
}
Motor_ControlModeFlags_T; //control state

typedef enum
{
	MOTOR_ALIGN_MODE_DISABLE,
	MOTOR_ALIGN_MODE_ALIGN,
	MOTOR_ALIGN_MODE_HFI,
} Motor_AlignMode_T;

//typedef enum
//{
//	MOTOR_BRAKE_MODE_PASSIVE,
//	MOTOR_BRAKE_MODE_CONSTANT,
//	MOTOR_BRAKE_MODE_PROPRTIONAL,
//	MOTOR_BRAKE_MODE_SCALAR,
//} Motor_BrakeMode_T;

/*
 *  Direction Run Substate
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

/*
 * Fault/Run Substate
 */
typedef enum
{
	MOTOR_STATUS_OKAY,
	MOTOR_STATUS_ERROR_1,
} Motor_Status_T;

typedef union
{
	struct
	{
		uint32_t OverHeat :1;
	};
	uint32_t State;
} Motor_ErrorFlags_T;

typedef union
{
	struct
	{
		uint32_t IOverLimit :1;
	};
	uint32_t State;
} Motor_WarningFlags_T;

/*
 * Calibration Substate Flag
 */
typedef enum
{
	MOTOR_CALIBRATION_STATE_DISABLE,
	MOTOR_CALIBRATION_STATE_ADC,
	MOTOR_CALIBRATION_STATE_HALL,
	MOTOR_CALIBRATION_STATE_ENCODER,
	MOTOR_CALIBRATION_STATE_SIN_COS,
} Motor_CalibrationState_T;

/*!
	@brief Motor Parameters
	Runtime variable configuration

	load from flash
 */
typedef struct __attribute__ ((aligned (4U)))
{
	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_ControlMode_T 		ControlMode; /* User ControlMode, effective for throttle only */
//	Motor_BrakeMode_T 			BrakeMode;
	Motor_AlignMode_T 			AlignMode;

	Motor_DirectionCalibration_T DirectionCalibration;
	uint16_t AlignVoltage_Frac16;
	uint16_t AlignTime_ControlCycles;
	//	uint8_t BrakeCoeffcient;
	//	uint32_t RampAcceleration;

	uint16_t OpenLoopVPwmMin;
	uint16_t OpenLoopVPwmMax;
	uint16_t OpenLoopSpeedStart;
	uint16_t OpenLoopSpeedFinal;
	uint16_t OpenLoopAccel;
	//	uint16_t OpenLoopVHzGain; //vhz scale
	//	uint16_t OpenLoopZcdTransition;

	uint16_t SpeedRefMax_RPM;
	uint16_t SpeedRefVoltage_RPM;

	uint16_t IRefMax_Amp;
	uint16_t IaRefZero_ADCU;
	uint16_t IbRefZero_ADCU;
	uint16_t IcRefZero_ADCU;
	uint16_t IaRefMax_ADCU;
	uint16_t IbRefMax_ADCU;
	uint16_t IcRefMax_ADCU;

	uint16_t IBusLimit_Frac16; 	/* Six-Step   */
	qfrac16_t IqLimit;			/* FOC   */

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
 	const Motor_Params_T * const P_PARAMS_NVM;

	const Linear_T UNIT_V_ABC; 	//Bemf V and mV conversion //const using fixed resistor values

	AnalogN_T * const P_ANALOG_N;
	const AnalogN_AdcFlags_T ADCS_ACTIVE_PWM_THREAD; //rename group adc active
	const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
//	const AnalogN_Conversion_T CONVERSION_OPTION_PWM_ON;
//	const AnalogN_Conversion_T CONVERSION_OPTION_RESTORE;
}
Motor_Config_T;

typedef struct
{
 	const Motor_Config_T CONFIG;	// compile time const, unique per motor
 	Motor_Params_T Parameters; 		// load from eeprom

	Phase_T Phase;
	Encoder_T Encoder;
	Hall_T Hall;
	SinCos_T SinCos;
	Thermistor_T Thermistor;

	/*
	 * State and SubStates
	 */
 	StateMachine_T StateMachine;

 	/* Run State */
	Motor_Direction_T Direction; 		/* Active spin direction */
	Motor_Direction_T UserDirection; 	/* Passed to StateMachine */
//	bool Brake; //can change to quadrant to include plugging
	Motor_ControlModeFlags_T ControlModeFlags;

	Motor_ErrorFlags_T ErrorFlags;
	Motor_WarningFlags_T WarningFlags;

	Motor_CalibrationState_T CalibrationState; /* Substate, selection for calibration */
	uint8_t CalibrationStateStep;
	bool IsPwmOn;
	bool IOverLimitFlag;

	volatile MotorAnalog_Results_T AnalogResults;

	uint32_t ControlTimerBase;	 	/* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
	Timer_T ControlTimer; 			/* State Timer, openloop, Bem */
	Timer_T MillisTimer; 			//  millis thread

	Timer_T SecondsTimer; 			//  Heat thread

	//not const due to adc calibration
	Linear_T UnitIa; 	//Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;

	//Calibration use
	Filter_T FilterA;
	Filter_T FilterB;
	Filter_T FilterC;

	Linear_T Ramp;
//	Linear_T RampUp;
//	Linear_T RampDown;
	uint32_t RampIndex;
	int32_t RampCmd;			//SetPoint after ramp, VReq/IReq/SpeedReq

	/* Speed Feedback */
	PID_T PidSpeed;
	Timer_T SpeedTimer;			// SpeedCalc Timer
	uint16_t Speed_RPM;			// Common Feedback Variable
	uint32_t Speed_Frac16; 		/* Can over saturate */
	int32_t SpeedControl; 		/* Speed PID output, (Speed_Frac16 - RampCmd) => (VPwm, Vq, Iq), updated once per millis */


	uint16_t Speed2_RPM;
	uint32_t Speed2_Frac16;

	/*
	 * Open-loop
	 */
	bool IsOpenLoop;
	Linear_T OpenLoopRamp;
	uint32_t OpenLoopRampIndex;
	uint32_t OpenLoopCommutationPeriod;
	uint16_t OpenLoopSpeed_RPM;
	uint16_t OpenLoopVPwm;


	uint16_t VBemfPeak_ADCU;
	uint16_t VBemfPeakTemp_ADCU;
	uint16_t VFreqScalar;
	/*
	 * FOC
	 */
	FOC_T Foc;
	PID_T PidId;
	PID_T PidIq;
	qangle16_t ElectricalAngle;
	qangle16_t ElectricalAnglePrev;
	uint32_t DeltaAngle;
	/* interpolated angle */
	qangle16_t HallAngle;
	uint32_t InterpolatedAngleIndex;

	/*
	 * Six-Step
	 */
	PID_T PidIBus; //Six Step
//	BEMF_T Bemf;
	Motor_SectorId_T NextPhase;		 		//for 6 step openloop/sensorless
	Motor_SectorId_T CommutationPhase;	 	//for 6 step openloop/sensorless
	uint32_t CommutationTimeRef;
	uint32_t IBus_Frac16;
	uint32_t IBusSum_Frac16;
//	uint32_t IBusSum_ADCU;
//	uint32_t IBusPrev_Frac16;
	uint16_t VPwm; 				//Control Variable

	uint32_t MicrosRef; //debug
	volatile uint32_t DebugTime[10];
	volatile uint32_t HallDebug[13];
//	uint32_t JogSteps;
//	uint32_t StepCount;
}
Motor_T;

/******************************************************************************/
/*!
 *	Motor Common
 * 	@{
 */
/******************************************************************************/

/******************************************************************************/
/*
	Direction
*/
/******************************************************************************/
/*
 * CCW or CW
 */
static inline void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->Direction = direction;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			break;

		case MOTOR_SENSOR_MODE_HALL:
			if(direction == MOTOR_DIRECTION_CW)	{Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW);}
			else								{Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW);}
			break;

		default :
			break;
	}

}

/*
 * Forward/Reverse use calibration param
 */
static inline void Motor_SetDirectionForward(Motor_T * p_motor)
{
	if (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)	{Motor_SetDirection(p_motor, MOTOR_DIRECTION_CCW);}
	else																	{Motor_SetDirection(p_motor, MOTOR_DIRECTION_CW);}
}

static inline void Motor_SetDirectionReverse(Motor_T * p_motor)
{
	if (p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW)	{Motor_SetDirection(p_motor, MOTOR_DIRECTION_CW);}
	else																	{Motor_SetDirection(p_motor, MOTOR_DIRECTION_CCW);}
}


/******************************************************************************/
/*
	Ramp
*/
/******************************************************************************/
/*
 * Proc 20000Hz
 */
static inline void Motor_ProcRamp(Motor_T * p_motor)
{
	//index mode check negative
//	p_motor->RampCmd = Linear_Ramp_ProcIndexOutput(&p_motor->Ramp, &p_motor->RampIndex, p_motor->RampCmd);
//	p_motor->RampCmd = Linear_Ramp_GetTarget(&p_motor->Ramp); //disables ramp

	p_motor->RampCmd = Linear_Ramp_CalcNextOutput(&p_motor->Ramp, p_motor->RampCmd);
}

/*
 * Set 1000Hz
 */
static inline void Motor_SetRamp(Motor_T * p_motor, int32_t userCmd)
{
	/*
	 * Constant acceleration ramp
	 */
	Linear_Ramp_SetTarget(&p_motor->Ramp, userCmd);
}

static inline void Motor_SetRampInterpolate(Motor_T * p_motor, int32_t userCmd)
{
	/*
	 * dynamically generated Ramp,
	 * effectively divide input over control period intervals, when using 1ms period
	 */
//	Linear_Ramp_SetSlopeMillis(&p_motor->Ramp, 1U, 20000U, p_motor->RampCmd, userCmd);
}

static inline void Motor_ResetRamp(Motor_T * p_motor)
{
	p_motor->RampCmd = 0U;
//	p_motor->RampIndex = 0U;
	Linear_Ramp_SetTarget(&p_motor->Ramp, 0);
}

/*
 * match ramp output
 */
static inline void Motor_ResumeRampOutput(Motor_T * p_motor, int32_t matchOutput)
{
	p_motor->RampCmd = matchOutput;
//	Linear_Ramp_SetIndex(&p_motor->Ramp, &p_motor->RampIndex, matchOutput);
	Linear_Ramp_SetTarget(&p_motor->Ramp, matchOutput);
}


/******************************************************************************/
/*
	Speed
*/
/******************************************************************************/
static inline void Motor_CaptureEncoderSpeed(Motor_T * p_motor)
{
	p_motor->Speed_RPM =  (Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder));// + p_motor->Speed_RPM) / 2U;
	p_motor->Speed_Frac16 = ((uint32_t)p_motor->Speed_RPM * (uint32_t)65535U / (uint32_t)p_motor->Parameters.SpeedRefMax_RPM);
}

static inline void Motor_CaptureSpeed(Motor_T * p_motor)
{
	uint32_t deltaAngle;
	uint16_t angle = (uint16_t)p_motor->ElectricalAngle;
	uint16_t anglePrev = (uint16_t)p_motor->ElectricalAnglePrev;

//	if (angle < anglePrev)
//	{
//		deltaAngle = (uint32_t)65535U - anglePrev + angle + 1U;
//	}
//	else /* normal case */
	{
		deltaAngle = angle - anglePrev;
	}

	p_motor->DeltaAngle = deltaAngle;
	p_motor->ElectricalAnglePrev = angle;

	p_motor->Speed2_RPM = Encoder_ConvertAngleToRotationalSpeed_RPM(0, deltaAngle, 1000U) / p_motor->Encoder.Params.MotorPolePairs; //todo move to linear module
	p_motor->Speed2_Frac16 = deltaAngle*1000U*60U/((uint32_t)p_motor->Parameters.SpeedRefMax_RPM * p_motor->Encoder.Params.MotorPolePairs);

}

//Speed pid always uses directionless positive value [0:65535]
static inline void Motor_ResumeSpeedOutput(Motor_T * p_motor, int32_t matchOutput)
{
	Motor_ResumeRampOutput(p_motor, p_motor->Speed_Frac16); // req start from present speed
	PID_SetIntegral(&p_motor->PidSpeed, matchOutput);
	p_motor->SpeedControl = matchOutput; // output SpeedControl is V or I
}


static inline void Motor_PollDeltaTStop(Motor_T * p_motor)
{
	if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) //once per millis
	{
		p_motor->Speed_RPM = 0U;
		p_motor->Speed_Frac16 = 0U;
	}
}


/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);
extern bool Motor_CheckControlMode(Motor_T * p_motor, Motor_ControlMode_T mode);
extern void Motor_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode);

#endif
