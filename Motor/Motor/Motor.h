/******************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSourcery / The Firebrand Forge Inc

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
	@author FireSourcery
	@brief  Per Motor State Control.
	@version V0
*/
/******************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "Config.h"

#include "Global_Motor.h"
#include "MotorAnalog.h"
#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/SinCos/SinCos.h"
#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_ModeDT.h"
#include "Transducer/Thermistor/Thermistor.h"

#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Timer/Timer.h"

#include "Math/Q/Q.h"
#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear_Ramp.h"
// #include "Math/Linear/Linear_Speed.h"
#include "Math/Linear/Linear.h"
#include "Math/PID/PID.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_LIB_VERSION_OPT		0U
#define MOTOR_LIB_VERSION_MAJOR 	0U
#define MOTOR_LIB_VERSION_MINOR 	1U
#define MOTOR_LIB_VERSION_BUGFIX 	0U

typedef enum Motor_CommutationMode_Tag
{
	MOTOR_COMMUTATION_MODE_FOC,
	MOTOR_COMMUTATION_MODE_SIX_STEP,
}
Motor_CommutationMode_T;

/*
	Sensor Mode Param at start up.
*/
typedef enum Motor_SensorMode_Tag
{
	MOTOR_SENSOR_MODE_HALL,
	MOTOR_SENSOR_MODE_ENCODER,
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
	MOTOR_SENSOR_MODE_SIN_COS,
#endif
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE)
	MOTOR_SENSOR_MODE_SENSORLESS,
#endif
#if defined(CONFIG_MOTOR_SENSORS_EXTERN_ENABLE)
	MOTOR_SENSOR_MODE_EXTERN,
#endif
}
Motor_SensorMode_T;

typedef enum Motor_AlignMode_Tag
{
	MOTOR_ALIGN_MODE_DISABLE,
	MOTOR_ALIGN_MODE_ALIGN,
	MOTOR_ALIGN_MODE_HFI,
}
Motor_AlignMode_T;

/*
	Feedback Control Variable Mode
*/
typedef enum Motor_FeedbackMode_Tag
{
	MOTOR_FEEDBACK_MODE_OPEN_LOOP,
	MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE,
	MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT,
	MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT,
}
Motor_FeedbackMode_T;

/*
	Bit representation
*/
typedef union Motor_FeedbackModeFlags_Tag
{
	struct
	{
		uint32_t Speed 		: 1U;	/* 0 -> Voltage or Current only, 1 -> Speed feedback */
		uint32_t Current 	: 1U;	/* 0 -> Voltage, 1-> Current */
		uint32_t Scalar 	: 1U; 	/* 1-> Use Scalar  */
		uint32_t OpenLoop 	: 1U; 	/* 0 -> Position feedback, 1 -> Openloop */
		uint32_t Update 	: 1U;	/* Control Mode Update, shared */
	};
	uint32_t State;
}
Motor_FeedbackModeFlags_T;

/*
	Effectively sync mailbox for async calculations
*/
typedef union Motor_ControlFlags_Tag
{
	struct
	{
		uint32_t SensorFeedback				: 1U;
		uint32_t HeatWarning 				: 1U;
		uint32_t VoltageModeILimitActive 	: 1U;
		// uint32_t FieldWeakening 			: 1U;
	};
	uint32_t State;
}
Motor_ControlFlags_T;

typedef union
{
	struct
	{
		uint32_t HeatShutdown 		: 1U;
		uint32_t AlignStartUp 		: 1U;
	};
	uint32_t State;
}
Motor_FaultFlags_T;

// typedef enum Motor_SpeedLimitActiveId_Tag
// {
// 	MOTOR_SPEED_LIMIT_ACTIVE_DISABLE = 0U,
// 	MOTOR_SPEED_LIMIT_ACTIVE_SYSTEM = 1U, /* From upper module */
// 	MOTOR_SPEED_LIMIT_ACTIVE_USER = 2U,
// }
// Motor_SpeedLimitActiveId_T;

typedef enum Motor_ILimitActiveId_Tag
{
	MOTOR_I_LIMIT_ACTIVE_DISABLE = 0U,
	MOTOR_I_LIMIT_ACTIVE_HEAT = 1U, 	/* MotorHeat */
	MOTOR_I_LIMIT_ACTIVE_SYSTEM = 20U,  /* From upper module */
	MOTOR_I_LIMIT_ACTIVE_USER = 30U,
}
Motor_ILimitActiveId_T;

/*
	Direction Run SubState
*/
typedef enum Motor_Direction_Tag
{
	MOTOR_DIRECTION_CW,
	MOTOR_DIRECTION_CCW,
}
Motor_Direction_T;

typedef enum Motor_DirectionCalibration_Tag
{
	MOTOR_FORWARD_IS_CW,
	MOTOR_FORWARD_IS_CCW,
}
Motor_DirectionCalibration_T;

/*
	Align SubState
*/
typedef enum Motor_AlignState_Tag
{
	MOTOR_ALIGN_STATE_ALIGN,
	MOTOR_ALIGN_STATE_START_UP,
	MOTOR_ALIGN_STATE_CHECK_FAULT,
}
Motor_AlignState_T;

/*
	Calibration SubState
*/
typedef enum Motor_CalibrationState_Tag
{
	MOTOR_CALIBRATION_STATE_DISABLE,
	MOTOR_CALIBRATION_STATE_ADC,
	MOTOR_CALIBRATION_STATE_HALL,
	MOTOR_CALIBRATION_STATE_ENCODER,
	MOTOR_CALIBRATION_STATE_SIN_COS,
	MOTOR_CALIBRATION_STATE_POSITION_SENSOR,
}
Motor_CalibrationState_T;

/*
	All modules independently conform to same ID
*/
typedef enum Motor_SectorId_Tag
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

	MOTOR_PHASE_ERROR_0 = PHASE_ID_0,
	MOTOR_PHASE_AC = PHASE_ID_1_AC,
	MOTOR_PHASE_BC = PHASE_ID_2_BC,
	MOTOR_PHASE_BA = PHASE_ID_3_BA,
	MOTOR_PHASE_CA = PHASE_ID_4_CA,
	MOTOR_PHASE_CB = PHASE_ID_5_CB,
	MOTOR_PHASE_AB = PHASE_ID_6_AB,
	MOTOR_PHASE_ERROR_7 = PHASE_ID_7,
}
Motor_SectorId_T;

/******************************************************************************/
/*!
	Frac16 [-1:1] <=> [-65536:65536] in q16.16 may over saturate
	FracU16 [0:1] <=> [0:65535] in q0.16
	FracS16 [-1:1] <=> [-32768:32767] in q1.15
*/
/******************************************************************************/
/*!
	@brief Motor Parameters - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct __attribute__((aligned(2U))) Motor_Params_Tag
{
	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_AlignMode_T 			AlignMode;
	Motor_FeedbackMode_T 		DefaultFeedbackMode; 	/* Default FeedbackMode, and ThrottleCmd */
	// Motor_FeedbackMode_T 		ThrottleMode; 	/* Motor layer per Motor Implementation */
	// Motor_FeedbackMode_T 		BrakeMode; 		/* Motor layer per Motor Implementation */
	Motor_DirectionCalibration_T DirectionCalibration;

 	/*
		Ref values, known calibration parameter provide by user
	*/
	uint8_t PolePairs;
	uint16_t Kv;
	uint16_t SpeedFeedbackRef_Rpm; 	/* Feedback / PID Regulator Limits Ref, User IO units conversion, Encoder speed calc ref. */
	uint16_t SpeedVRef_Rpm; 		/* Voltage Match Ref. VF Mode, Freewheel to Run. Use < SpeedFeedbackRef_Rpm to begin at lower speed. */
	//option Scale to Kv or Bemf
#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	uint16_t IPeakRef_Adcu;
#endif
	uint16_t IaZeroRef_Adcu;
	uint16_t IbZeroRef_Adcu;
	uint16_t IcZeroRef_Adcu;

	/* "Root" Limits */
 	uint16_t SpeedLimitCcw_Frac16;		/* Persistent User Param. Frac16 of SpeedFeedbackRef_Rpm */
	uint16_t SpeedLimitCw_Frac16;
	uint16_t ILimitMotoring_Frac16;		/* Persistent User Param. Frac16 of RefMax I_UNITS_AMPS */
	uint16_t ILimitGenerating_Frac16;

	uint16_t RampAccel_Cycles;

	uint16_t AlignVPwm_Frac16;
	uint16_t AlignTime_Cycles;
	// uint16_t VoltageBrakeScalar_InvFrac16; /* [0:65535], 0 is highest intensity */
	//	uint8_t BrakeCoeffcient;

#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
	uint16_t OpenLoopSpeed_RPM; 	/* Max */
	uint16_t OpenLoopSpeed_Frac16; 	/* Max */
	uint16_t OpenLoopVPwm_Frac16; 	/* Frac16 */
	uint16_t OpenLoopAccel_Cycles;		/* Time to reach OpenLoopSpeed_RPM */
	// uint16_t OpenLoopVHzGain;
#endif

	uint16_t SurfaceDiameter;
	uint16_t GearRatio_Factor;
	uint16_t GearRatio_Divisor;

	Phase_Mode_T PhasePwmMode; 	/* Only 1 nvm param for phase module. */
}
Motor_Params_T;

/*!
	@brief Motor Config - Compile time const configuration. Unique per Motor
*/
typedef const struct Motor_Init_Tag
{
	AnalogN_T * const P_ANALOG_N;
	const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
	const Motor_Params_T * const P_PARAMS_NVM;
	void (*INIT_SENSOR_HALL)(void);
	void (*INIT_SENSOR_ENCODER)(void);
}
Motor_Config_T;

typedef struct Motor_Tag
{
	const Motor_Config_T CONFIG;
	Motor_Params_T Parameters;

	volatile MotorAnalog_Results_T AnalogResults;

	Encoder_T Encoder;
	Hall_T Hall;
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
	SinCos_T SinCos;
#endif
	Phase_T Phase;
	Thermistor_T Thermistor;

	/*
		State and Substates
	*/
	StateMachine_T StateMachine;
	uint32_t ControlTimerBase;	 	/* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
	Timer_T ControlTimer; 			/* State Timer, openloop, Bemf */
	/* Run SubState */
	Motor_Direction_T Direction; 			/* Active spin direction */
	Motor_FeedbackModeFlags_T FeedbackModeFlags;
	Motor_ControlFlags_T ControlFlags; 		/* Run SubState */
	/* Calibration SubState */
	Motor_CalibrationState_T CalibrationState; 	/* SubState, selection for calibration */
	uint8_t CalibrationStateIndex;
	Motor_FaultFlags_T FaultFlags;
	Motor_AlignState_T AlignState;

	/*
		Active Limits
	*/
	uint16_t SpeedLimitCcw_Frac16; 		/* Active SpeedLimit, Frac16 of SpeedRefMax Param */
	uint16_t SpeedLimitCw_Frac16;
	uint16_t SpeedLimit_Frac16; 		/* Active SpeedLimit, optionally reduce 1 check of direction during User_SetFeedbackCmd */
	// Motor_SpeedLimitActiveId_T SpeedLimitActiveId;
	// uint16_t SpeedLimitActiveScalar;
	uint16_t ILimitMotoring_Frac16;		/* Active ILimit */
	uint16_t ILimitGenerating_Frac16;
	Motor_ILimitActiveId_T ILimitActiveId;
	uint16_t ILimitActiveSentinel;		/* Store for comparison */
	int16_t VoltageModeILimitCcw_FracS16; /* [-32767:32767] directional input into IqPid, Same as SpeedPid output limit during SPEED_CURRENT_MODE */
	int16_t VoltageModeILimitCw_FracS16;
	// int16_t TorqueModeSpeedLimitCcw_FracS16;
	// int16_t TorqueModeSpeedLimitCw_FracS16;

	/*
		UserCmd Input => Ramp
	*/
	Linear_T Ramp;		/* User Input Ramp - Speed, Current, or Voltage. Updated without StateMachine Check */
						/* [-32767:32767] SetPoint after ramp => SpeedReq, IReq, VReq. [0:65535] VFreq Mode */
	Linear_T AuxRamp; 	/* OpenLoop and Align */

	/*
		Speed Feedback
	*/
	int32_t Speed_Frac16; 			/* [~-65535*2:~65535*2] Speed Feedback Variable. Can over saturate */
	PID_T PidSpeed;					/* Input Speed_Frac16 Q16.16, Output SpeedControl_FracS16 => VPwm, Vq, Iq */
	Timer_T SpeedTimer;				/* Speed Calc Timer */
	int16_t SpeedControl_FracS16; 	/* [~-32767:~32767] Speed Control Variable. PidSpeed(RampCmd - Speed_Frac16 / 2) => VPwm, Vq, Iq. Updated once per ms */

	/* Sensorless and SinCos. Non square wave Encoder module */
	Linear_T UnitsAngleSpeed; 			/*  */
	Linear_T UnitsSurfaceSpeed; 		/*  */
	qangle16_t SpeedAngle; 			/* Save for reference, MechanicalDelta */

	// PID_T PidPosition;
	// int16_t PositionControl;

	qangle16_t MechanicalAngle;
	qangle16_t ElectricalAngle;		/* OpenLoop, Shared E-Cycle edge detect, User output */
	uint16_t VBemfPeak_Adcu;
	uint16_t VBemfPeakTemp_Adcu;
	uint16_t IPhasePeak_Adcu;
	uint16_t IPhasePeakTemp_Adcu;

	/*
		FOC
	*/
	FOC_T Foc;
	PID_T PidIq;					/* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
	PID_T PidId;

	/*
		Six-Step
	*/
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	PID_T PidIBus;
	BEMF_T Bemf;
	Motor_SectorId_T NextPhase;
	Motor_SectorId_T CommutationPhase;
	uint32_t CommutationTimeRef;
	uint32_t IBus_Frac16;
	uint32_t IBusSum_Frac16;
	uint16_t VPwm; 	/* Six-Step Control Variable */
#endif

	/*
		OpenLoops speed ramp
	*/
#if defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE) || defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_DEBUG_ENABLE)
	Linear_T OpenLoopSpeedRamp;			/* OpenLoopSpeed Ramp */
	uint16_t OpenLoopSpeed_RPM;
	#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	uint32_t OpenLoopCommutationPeriod;
	#endif
#endif

	/* Jog */
	uint32_t JogIndex;

	/*
		Unit Conversions
	*/
	Linear_T UnitsIa; 	 	/* Frac16 and Amps */
	Linear_T UnitsIb;
	Linear_T UnitsIc;
	Linear_T UnitsVabc;		/* Bemf V,mV, and Frac16 conversion */
	Linear_T UnitsVSpeed;

	Filter_T FilterA; /* Calibration use */
	Filter_T FilterB;
	Filter_T FilterC;

#if  defined(CONFIG_MOTOR_DEBUG_ENABLE)
	uint32_t MicrosRef;
	volatile bool DebugFlag;
	volatile uint32_t DebugError;
	volatile uint32_t DebugTime[10U];
	volatile uint32_t DebugTimeABC[3U];

	// // volatile int32_t FreqD[25];
	// // volatile int32_t DeltaD[25];
	// // volatile uint32_t DeltaTh[25];
	// // volatile uint32_t Angle[25];
	// volatile int32_t Speed[25];
	// // volatile int32_t RampValue[25];
	// // volatile int32_t ErrorSum[25];
	// volatile int32_t SpeedControl[25];
	// // volatile int16_t Iq[25];
	// // volatile int16_t Id[25];


	// volatile uint32_t ControlTimerDebug[25];
	// volatile uint32_t ControlTimerDebug2[25];


	// // volatile int32_t FreqD2[25];
	// // volatile int32_t DeltaD2[25];
	// // volatile uint32_t DeltaTh2[25];
	// // volatile uint32_t Angle[25];
	// volatile int32_t Speed2[25];
	// // volatile int32_t RampValue[25];
	// // volatile int32_t ErrorSum[25];
	// volatile int32_t SpeedControl2[25];
	// volatile int16_t Vq[25];
	// volatile int16_t Vq2[25];
	// // volatile int16_t Id2[25];

	volatile uint32_t DebugCounter;
	volatile uint32_t DebugCounter2;
#endif
}
Motor_T;

#if defined(CONFIG_MOTOR_DEBUG_ENABLE)
	#include "Motor_Debug.h"
#endif

/******************************************************************************/
/*
	Simplify CommutationMode Check
*/
/******************************************************************************/
typedef void(*Motor_CommutationModeFunction_T)(Motor_T * p_motor);

static inline void Motor_ProcCommutationMode(Motor_T * p_motor, Motor_CommutationModeFunction_T focFunction, Motor_CommutationModeFunction_T sixStepFunction)
{
#if 	defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) 				{ focFunction(p_motor); }
	else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ 	{ sixStepFunction(p_motor); }
#elif 	defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
	(void)focFunction;	sixStepFunction(p_motor);
#else /* defined(CONFIG_MOTOR_FOC_ENABLE) */
	(void)sixStepFunction;	focFunction(p_motor);
#endif
}

// typedef void(*Motor_CommutationModeFunction1_T)(Motor_T * p_motor, uint32_t var);

// static inline void Motor_ProcCommutationMode1(Motor_T * p_motor, Motor_CommutationModeFunction1_T focFunction, Motor_CommutationModeFunction1_T sixStepFunction, uint32_t var)
// {
// #if 	defined(CONFIG_MOTOR_SIX_STEP_ENABLE) && defined(CONFIG_MOTOR_FOC_ENABLE)
// 	if(p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_FOC) 				{ focFunction(p_motor, var); }
// 	else /* p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP */ 	{ sixStepFunction(p_motor, var); }
// #elif 	defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
// 	(void)focFunction;	sixStepFunction(p_motor, var);
// #else /* defined(CONFIG_MOTOR_FOC_ENABLE) */
// 	(void)sixStepFunction;	focFunction(p_motor, var);
// #endif
// }

/******************************************************************************/
/*

*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_UNIT_CONVERSION_LOCAL
static inline int32_t _Motor_ConvertToSpeedFrac16(Motor_T * p_motor, int32_t speed_rpm) 	{ return speed_rpm * 65535 / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline int16_t _Motor_ConvertToSpeedRpm(Motor_T * p_motor, int32_t speed_frac16) 	{ return speed_frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t _Motor_ConvertToIFrac16(Motor_T * p_motor, int32_t i_amp) 			{ (void)p_motor; return i_amp * 65535 /  GLOBAL_MOTOR.I_UNITS_AMPS; }
static inline int16_t _Motor_ConvertToIAmp(Motor_T * p_motor, int32_t i_frac16) 			{ (void)p_motor; return i_frac16 *  GLOBAL_MOTOR.I_UNITS_AMPS / 65536; }
static inline uint32_t _Motor_ConvertToMillis(Motor_T * p_motor, int32_t controlCycles) 	{ (void)p_motor; return controlCycles * 1000 / GLOBAL_MOTOR.CONTROL_FREQ; }
static inline uint32_t _Motor_ConvertToControlCycles(Motor_T * p_motor, int32_t millis) 	{ (void)p_motor; return millis * GLOBAL_MOTOR.CONTROL_FREQ / 1000; }
// static inline uint32_t speed_angle16torpm(uint16_t angle16, uint32_t sampleFreq) { return  (angle16 * sampleFreq >> 16U) * 60U; }
// static inline uint32_t speed_rpmtoangle16(uint16_t rpm, uint32_t sampleFreq) { return (rpm << 16U) / (60U * sampleFreq); }
static inline uint32_t _Motor_ConvertAngleToRpm(uint16_t angle16, uint32_t sampleFreq) 		{ return (angle16 * sampleFreq >> 16U) * 60U; }
static inline uint32_t _Motor_ConvertRpmToAngle(uint16_t rpm, uint32_t sampleFreq) 			{ return (rpm << 16U) / (60U * sampleFreq); }
#endif

/******************************************************************************/
/*
	Interrupts
*/
/******************************************************************************/
static inline void Motor_ClearPwmInterrupt(Motor_T * p_motor) 	{ Phase_ClearInterrupt(&p_motor->Phase); }
static inline void Motor_DisablePwm(Motor_T * p_motor) 			{ Phase_DisableInterrupt(&p_motor->Phase); }
static inline void Motor_EnablePwm(Motor_T * p_motor) 			{ Phase_EnableInterrupt(&p_motor->Phase); }

/******************************************************************************/
/*
	Speed
*/
/******************************************************************************/
/*
	Speed Feedback Loop
	SpeedControl_FracS16 update ~1000Hz, Ramp input 1000Hz, RampCmd output 20000Hz
	input	RampCmd[-32767:32767] - (speedFeedback_Frac16 / 2)[-32767:32767]
			accepts over saturated inputs
	output 	SpeedControl_FracS16[-32767:32767] => IqReq or VqReq
*/
static inline void Motor_ProcSpeedFeedback(Motor_T * p_motor, int32_t speedFeedback_FracS16)
{
	if(p_motor->FeedbackModeFlags.Speed == 1U) { p_motor->SpeedControl_FracS16 = PID_Calc(&p_motor->PidSpeed, Linear_Ramp_GetOutput(&p_motor->Ramp), speedFeedback_FracS16); };
}

static inline void Motor_SetSpeedOutput(Motor_T * p_motor, int32_t speedControlMatch_FracS16)
{
	p_motor->SpeedControl_FracS16 = speedControlMatch_FracS16; /* SpeedControl_FracS16 may be V or I */
	PID_SetIntegral(&p_motor->PidSpeed, speedControlMatch_FracS16);
}

static inline bool Motor_CheckSpeed(Motor_T * p_motor)
{
	return (p_motor->Speed_Frac16 < 65536);	/* Disable release from fw. Only check forward direction */
}

/* VScalar Ramp direction? */
static inline bool Motor_CheckAlignStartUp(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_ENCODER:	if((p_motor->Speed_Frac16 ^ Linear_Ramp_GetTarget(&p_motor->Ramp)) < 0) { p_motor->FaultFlags.AlignStartUp = 1U; } break;
		default: break;
	}

	return p_motor->FaultFlags.AlignStartUp;
}


/*
	Match to Bemf
	Captured VBemfPeak always positive
*/
static inline uint16_t Motor_GetVSpeedFrac16_VBemf(Motor_T * p_motor)
{
	return Linear_Voltage_CalcFracU16(&p_motor->UnitsVabc, p_motor->VBemfPeak_Adcu);
}

/*
	Match to Speed
	VSpeed = Speed_Frac16 * SpeedVRef_Rpm / SpeedFeedbackRef_Rpm
	User sets lower SpeedVRef_Rpm to ensure not match to higher speed
	Output must be saturated. Calling function /2 does not clear over saturation
*/
static inline uint16_t Motor_GetVSpeedFrac16_Speed(Motor_T * p_motor)
{
	return Linear_Function_FracU16(&p_motor->UnitsVSpeed, p_motor->Speed_Frac16);
}


/******************************************************************************/
/*
	OpenLoop Common
*/
/******************************************************************************/
static inline bool Motor_CheckPositionFeedback(Motor_T * p_motor)
{
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
	return (p_motor->ControlFlags.SensorFeedback == 1U);
#else
	(void)p_motor; return true;
#endif
}

// static inline bool Motor_CheckOpenLoop(Motor_T * p_motor) { return !Motor_CheckPositionFeedback(p_motor); }

static inline bool Motor_SetPositionFeedback(Motor_T * p_motor, bool isTrue)
{
#if defined(CONFIG_MOTOR_SENSORS_SENSORLESS_ENABLE) || defined(CONFIG_MOTOR_OPEN_LOOP_ENABLE)  || defined(CONFIG_MOTOR_DEBUG_ENABLE)
	(p_motor->ControlFlags.SensorFeedback = isTrue);
#else
	(void)p_motor; (void)isTrue;
#endif
}

/* todo */
extern bool Motor_CheckSensorFeedback(Motor_T * p_motor);

/******************************************************************************/
/*
	Feedback Mode
*/
/******************************************************************************/
static inline Motor_FeedbackModeFlags_T Motor_ConvertFeedbackModeFlags(Motor_FeedbackMode_T mode)
{
	static const Motor_FeedbackModeFlags_T MODE_OPEN_LOOP 		= { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, .Scalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_VOLTAGE 		= { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .Scalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_VOLTAGE_FREQ 	= { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .Scalar = 1U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_CURRENT 		= { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, .Scalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_SPEED_VOLTAGE 	= { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, .Scalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_SPEED_CURRENT 	= { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, .Scalar = 0U, .Update = 0U, };

	Motor_FeedbackModeFlags_T flags;

	switch(mode)
	{
		case MOTOR_FEEDBACK_MODE_OPEN_LOOP:					flags.State = MODE_OPEN_LOOP.State; 	break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE:			flags.State = MODE_VOLTAGE.State;		break;
		case MOTOR_FEEDBACK_MODE_SCALAR_VOLTAGE_FREQ:		flags.State = MODE_VOLTAGE_FREQ.State;	break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT:			flags.State = MODE_CURRENT.State;		break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE:	flags.State = MODE_SPEED_VOLTAGE.State;	break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT:	flags.State = MODE_SPEED_CURRENT.State;	break;
		default: flags.State = 0; break;
	}

	return flags;
}

/*!
	check feedback mode change
	@return
*/
static inline bool Motor_CheckFeedbackModeFlags(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	return (p_motor->FeedbackModeFlags.State == Motor_ConvertFeedbackModeFlags(mode).State);
}

/*
	Sets flags only
	Motor_User_SetControlMode() apply flags to run state
*/
static inline void Motor_SetFeedbackModeFlags(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	p_motor->FeedbackModeFlags.State = Motor_ConvertFeedbackModeFlags(mode).State;
}


/* todo fix flags */
static inline bool Motor_CheckControlMode(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	// return (((Motor_CheckFeedbackModeFlags(p_motor, mode) == false) && (Motor_CheckPositionFeedback(p_motor) == true)) || p_motor->FeedbackModeFlags.Update);
	return (Motor_CheckFeedbackModeFlags(p_motor, mode) == false);
}

static inline void Motor_SetControlFlags(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	// p_motor->FeedbackModeFlags.State = Motor_ConvertFeedbackModeFlags(mode).State;
	// p_motor->FeedbackModeFlags.OpenLoop |= Motor_ConvertFeedbackModeFlags(mode).OpenLoop;
	Motor_SetFeedbackModeFlags(p_motor, mode);
	if(p_motor->FeedbackModeFlags.OpenLoop == 1U) { p_motor->ControlFlags.SensorFeedback = 0U; } /* Clear only */
}

/******************************************************************************/
/*
	Common Sets
*/
/******************************************************************************/
static inline void Motor_ResetSpeedLimits(Motor_T * p_motor)
{
	p_motor->SpeedLimitCcw_Frac16 = p_motor->Parameters.SpeedLimitCcw_Frac16;
	p_motor->SpeedLimitCw_Frac16 = p_motor->Parameters.SpeedLimitCw_Frac16;
	// p_motor->SpeedLimitActiveId = MOTOR_SPEED_LIMIT_ACTIVE_DISABLE;
	// p_motor->SpeedLimitActiveScalar = 0xFFFF;
}

static inline void Motor_ResetILimits(Motor_T * p_motor)
{
	p_motor->ILimitMotoring_Frac16 = p_motor->Parameters.ILimitMotoring_Frac16;
	p_motor->ILimitGenerating_Frac16 = p_motor->Parameters.ILimitGenerating_Frac16;
	p_motor->ILimitActiveSentinel = 0xFFFFU; /* Use for comparison on set */
}


/******************************************************************************/
/*
	Motor Encoder Wrapper
*/
/******************************************************************************/
/*!
	Electrical Theta Angle, position Angle [Degree16s]
		=> MechanicalTheta * PolePairs

	todo push to encoder
*/
static inline int16_t Motor_GetEncoderElectricalAngle(Motor_T * p_motor)
{
	int16_t angle = ((_Encoder_GetAngle32(&p_motor->Encoder) >> 6U) * p_motor->Parameters.PolePairs) >> 10U;  /* MotorPolePairs less than 64 */
	return Encoder_GetDirection_Quadrature(&p_motor->Encoder) * angle;
}


/*
	Restore Run Ramp after OpenLoop/Align, MatchOutput will overwrite target value
	alternatively allocate Ramp for Align/OpenLoop
*/
// static inline void Motor_ResetRampSlope(Motor_T * p_motor)
// {
// 	Linear_Ramp_SetSlope(&p_motor->Ramp, p_motor->Parameters.RampAccel_Cycles, 0U, INT16_MAX);
// }

/*!
	Convert user reference direction to CCW/CW direction
	@param[in] userCmd int16_t[-32768:32767]
	@return int32_t[-32768:32768], Over saturated if input is -32768
*/
static inline int32_t _Motor_ConvertDirectionalCmd(Motor_T * p_motor, int16_t userCmd)
{
	return (p_motor->Direction == MOTOR_DIRECTION_CCW) ? userCmd : (int32_t)0 - userCmd;
}

static inline void Motor_SetDirectionalCmd(Motor_T * p_motor, int16_t userCmd)
{
	Linear_Ramp_SetTarget(&p_motor->Ramp, _Motor_ConvertDirectionalCmd(p_motor, userCmd));
}

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);
extern void Motor_InitSensor(Motor_T * p_motor);

extern void Motor_ResetPidILimits(Motor_T * p_motor);
extern void Motor_SetLimitsCcw(Motor_T * p_motor);
extern void Motor_SetLimitsCw(Motor_T * p_motor);
extern void Motor_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_SetDirectionCw(Motor_T * p_motor);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirectionForward(Motor_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_T * p_motor);
extern void Motor_ZeroSensor(Motor_T * p_motor);
extern qangle16_t Motor_GetMechanicalAngle(Motor_T * p_motor);

extern void Motor_ResetUnitsSensor(Motor_T * p_motor);
extern void Motor_ResetUnitsVabc(Motor_T * p_motor);
extern void Motor_ResetUnitsIabc(Motor_T * p_motor);
extern void Motor_ResetUnitsHallEncoder(Motor_T * p_motor);
extern void Motor_ResetUnitsEncoder(Motor_T * p_motor);
extern void Motor_ResetUnitsSinCos(Motor_T * p_motor);
extern void Motor_ResetUnitsAngleSpeed_Mech(Motor_T * p_motor);
extern void Motor_ResetUnitsAngleSpeed_ElecControl(Motor_T * p_motor);
extern void Motor_ResetUnitsVSpeed(Motor_T * p_motor);

extern void Motor_Jog12Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6PhaseStep(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog12(Motor_T * p_motor);
extern void Motor_Jog6Phase(Motor_T * p_motor);
extern void Motor_Jog6(Motor_T * p_motor);

#endif
