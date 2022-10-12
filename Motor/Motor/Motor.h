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
#include "Math/Linear/Linear_Speed.h"
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

typedef enum Motor_SensorMode_Tag
{
	MOTOR_SENSOR_MODE_OPEN_LOOP,
	MOTOR_SENSOR_MODE_HALL,
	MOTOR_SENSOR_MODE_ENCODER,
	MOTOR_SENSOR_MODE_SIN_COS,
	MOTOR_SENSOR_MODE_SENSORLESS,
}
Motor_SensorMode_T;

/*
	Feedback Control Variable Mode
*/
typedef enum Motor_FeedbackMode_Tag
{
	MOTOR_FEEDBACK_MODE_OPEN_LOOP,
	MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE,
	MOTOR_FEEDBACK_MODE_VOLTAGE_FREQ_SCALAR,
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
		uint32_t OpenLoop 		: 1U; 	/* 0 -> Position feedback, 1 -> Openloop*/
		uint32_t Speed 			: 1U;	/* 0 -> Voltage or Current only, 1 -> Speed feedback */
		uint32_t Current 		: 1U;	/* 0 -> Voltage, 1-> Current */
		uint32_t VFreqScalar 	: 1U; 	/* 1-> Use VFreqScalar  */
		uint32_t Update 		: 1U;	/* Control Mode Update, shared */
	};
	uint32_t State;
}
Motor_FeedbackModeFlags_T;

//move control mode update
// typedef union Motor_ControlFlags_Tag
// {
// 	struct
// 	{
// 		Motor_FeedbackModeFlags_T FeedbackModeFlags;
// 		uint32_t Active 			: 1U;
// 		uint32_t FieldWeakening 	: 1U;
// 	};
// 	uint32_t State;
// }
// Motor_ControlFlags_T;

/*
	Effectively sync mailbox for async calculations
*/
typedef union Motor_RunStateFlags_Tag
{
	struct
	{
		uint32_t HeatWarning 				: 1U;
		uint32_t VoltageModeILimitActive 	: 1U;
		// uint32_t SpeedLimitScalarActive 	: 1U;
		// uint32_t ILimitScalarActive 		: 1U; /* Set approx 1/s */
		// uint32_t FieldWeakening 			: 1U; //todo
	};
	uint32_t State;
}
Motor_RunStateFlags_T;

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
	MOTOR_I_LIMIT_ACTIVE_HEAT = 1U,
	MOTOR_I_LIMIT_ACTIVE_SYSTEM = 20U,  /* From upper module */
	MOTOR_I_LIMIT_ACTIVE_USER = 30U,
}
Motor_ILimitActiveId_T;

typedef enum Motor_TorqueMode_Tag
{
	MOTOR_TORQUE_MODE_MOTORING,
	MOTOR_TORQUE_MODE_GENERATING,
}
Motor_TorqueDirection_T;

typedef enum Motor_AlignMode_Tag
{
	MOTOR_ALIGN_MODE_DISABLE,
	MOTOR_ALIGN_MODE_ALIGN,
	MOTOR_ALIGN_MODE_HFI,
}
Motor_AlignMode_T;

/*
	Direction Run Substate
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
	Calibration Substate Flag
*/
typedef enum Motor_CalibrationState_Tag
{
	MOTOR_CALIBRATION_STATE_DISABLE,
	MOTOR_CALIBRATION_STATE_ADC,
	MOTOR_CALIBRATION_STATE_HALL,
	MOTOR_CALIBRATION_STATE_ENCODER,
	MOTOR_CALIBRATION_STATE_SIN_COS,
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

	MOTOR_PHASE_ERROR_0 = 0U,
	MOTOR_PHASE_AC = 1U,
	MOTOR_PHASE_BC = 2U,
	MOTOR_PHASE_BA = 3U,
	MOTOR_PHASE_CA = 4U,
	MOTOR_PHASE_CB = 5U,
	MOTOR_PHASE_AB = 6U,
	MOTOR_PHASE_ERROR_7 = 7U,
}
Motor_SectorId_T;

/*!
	@brief Motor Parameters - Runtime variable configuration. Load from non volatile memory.
*/
typedef struct __attribute__((aligned(4U))) Motor_Params_Tag
{
	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_FeedbackMode_T 		FeedbackMode; 	/* User FeedbackMode, UserControlModeCmd, and ThrottleCmd */
	Motor_AlignMode_T 			AlignMode;
	Motor_DirectionCalibration_T DirectionCalibration;
	uint8_t PolePairs;

 	/*
		Ref values, known calibration parameter provide by user
	*/

	/*
		Motor Refs use Speed at VSource.
	*/
	uint16_t SpeedFeedbackRef_Rpm; 	/* Feedback / PID Regulator, Limits Ref. User IO units conversion, Encoder speed calc ref. */
	uint16_t SpeedVMatchRef_Rpm; 	/* Votlage Match Ref. VF Mode, Freewheel to Run. Use higher value to bias speed matching to begin at lower speed. */

	uint16_t IPeakRef_Adcu; 		/* Zero-To-Peak, derived from sensor hardware */ //todo change to I_ZERO_TO_PEAK_ADCU
	uint16_t IaZeroRef_Adcu;
	uint16_t IbZeroRef_Adcu;
	uint16_t IcZeroRef_Adcu;

	/* "Root" Limits */
 	uint16_t SpeedLimitCcw_Frac16;		/* Persistent User Param. Frac16 of SpeedFeedbackRef_Rpm */
	uint16_t SpeedLimitCw_Frac16;
	uint16_t ILimitMotoring_Frac16;		/* Persistent User Param. Frac16 of RefMax I_MAX_AMP */
	uint16_t ILimitGenerating_Frac16;
	uint16_t ILimitHeat_Frac16; 		/* Base Heat Limit. Active on thermistor warning. Frac16 scalar on active limit */

	uint16_t AlignVoltage_Frac16;
	uint16_t AlignTime_ControlCycles;
	// uint16_t VoltageBrakeScalar_InvFrac16; /* [0:65535], 0 is highest intensity */
	//	uint8_t BrakeCoeffcient;
	//	uint32_t RampAcceleration;

	uint16_t OpenLoopVPwmMin;
	uint16_t OpenLoopVPwmMax;
	uint16_t OpenLoopSpeedStart;
	uint16_t OpenLoopSpeedFinal;
	uint16_t OpenLoopAccel;
	//	uint16_t OpenLoopVHzGain;
	//	uint16_t OpenLoopZcdTransition;

	Phase_Mode_T PhasePwmMode; /* Only 1 nvm param for phase module. */
}
Motor_Params_T;

/*!
	@brief Motor Config - Compile time const configuration. Unique per Motor
*/
typedef const struct Motor_Init_Tag
{
	const uint16_t UNIT_VABC_R1;
	const uint16_t UNIT_VABC_R2;
	const uint16_t I_MAX_AMP; 				/* Motor I controller rating. pass to Linear_ADC. Unit conversion, ui output and param set. */
	const uint16_t I_ZERO_TO_PEAK_ADCU; 	/* Sensor calibration */
	AnalogN_T * const P_ANALOG_N;
	const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
	const Motor_Params_T * const P_PARAMS_NVM;
}
Motor_Config_T;

typedef struct Motor_Tag
{
	const Motor_Config_T CONFIG;
	Motor_Params_T Parameters;

	volatile MotorAnalog_Results_T AnalogResults;

	Phase_T Phase;
	Encoder_T Encoder;
	Hall_T Hall;
	SinCos_T SinCos;
	Thermistor_T Thermistor;

	/*
		State and Substates
	*/
	StateMachine_T StateMachine;

	uint32_t ControlTimerBase;	 	/* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
	Timer_T ControlTimer; 			/* State Timer, openloop, Bemf */

	/* Run Substate */
	Motor_Direction_T Direction; 			/* Active spin direction */
	Motor_Direction_T UserDirection; 		/* Passed to StateMachine */
	Motor_FeedbackModeFlags_T FeedbackModeFlags;
	Motor_RunStateFlags_T RunStateFlags; 	/* Run Substate */
	// Motor_TorqueDirection_T TorqueDirection;

	/*
		Limits
	*/
	uint16_t SpeedLimitCcw_Frac16; 		/* Active SpeedLimit, Frac16 of SpeedRefMax Param */
	uint16_t SpeedLimitCw_Frac16;
	uint16_t SpeedLimit_Frac16; 		/* Active SpeedLimit, optionally reduce 1 check of direction during User_SetFeedbackCmd */
	// Motor_SpeedLimitActiveId_T SpeedLimitActiveId;
	// uint16_t SpeedLimitActiveScalar;

	uint16_t ILimitMotoring_Frac16;		/* Active ILimit */
	uint16_t ILimitGenerating_Frac16;
	Motor_ILimitActiveId_T ILimitActiveId;
	uint16_t ILimitActiveScalar;		/* Store for comparison */
	int16_t VoltageModeILimit_QFracS16; /* [-32767:32767] directional input into pid */
	Linear_T ILimitHeatRate;

	/* Calibration Substate */
	Motor_CalibrationState_T CalibrationState; 	/* Substate, selection for calibration */
	uint8_t CalibrationStateIndex;

	/* Jog */
	uint32_t JogIndex;

	uint16_t VBemfPeak_Adcu; //todo
	uint16_t VBemfPeakTemp_Adcu;
	uint16_t IPhasePeak_Adcu;
	uint16_t IPhasePeakTemp_Adcu;
	uint16_t IPhasePeak2_Adcu;

	/*
		UserCmd Input => Ramp
	*/
	Linear_T Ramp;
	int32_t RampCmd;		/* [-32767:32767] SetPoint after ramp => SpeedReq, IReq, VReq. [0:65535] VFreq Mode */
	uint32_t RampIndex;		/* Index mode only */

	/*
		Speed Feedback
	*/
	PID_T PidSpeed;					/* Input SpeedFeedback_Frac16 Q16.16, Output SpeedControl => VPwm, Vq, Iq */
	Timer_T SpeedTimer;				/* Speed Calc Timer */
	int32_t SpeedFeedback_Frac16; 	/* [~-65535*2:~65535*2] Speed Feedback Variable. Can over saturate */
	int32_t SpeedControl; 			/* [~-32767:~32767] Speed Control Variable. PidSpeed(RampCmd - SpeedFeedback_Frac16 / 2) => VPwm, Vq, Iq. Updated once per millis */
	// uint32_t Speed2_Frac16;

	/*
		FOC
	*/
	FOC_T Foc;
	PID_T PidIq;					/* Input (IqReq - IqFeedback), Output Vq. Sign indicates ccw/cw direction */
	PID_T PidId;
	qangle16_t ElectricalAngle;		/* Save for user output, can remove later */
	// uint32_t ElectricalDelta;

	Linear_T UnitAngleRpm; 			/* Non Encoder/Hall sensor */
	qangle16_t SpeedAngle; 			/* Save for reference */
	// uint32_t MechanicalDelta;

	/* Interpolated angle */
	qangle16_t HallAngle;
	uint32_t InterpolatedAngleIndex;

	/*
		Six-Step
	*/
	PID_T PidIBus;
//	BEMF_T Bemf;
	// Motor_SectorId_T NextPhase;
	// Motor_SectorId_T CommutationPhase;
	// uint32_t CommutationTimeRef;
	// uint32_t IBus_Frac16;
	// uint32_t IBusSum_Frac16;
	// uint16_t VPwm; 	/* Six-Step Control Variable */

	/*
		Open-loop
	*/
	// bool IsOpenLoop;
	Linear_T OpenLoopRamp;
	uint32_t OpenLoopRampIndex;
	uint32_t OpenLoopCommutationPeriod;
	uint16_t OpenLoopSpeed_RPM;
	uint16_t OpenLoopVPwm;

	uint32_t MicrosRef; //debug
	volatile uint32_t DebugTime[10U];
	volatile uint32_t Debug[20U];

	/*
		Unit Conversions
	*/
	Linear_T UnitIa; 	//Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;
	Linear_T UnitVabc;	//Bemf V and mV conversion
	Linear_T SpeedVMatchRatio; 	/* SpeedVMatch_Factor = SpeedFeedbackRef_Rpm << 14 / SpeedVMatchRef_Rpm */

	Filter_T FilterA; 	//Calibration use
	Filter_T FilterB;
	Filter_T FilterC;
}
Motor_T;

static inline void Motor_DisablePwm(Motor_T * p_motor) 			{ Phase_DisableInterrupt(&p_motor->Phase); } 	//todo
static inline void Motor_EnablePwm(Motor_T * p_motor) 			{ Phase_EnableInterrupt(&p_motor->Phase); } 	//todo
static inline void Motor_ClearPwmInterrupt(Motor_T * p_motor) 	{ Phase_ClearInterrupt(&p_motor->Phase); }

/******************************************************************************/
/*
	Ramp
*/
/******************************************************************************/
/*
	Proc 20000Hz
*/
static inline void Motor_ProcRamp(Motor_T * p_motor)
{
	//index mode check negative
//	p_motor->RampCmd = Linear_Ramp_ProcIndexOutput(&p_motor->Ramp, &p_motor->RampIndex, p_motor->RampCmd);
//	p_motor->RampCmd = Linear_Ramp_GetTarget(&p_motor->Ramp); //disables ramp
	p_motor->RampCmd = Linear_Ramp_CalcNextOutput(&p_motor->Ramp, p_motor->RampCmd);
}

/*
	Set 1000Hz
*/
static inline void Motor_SetRamp(Motor_T * p_motor, int32_t userCmd)
{
	Linear_Ramp_SetTarget(&p_motor->Ramp, userCmd); 	/* Constant acceleration ramp */
}

/*
	dynamically generated Ramp,
	divide input over control period intervals, when using 1ms period
	acceleration proportional to change in userCmd
*/
// static inline void Motor_SetRampInterpolate(Motor_T * p_motor, int32_t userCmd)
// {
// 	// Linear_Ramp_SetSlopeMillis(&p_motor->Ramp, 1U, 20000U, p_motor->RampCmd, userCmd);
// }

static inline void Motor_ResetRamp(Motor_T * p_motor)
{
	p_motor->RampCmd = 0;
	//	p_motor->RampIndex = 0U;
	Linear_Ramp_SetTarget(&p_motor->Ramp, 0);
}

/*
	Match ramp output
*/
static inline void Motor_SetRampOutput(Motor_T * p_motor, int32_t matchOutput)
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
static inline void Motor_SetSpeedOutput(Motor_T * p_motor, int32_t speedControlMatch)
{
	Motor_SetRampOutput(p_motor, p_motor->SpeedFeedback_Frac16 / 2U); 	/* RampOut/PidIn always use SpeedFeedback */
	PID_SetIntegral(&p_motor->PidSpeed, speedControlMatch);  			/* SpeedControl may be V or I */
	p_motor->SpeedControl = speedControlMatch;
}

static inline void Motor_PollDeltaTStop(Motor_T * p_motor)
{
	if(Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) { p_motor->SpeedFeedback_Frac16 = 0U; }
}

/******************************************************************************/
/*
	Reset Sensors/Align
*/
/******************************************************************************/
static inline void Motor_ZeroSensor(Motor_T * p_motor)
{
	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			p_motor->OpenLoopRampIndex = 0U;
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			Encoder_Zero(&p_motor->Encoder);
			break;

		case MOTOR_SENSOR_MODE_HALL:
			Encoder_Zero(&p_motor->Encoder); //zero angle speed //reset before Encoder_DeltaT_SetInitial
			Encoder_DeltaT_SetInitial(&p_motor->Encoder, 10U); /* Set first capture DeltaT = 0xffff */
			Hall_ResetCapture(&p_motor->Hall);
			break;

		case MOTOR_SENSOR_MODE_SIN_COS:
			break;

		default:
			break;
	}
}

/******************************************************************************/
/*
	Feedback Mode
*/
/******************************************************************************/
static inline Motor_FeedbackModeFlags_T Motor_ConvertFeedbackModeFlags(Motor_FeedbackMode_T mode)
{
	static const Motor_FeedbackModeFlags_T MODE_OPEN_LOOP 		= { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_VOLTAGE 		= { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_VOLTAGE_FREQ 	= { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .VFreqScalar = 1U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_CURRENT 		= { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_SPEED_VOLTAGE 	= { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_FeedbackModeFlags_T MODE_SPEED_CURRENT 	= { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, .VFreqScalar = 0U, .Update = 0U, };

	Motor_FeedbackModeFlags_T flags;

	switch(mode)
	{
		case MOTOR_FEEDBACK_MODE_OPEN_LOOP:					flags.State = MODE_OPEN_LOOP.State; 	break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_VOLTAGE:			flags.State = MODE_VOLTAGE.State;		break;
		case MOTOR_FEEDBACK_MODE_VOLTAGE_FREQ_SCALAR:		flags.State = MODE_VOLTAGE_FREQ.State;	break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_CURRENT:			flags.State = MODE_CURRENT.State;		break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_VOLTAGE:	flags.State = MODE_SPEED_VOLTAGE.State;	break;
		case MOTOR_FEEDBACK_MODE_CONSTANT_SPEED_CURRENT:	flags.State = MODE_SPEED_CURRENT.State;	break;
		default: flags.State = 0; break;
	}

	return flags;
}

/*!
	check feedback mode change and controlmode inactive
	@return true if needs update
*/
static inline bool Motor_CheckControlUpdate(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	// ControlFlags_T controlFlags;
	// controlFlags.FeedbackModeFlags.State = Motor_ConvertFeedbackModeFlags(mode).State;
	// return (p_motor->ControlFlags.State != controlFlags);

	return (p_motor->FeedbackModeFlags.State != Motor_ConvertFeedbackModeFlags(mode).State);
}

/*
	Sets flags only
	Motor_User_SetControlMode() applys flags to run state
*/
static inline void Motor_SetFeedbackModeFlags(Motor_T * p_motor, Motor_FeedbackMode_T mode)
{
	p_motor->FeedbackModeFlags.State = Motor_ConvertFeedbackModeFlags(mode).State;
}


/******************************************************************************/
/*
	Common Sets
*/
/******************************************************************************/
static inline int32_t Motor_ConvertToSpeedFrac16(Motor_T * p_motor, int32_t speed_rpm) 	{ return speed_rpm * 65535 / p_motor->Parameters.SpeedFeedbackRef_Rpm; }
static inline int16_t Motor_ConvertToSpeedRpm(Motor_T * p_motor, int32_t speed_frac16) 	{ return speed_frac16 * p_motor->Parameters.SpeedFeedbackRef_Rpm / 65536; }
static inline int32_t Motor_ConvertToIFrac16(Motor_T * p_motor, int32_t i_amp) 			{ return i_amp * 65535 / p_motor->CONFIG.I_MAX_AMP; }
static inline int16_t Motor_ConvertToIAmp(Motor_T * p_motor, int32_t i_frac16) 			{ return i_frac16 * p_motor->CONFIG.I_MAX_AMP / 65536; }

static inline void Motor_ResetSpeedLimits(Motor_T * p_motor)
{
	p_motor->SpeedLimitCcw_Frac16 = p_motor->Parameters.SpeedLimitCcw_Frac16;
	p_motor->SpeedLimitCw_Frac16 = p_motor->Parameters.SpeedLimitCw_Frac16;
}

static inline void Motor_ResetILimits(Motor_T * p_motor)
{
	p_motor->ILimitMotoring_Frac16 = p_motor->Parameters.ILimitMotoring_Frac16;
	p_motor->ILimitGenerating_Frac16 = p_motor->Parameters.ILimitGenerating_Frac16;
}

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern uint16_t _Motor_GetAdcVRef(void);
extern uint16_t _Motor_GetVSourceRef(void);
extern void Motor_InitAdcVRef_MilliV(uint16_t adcVRef_MilliV);
extern void Motor_InitVSourceRef_V(uint16_t vRefSupply);

extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);

extern void Motor_Jog12Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6PhaseStep(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6Step(Motor_T * p_motor, uint8_t step);
extern void Motor_Jog6(Motor_T * p_motor);
extern void Motor_Jog12(Motor_T * p_motor);

extern void Motor_SetDirectionCcw(Motor_T * p_motor);
extern void Motor_SetDirectionCw(Motor_T * p_motor);
extern void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction);
extern void Motor_SetDirectionForward(Motor_T * p_motor);
extern void Motor_SetDirectionReverse(Motor_T * p_motor);

extern void Motor_ResetSensorMode(Motor_T * p_motor);
extern void Motor_ResetUnitsVabc(Motor_T * p_motor);
extern void Motor_ResetUnitsIabc(Motor_T * p_motor);
extern void Motor_ResetUnitsHall(Motor_T * p_motor);
extern void Motor_ResetSpeedVMatchRatio(Motor_T * p_motor);

#endif
