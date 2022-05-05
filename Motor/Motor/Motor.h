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

	User set
*/
typedef enum Motor_ControlMode_Tag
{
	MOTOR_CONTROL_MODE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,
}
Motor_ControlMode_T; //Motor_FeedbackMode_T 

/*
	Active state, checked on runtime
*/
typedef union Motor_ControlModeFlags_Tag
{
	struct
	{
		uint32_t OpenLoop : 1; //0 -> position feedback, 1 -> Openloop, control mode live toggle
		uint32_t Speed : 1;	//0 -> const voltage or current, 1 -> speed feedback
		uint32_t Current : 1;	//0 -> voltage, 1-> current
		uint32_t VFreqScalar : 1; //Use v scalar 
		uint32_t Hold : 1;
		uint32_t Update : 1;
		//		uint32_t Regen 			:1;
		//		uint32_t Direction 		:1;
	};
	uint32_t State;
}
Motor_ControlModeFlags_T;

/*
	Effectively sync mailbox for calculation results performed at different frequencies
*/
	// struct
	// {
	// 	uint32_t IWarningActive 	: 1U; /* Set approx 1/s, check 1/ms */
	// 	uint32_t HeatWarning 		: 1U;
	// 	uint32_t VWarning 			: 1U; 
	// };

typedef union Motor_RunStateFlags_Tag
{
	struct
	{
		uint32_t Hold 				: 1U;
		uint32_t ILimitActive 		: 1U;
		uint32_t IWarningActive 	: 1U; /* Set approx 1/s, check 1/ms */
		uint32_t HeatWarning 		: 1U;
		uint32_t VWarning 			: 1U;
		uint32_t FieldWeakening 	: 1U; //todo
	};
	uint32_t State;
}
Motor_RunStateFlags_T;

typedef enum Motor_CmdMode_Tag
{
	MOTOR_CMD_MODE_MOTORING,
	MOTOR_CMD_MODE_GENERATING, 
}
Motor_CmdMode_T;

// typedef union Motor_ControlCmdFlags_Tag
// {
// 	struct
// 	{ 
// 		uint32_t Motoring : 1U;	 
// 		uint32_t Current : 1U;  
// 		uint32_t Update : 1U; 
// 	};
// 	uint32_t State;
// }
// Motor_CmdModeFlags_T;

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
	MOTOR_DIRECTION_CW = 0U,
	MOTOR_DIRECTION_CCW = 1U,
}
Motor_Direction_T;

typedef enum Motor_DirectionCalibration_Tag
{
	MOTOR_FORWARD_IS_CW,
	MOTOR_FORWARD_IS_CCW,
}
Motor_DirectionCalibration_T;

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

// /*
//  * Fault/Run Substate
//  */
// typedef enum
// {
// 	MOTOR_STATUS_OKAY,
// 	MOTOR_STATUS_ERROR_1,
// } Motor_Status_T;

// typedef union
// {
// 	struct
// 	{
// 		uint32_t OverHeat :1U;
// //		uint32_t UserCheck :1U;
// 	};
// 	uint32_t State;
// } 
// Motor_ErrorFlags_T;

// typedef union
// {
// 	struct
// 	{
// 		uint32_t IOverLimit :1U;
// 	};
// 	uint32_t State;
// } 
// Motor_WarningFlags_T;


/*!
	@brief Motor Parameters
	Runtime variable configuration

	load from NvM
*/
typedef struct __attribute__((aligned(4U))) Motor_Params_Tag
{
	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_ControlMode_T 		ControlMode; /* User ControlMode, effective for throttle only */
//	Motor_BrakeMode_T 			BrakeMode;
	Motor_AlignMode_T 			AlignMode;
	Phase_Mode_T				PhasePwmMode; /* Only 1 nvm param for phase module. */

	Motor_DirectionCalibration_T DirectionCalibration;

	uint8_t PolePairs;
	uint16_t VSupply; /* use static? Vabc unit conversion */

	// uint16_t IBusLimit_Frac16; 	/* Six-Step   */
	// qfrac16_t IqLimit;			/* FOC   */
	// uint16_t VPwmLimit;
	uint16_t ILimitMotoring_Frac16;	/* User ILimit  */
	uint16_t ILimitGenerating_Frac16;
	uint16_t ILimitHeat_Frac16;	 //absolute or ratio?
	// uint16_t ILimitLowV_Frac16;

	uint16_t SpeedLimitCcw_Frac16; 
	uint16_t SpeedLimitCw_Frac16; 
	// uint16_t SpeedLimitFwd_RPM; 
	// uint16_t SpeedLimitRev_RPM; 

	uint16_t SpeedRefMax_RPM;		/* Ref and limit, pass to encoder and calc user. spec to motor/vsupply? */
	uint16_t SpeedRefVoltage_RPM; 	/* bemf matching */

	uint16_t IRefMax_Amp;		/* store for Linear_ADC, known calibration parameter provide by user */
	uint16_t IRefPeak_ADCU; 	/* Zero-To-Peak derived from, known calibration parameter provide by user */

	uint16_t IaRefZero_ADCU;
	uint16_t IbRefZero_ADCU;
	uint16_t IcRefZero_ADCU;
	uint16_t IaRefMax_ADCU;
	uint16_t IbRefMax_ADCU;
	uint16_t IcRefMax_ADCU;

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

	// uint16_t VFreqScalar;
}
Motor_Params_T;

/*
	@brief Motor Init
	Compile time const configuration
*/
typedef const struct Motor_Init_Tag
{
	// const uint16_t I_MAX_AMP;
	// const uint16_t I_MAX_ADCU; 
	const uint16_t UNIT_VABC_R1;
	const uint16_t UNIT_VABC_R2;
	AnalogN_T * const P_ANALOG_N;
	const MotorAnalog_Conversions_T ANALOG_CONVERSIONS;
	const Motor_Params_T * const P_PARAMS_NVM;
}
Motor_Config_T;

typedef struct Motor_Tag
{
	const Motor_Config_T CONFIG;	// compile time const, unique per motor
	Motor_Params_T Parameters; 		// load from eeprom

	volatile MotorAnalog_Results_T AnalogResults;

	Phase_T Phase;
	Encoder_T Encoder;
	Hall_T Hall;
	SinCos_T SinCos;
	Thermistor_T Thermistor;

	/*
		State and SubStates
	*/
	StateMachine_T StateMachine;

	/* Run State */
	Motor_Direction_T Direction; 		/* Active spin direction */
	Motor_Direction_T UserDirection; 	/* Passed to StateMachine */
	Motor_ControlModeFlags_T ControlModeFlags;
	Motor_RunStateFlags_T RunStateFlags; /* run substate */

	Motor_CmdMode_T CmdMode;
	//ilimit factor_frac16

	// Motor_ErrorFlags_T ErrorFlags;
	// Motor_WarningFlags_T WarningFlags;

	Motor_CalibrationState_T CalibrationState; /* Substate, selection for calibration */
	uint8_t CalibrationStateStep;
	// bool IsPwmOn;
	bool IOverLimitFlag;

	/*
		UserCmd Input
	*/
	Linear_T Ramp; //	Linear_T RampUp; //	Linear_T RampDown;
	uint32_t RampIndex;
	int32_t RampCmd;	//SetPoint after ramp, VReq/IReq/SpeedReq

	/*
		Speed Feedback
	*/
	PID_T PidSpeed;						/* Input SpeedFeedback_Frac16 Q0.16, Output SpeedControl => VPwm, Vq, Iq,  */
	Timer_T SpeedTimer;					/* Speed Calc Timer */
	uint32_t SpeedFeedback_Frac16; 		/* Feedback Variable, can over saturate */
	int32_t SpeedControl; 				/* Speed PID output, (SpeedFeedback_Frac16 - RampCmd) => (VPwm, Vq, Iq), updated once per millis */
		//	uint16_t Speed_RPM;			
		// uint16_t Speed2_RPM;
		// uint32_t Speed2_Frac16;

	uint16_t VBemfPeak_ADCU;
	uint16_t VBemfPeakTemp_ADCU;

	/*
		Unit Conversions
	*/
	Linear_T UnitIa; 	//Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;
	Linear_T UnitVabc;	//Bemf V and mV conversion

	Filter_T FilterA; 	//Calibration use
	Filter_T FilterB;
	Filter_T FilterC;

	uint32_t ControlTimerBase;	 	/* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
	Timer_T ControlTimer; 			/* State Timer, openloop, Bem */
	Timer_T MillisTimer; 			//  millis thread
//	Timer_T SecondsTimer; 			//  Heat thread

	/*
		FOC
	*/
	FOC_T Foc;
	PID_T PidId;
	PID_T PidIq;					/* Input  Iq, Output Vq, sign indicates direction */
	qangle16_t ElectricalAngle;		/* Save for UI use */
	// uint32_t ElectricalDelta;

	Linear_T UnitAngleRpm; //non encoder/hall sensor
	qangle16_t MechanicalAngle;		/* Save for UI use */
	// uint32_t MechanicalDelta;
	qangle16_t SpeedAngle;
	// uint32_t SpeedDelta;

	/* interpolated angle */
	qangle16_t HallAngle;
	uint32_t InterpolatedAngleIndex;

	/*
		Six-Step
	*/
	PID_T PidIBus; 
//	BEMF_T Bemf;
	Motor_SectorId_T NextPhase;		 		//for 6 step openloop/sensorless
	Motor_SectorId_T CommutationPhase;	 	//for 6 step openloop/sensorless
	uint32_t CommutationTimeRef;
	uint32_t IBus_Frac16;
	uint32_t IBusSum_Frac16;
	//	uint32_t IBusSum_ADCU;
	//	uint32_t IBusPrev_Frac16;
	uint16_t VPwm; 				//Control Variable

	/*
		Open-loop
	*/
	bool IsOpenLoop;
	Linear_T OpenLoopRamp;
	uint32_t OpenLoopRampIndex;
	uint32_t OpenLoopCommutationPeriod;
	uint16_t OpenLoopSpeed_RPM;
	uint16_t OpenLoopVPwm;

	uint32_t MicrosRef; //debug
	volatile uint32_t DebugTime[10U];
	volatile uint32_t Debug[20U];
	uint32_t JogIndex;
	//	uint32_t StepCount;
}
Motor_T;

/******************************************************************************/
/*!
	Motor Common
	@{
*/
/******************************************************************************/
//void MotorAnalog_EnqueueGroupV(Motor_T * p_motor)
//{
//} 

/*
	CCW or CW
	state machine protected
*/
static inline void Motor_SetDirection(Motor_T * p_motor, Motor_Direction_T direction)
{
	p_motor->Direction = direction;
 
	if(direction == MOTOR_DIRECTION_CCW)
	{ 
		// pidspeedlimit	
	}
	else//  MOTOR_DIRECTION_CW = 1U
	{

	}
	 

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP:
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_ENCODER:
			break;

		case MOTOR_SENSOR_MODE_HALL:
			if(direction == MOTOR_DIRECTION_CW) 	{ Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CW); }
			else 									{ Hall_SetDirection(&p_motor->Hall, HALL_DIRECTION_CCW); }
			break;

		default:
			break;
	} 
}

/*
	Forward/Reverse using calibration param
*/
static inline void Motor_SetDirectionForward(Motor_T * p_motor)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) { Motor_SetDirection(p_motor, MOTOR_DIRECTION_CCW); }
	else { Motor_SetDirection(p_motor, MOTOR_DIRECTION_CW); }
}

static inline void Motor_SetDirectionReverse(Motor_T * p_motor)
{
	if(p_motor->Parameters.DirectionCalibration == MOTOR_FORWARD_IS_CCW) { Motor_SetDirection(p_motor, MOTOR_DIRECTION_CW); }
	else { Motor_SetDirection(p_motor, MOTOR_DIRECTION_CCW); }
}

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
	/*
		Constant acceleration ramp
	*/
	Linear_Ramp_SetTarget(&p_motor->Ramp, userCmd);
}

/*
	dynamically generated Ramp,
	divide input over control period intervals, when using 1ms period
*/
static inline void Motor_SetRampInterpolate(Motor_T * p_motor, int32_t userCmd)
{
	// Linear_Ramp_SetSlopeMillis(&p_motor->Ramp, 1U, 20000U, p_motor->RampCmd, userCmd);
}

static inline void Motor_ResetRamp(Motor_T * p_motor)
{
	p_motor->RampCmd = 0U;
	//	p_motor->RampIndex = 0U;
	Linear_Ramp_SetTarget(&p_motor->Ramp, 0);
}

/*
	match ramp output
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
/* Speed PID always use directionless positive value [0:65535] */
static inline void Motor_ResumeSpeedOutput(Motor_T * p_motor, int32_t speedControlMatch)
{
	Motor_ResumeRampOutput(p_motor, p_motor->SpeedFeedback_Frac16); // req start from present speed
	PID_SetIntegral(&p_motor->PidSpeed, speedControlMatch);
	p_motor->SpeedControl = speedControlMatch; // output SpeedControl is V or I
}

static inline void Motor_PollDeltaTStop(Motor_T * p_motor)
{
	if(Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) //once per millis
	{ 
		p_motor->SpeedFeedback_Frac16 = 0U;
	}
}

static inline uint16_t Motor_ConvertToSpeedFrac16(Motor_T * p_motor, uint16_t speed_rpm)
{
	return speed_rpm * 65536U / p_motor->Parameters.SpeedRefMax_RPM;
}

static inline uint16_t Motor_ConvertToSpeedRpm(Motor_T * p_motor, uint16_t speed_frac16)
{
	return speed_frac16 * p_motor->Parameters.SpeedRefMax_RPM / 65536U;
}

// static inline uint16_t Motor_GetSpeed_RPM(Motor_T * p_motor)
// {
// 	return p_motor->SpeedFeedback_Frac16 * p_motor->Parameters.SpeedRefMax_RPM / 65536U;
// } 
 
/******************************************************************************/
/*
	Control Mode
*/
/******************************************************************************/
static inline Motor_ControlModeFlags_T Motor_ConvertControlModeFlags(Motor_ControlMode_T mode)
{
	static const Motor_ControlModeFlags_T MODE_OPEN_LOOP 		= { .OpenLoop = 1U, .Speed = 0U, .Current = 0U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_ControlModeFlags_T MODE_VOLTAGE 			= { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_ControlModeFlags_T MODE_VOLTAGE_FREQ 	= { .OpenLoop = 0U, .Speed = 0U, .Current = 0U, .VFreqScalar = 1U, .Update = 0U, };
	static const Motor_ControlModeFlags_T MODE_CURRENT 			= { .OpenLoop = 0U, .Speed = 0U, .Current = 1U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_ControlModeFlags_T MODE_SPEED_VOLTAGE 	= { .OpenLoop = 0U, .Speed = 1U, .Current = 0U, .VFreqScalar = 0U, .Update = 0U, };
	static const Motor_ControlModeFlags_T MODE_SPEED_CURRENT 	= { .OpenLoop = 0U, .Speed = 1U, .Current = 1U, .VFreqScalar = 0U, .Update = 0U, };

	Motor_ControlModeFlags_T flags;

	switch(mode)
	{
		case MOTOR_CONTROL_MODE_OPEN_LOOP:					flags.State = MODE_OPEN_LOOP.State; 	break;
		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:			flags.State = MODE_VOLTAGE.State;		break;
		case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:		flags.State = MODE_VOLTAGE_FREQ.State;	break;
		case MOTOR_CONTROL_MODE_CONSTANT_CURRENT:			flags.State = MODE_CURRENT.State;		break;
		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:		flags.State = MODE_SPEED_VOLTAGE.State;	break;
		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT:		flags.State = MODE_SPEED_CURRENT.State;	break;
		default: flags.State = 0; break;
	}

	return flags;
}

static inline bool Motor_CheckControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
{
	if(p_motor->ControlModeFlags.State != Motor_ConvertControlModeFlags(mode).State)
	{
		p_motor->ControlModeFlags.Update = 1U;
	}

	return p_motor->ControlModeFlags.Update;
}

static inline void Motor_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode)
{
	p_motor->ControlModeFlags.State = Motor_ConvertControlModeFlags(mode).State;
}


/******************************************************************************/
/*
	Common Sets
*/
/******************************************************************************/


/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern uint16_t _Motor_GetAdcVRef(void);

extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);
extern bool Motor_CheckControlMode(Motor_T * p_motor, Motor_ControlMode_T mode);
extern void Motor_SetControlMode(Motor_T * p_motor, Motor_ControlMode_T mode);

#endif
