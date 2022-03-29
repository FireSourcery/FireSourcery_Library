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
//#include "Transducer/BEMF/BEMF.h"

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

#define MOTOR_LIB_VERSION_OPT		0xFF
#define MOTOR_LIB_VERSION_MAJOR 	0
#define MOTOR_LIB_VERSION_MINOR 	0
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
	MOTOR_SENSOR_MODE_SENSORLESS,
} Motor_SensorMode_T;

/*
 * Feedback Control Variable Mode
 */
typedef enum
{
	MOTOR_CONTROL_MODE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
//	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,
}
Motor_ControlMode_T; //Motor_ControlModeUser_T FeedbackMode


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
//	uint32_t OpenLoop		:1; //0 -> position feedback, 1 -> Openloop, control mode live toggle
//	uint32_t Sensorless		:1; //0 -> encoder/hall, 1 -> Sensorless,
//	uint32_t Speed			:1;	//0 -> const voltage or current, 1 -> speed feedback
//	uint32_t Current		:1;	//0 -> voltage, 1-> current
//	uint32_t Brake			:1; // control mode toggle
//}
//Motor_ControlModeFlags_T;

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
	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_ControlMode_T 		ControlMode;
	Motor_BrakeMode_T 			BrakeMode;
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

	uint16_t SpeedMax_RPM;
	uint16_t IRefMax_Amp;
	uint16_t IaRefZero_ADCU;
	uint16_t IbRefZero_ADCU;
	uint16_t IcRefZero_ADCU;
	uint16_t IaRefMax_ADCU;
	uint16_t IbRefMax_ADCU;
	uint16_t IcRefMax_ADCU;

	uint16_t IBusLimit_Frac16; 	/* Six-Step PID */
	qfrac16_t IqLimit;			/* FOC PID */

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

	//const using fixed resistor values
//	const Linear_T UNIT_V_POS;
	const Linear_T UNIT_V_ABC; 	//Bemf V and mV conversion

	AnalogN_T * const P_ANALOG_N;

//	const AnalogN_Conversion_T CONVERSION_VPOS_PWM_ON;
	const AnalogN_AdcFlags_T ADCS_ACTIVE_PWM_THREAD;
//	const AnalogN_AdcFlags_T ADCS_ACTIVE_TIMER_THREAD;
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

}
Motor_Config_T;

typedef struct
{
 	const Motor_Config_T CONFIG;	// compile time const, unique per motor
 	Motor_Params_T Parameters; 		// load from eeprom

	Encoder_T Encoder;
	Phase_T Phase;
	Hall_T Hall;
	Thermistor_T Thermistor;

	/*
	 * State and SubStates
	 */
 	StateMachine_T StateMachine;

 	/* Run State */
	Motor_Direction_T Direction; 		/* Active spin direction */
	Motor_Direction_T UserDirection; 	/* Passed to StateMachine */
	bool Brake; //can change to quadrant to include plugging
//	Motor_ControlModeFlags_T ControlMode;

	Motor_ErrorFlags_T ErrorFlags;

	Motor_CalibrationState_T CalibrationState; /* Substate, selection for calibration */
	uint8_t CalibrationSubstateStep;
	bool IsPwmOn;
	bool IOverLimitFlag;

	volatile MotorAnalog_Results_T AnalogResults;

	uint32_t ControlTimerBase;	 	/* Control Freq ~ 20kHz, calibration, commutation, angle control. overflow at 20Khz, 59 hours*/
	Timer_T ControlTimer; 			/* State Timer, openloop, Bem */
	Timer_T MillisTimer; 			//  millis thread

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
	uint16_t RampCmd;			//SetPoint after ramp, VReq/IReq/SpeedReq

	/* Speed Feedback */
	PID_T PidSpeed;
	Timer_T SpeedTimer;			// SpeedCalc Timer
	uint16_t Speed_RPM;			// Common Feedback Variable
	uint32_t Speed_Frac16; 		/* Can over saturate */
	uint32_t SpeedControl; 		/* Speed PID output, (Speed_Frac16 - RampCmd) => (VPwm, Vq, Iq), updated once per millis */

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
	p_motor->RampCmd = Linear_Ramp_ProcOutput(&p_motor->Ramp, &p_motor->RampIndex, p_motor->RampCmd);
//	p_motor->RampCmd = Linear_Ramp_GetTarget(&p_motor->Ramp); //disables ramp
}

/*
 * Set 1000Hz
 */
static inline void Motor_SetRamp(Motor_T * p_motor, uint16_t userCmd)
{
	Linear_Ramp_SetTarget(&p_motor->Ramp, userCmd);
}

static inline void Motor_SetRampIndex(Motor_T * p_motor, uint16_t userCmd)
{
	Linear_Ramp_SetIndex(&p_motor->Ramp, &p_motor->RampIndex, userCmd);
}

static inline void Motor_ResetRamp(Motor_T * p_motor)
{
	p_motor->RampCmd = 0U;
	p_motor->RampIndex = 0U;
	Linear_Ramp_SetTarget(&p_motor->Ramp, 0);
}

/*
 * match proportional to speed
 */
static inline void Motor_ResumeRamp(Motor_T * p_motor)
{
	p_motor->RampCmd = p_motor->Speed_Frac16;
	Linear_Ramp_SetIndex(&p_motor->Ramp, &p_motor->RampIndex, p_motor->Speed_Frac16);
}


/******************************************************************************/
/*
	Speed
*/
/******************************************************************************/
static inline void Motor_CaptureSpeed(Motor_T * p_motor)
{
	p_motor->Speed_RPM = (Encoder_GetRotationalSpeed_RPM(&p_motor->Encoder) + p_motor->Speed_RPM) / 2U;
	p_motor->Speed_Frac16 = ((uint32_t)p_motor->Speed_RPM * (uint32_t)65535U / (uint32_t)p_motor->Parameters.SpeedMax_RPM);
}

/*
 * Common Speed PID Feedback Loop
 */
static inline void Motor_ProcSpeedFeedback(Motor_T * p_motor)
{
	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
		p_motor->SpeedControl = PID_Calc(&p_motor->PidSpeed, p_motor->RampCmd, p_motor->Speed_Frac16);
	}
}

static inline void Motor_PollDeltaTStop(Motor_T * p_motor)
{
	if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) //once per millis
	{
		p_motor->Speed_RPM = 0U;
		p_motor->Speed_Frac16 = 0U;
	}
}

//static inline void Motor_SetSpeedCmd(Motor_T * p_motor)
//{
//
//}

static inline uint32_t Motor_GetSpeed(Motor_T * p_motor)
{
	return p_motor->Speed_RPM;
}


/******************************************************************************/
/*
	StateMachine Mapped functions
*/
/******************************************************************************/

/******************************************************************************/
/*
	Idle Stop State
*/
/******************************************************************************/
static inline void Motor_StartIdle(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
	Motor_ResetRamp(p_motor);
//	p_motor->ControlTimerBase = 0U; //ok to reset timer
	Timer_StartPeriod(&p_motor->ControlTimer, 2000U); //100ms
}

static inline void Motor_ProcIdle(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VPOS);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
		AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);
		AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
	}
}

/******************************************************************************/
/*
	Align State
 */
/******************************************************************************/
//Motor_AlignMode_T Motor_GetAlignMode(Motor_T *p_motor)
//{
//	Motor_AlignMode_T alignMode;
//
//	if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
//	{
//		alignMode = MOTOR_ALIGN_MODE_DISABLE;
//	}
//	else
//	{
//		//		if useHFI alignMode= MOTOR_ALIGN_MODE_HFI;
//		//		else
//		alignMode = MOTOR_ALIGN_MODE_ALIGN;
//	}
//
//	return alignMode;
//}

static inline void Motor_StartAlign(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Parameters.AlignTime_ControlCycles);
	Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U);
	Phase_ActivateSwitchABC(&p_motor->Phase);
}

static inline bool Motor_ProcAlign(Motor_T * p_motor)
{
	bool status = Timer_Poll(&p_motor->ControlTimer);

	if(status == true)
	{
		p_motor->ElectricalAngle = 0U;
		Encoder_Reset(&p_motor->Encoder); //zero angularD
//		Motor_Float(&p_motor->Foc);
	}

	return status;
}

/******************************************************************************/
/*
	Calibration State Functions
 */
/******************************************************************************/
/*
 * Nonblocking Calibration State Functions
 */
static inline void Motor_SetCalibrationStateAdc(Motor_T * p_motor)		{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ADC;}
static inline void Motor_SetCalibrationStateHall(Motor_T * p_motor)		{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_HALL;}
static inline void Motor_SetCalibrationStateEncoder(Motor_T * p_motor)	{p_motor->CalibrationState = MOTOR_CALIBRATION_STATE_ENCODER;}

static void StartMotorCalibrateCommon(Motor_T * p_motor)
{
	p_motor->ControlTimerBase = 0U;
	Phase_Ground(&p_motor->Phase); //activates abc
	p_motor->CalibrationSubstateStep = 0U;
}

/*
 * Calibrate Current ADC
 */
static inline void Motor_StartCalibrateAdc(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 100000U); // Motor.Parameters.AdcCalibrationTime
//	FOC_SetZero(&p_motor->Foc);
//	Phase_ActuateDuty(&p_motor->Phase, FOC_GetDutyA(&p_motor->Foc), FOC_GetDutyB(&p_motor->Foc), FOC_GetDutyC(&p_motor->Foc)); //sets 0 current output

//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);

	p_motor->AnalogResults.Ia_ADCU = 2048U;
	p_motor->AnalogResults.Ib_ADCU = 2048U;
	p_motor->AnalogResults.Ic_ADCU = 2048U;

	Filter_MovAvg_InitN(&p_motor->FilterA, 2048U, 40U);
	Filter_MovAvg_InitN(&p_motor->FilterB, 2048U, 40U);
	Filter_MovAvg_InitN(&p_motor->FilterC, 2048U, 40U);
}

static inline bool Motor_CalibrateAdc(Motor_T *p_motor)
{
	bool isComplete = Timer_Poll(&p_motor->ControlTimer);

	if (isComplete == true)
	{
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaRefZero_ADCU, 4095U, p_motor->Parameters.IRefMax_Amp);
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbRefZero_ADCU, 4095U, p_motor->Parameters.IRefMax_Amp);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcRefZero_ADCU, 4095U, p_motor->Parameters.IRefMax_Amp);
		Phase_Float(&p_motor->Phase);
//		save params
	}
	else
	{
		p_motor->Parameters.IaRefZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		p_motor->Parameters.IbRefZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		p_motor->Parameters.IcRefZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);

		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
	}

	return isComplete;
}

/*
 * Calibrate Current ADC
 */
//static inline bool Motor_SixStep_CalibrateAdc(Motor_T *p_motor)
//{
////	bool isComplete = false;
//
////	 Timer_Poll(&p_motor->ControlTimer);
//
////	switch(p_motor->CalibrationSubstateStep)
////	{
////		case 0U :
////			Filter_MovAvg_InitN(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU, 1000U);
////			Filter_MovAvg_InitN(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU, 1000U);
////			Filter_MovAvg_InitN(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU, 1000U);
////			p_motor->CalibrationSubstateStep = 1U;
////			break;
////
////		case 1U:
////			if (Timer_Poll(&p_motor->ControlTimer) == false)
////			{
////				p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
////				p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
////				p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
////
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
////			}
////			else
////			{
////				p_motor->CalibrationSubstateStep = 2U;
////			}
////			break;
////
////		case 2U:
////
////			if (Timer_Poll(&p_motor->ControlTimer) == true)
////			{
////				Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
////				Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
////				Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
////				p_motor->CalibrationSubstateStep = 4U;
////			}
////			else
////			{
////				p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
////				p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
////				p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
////
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
////				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
////			}
////
////			//get bemf offset
//////		case 3U:
//////			Filter_MovAvg_InitN(&p_motor->FilterA, p_motor->AnalogResults.Va_ADCU, 1000U);
//////			Filter_MovAvg_InitN(&p_motor->FilterB, p_motor->AnalogResults.Vb_ADCU, 1000U);
//////			Filter_MovAvg_InitN(&p_motor->FilterC, p_motor->AnalogResults.Vc_ADCU, 1000U);
////
////		case 4U:
////
////			isComplete = true;
////
////
////		default :
////			break;
////	}
//
//	bool isComplete = Timer_Poll(&p_motor->ControlTimer);
//
//	if (isComplete == true)
//	{
//		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//		Phase_Float(&p_motor->Phase);
////		save params
//	}
//	else
//	{
//		p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
//		p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
//		p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
//
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
//		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);
//	}
//
//	return isComplete;
//}


static inline void Motor_StartCalibrateEncoder(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U);
	Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, 0U, 0U); /* Align Phase A 10% pwm */

}

static inline bool Motor_CalibrateEncoder(Motor_T * p_motor)
{
	bool isComplete = false;

	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationSubstateStep)
		{
			case 0U:
				Encoder_DeltaD_CalibrateQuadratureReference(&p_motor->Encoder);

				Phase_ActivateDuty(&p_motor->Phase, p_motor->Parameters.AlignVoltage_Frac16, p_motor->Parameters.AlignVoltage_Frac16, 0U);
				p_motor->CalibrationSubstateStep = 1U;
				break;

			case 1U:
				Encoder_DeltaD_CalibrateQuadraturePositive(&p_motor->Encoder);
				Phase_Float(&p_motor->Phase);
				p_motor->CalibrationSubstateStep = 0;
				isComplete = true;
				break;

			default:
				break;
		}
	}

	return isComplete;
}

static inline void Motor_StartCalibrateHall(Motor_T * p_motor)
{
	StartMotorCalibrateCommon(p_motor);
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); //Parameter.HallCalibrationTime
}

//120 degree hall aligned with phase
static inline bool Motor_CalibrateHall(Motor_T * p_motor)
{
	const uint16_t duty = p_motor->Parameters.AlignVoltage_Frac16;
	bool isComplete = false;

#ifndef CONFIG_HALL_DEBUG
	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		switch (p_motor->CalibrationSubstateStep)
		{
		case 0U:
			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, 0U);
			p_motor->CalibrationSubstateStep = 1U;
			break;

		case 1U:
			Hall_CalibratePhaseA(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, duty, 0U);
			p_motor->CalibrationSubstateStep = 2U;
			break;

		case 2U:
			Hall_CalibratePhaseInvC(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, 0);
			p_motor->CalibrationSubstateStep = 3U;
			break;

		case 3U:
			Hall_CalibratePhaseB(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, duty, duty);
			p_motor->CalibrationSubstateStep = 4U;
			break;

		case 4U:
			Hall_CalibratePhaseInvA(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, 0U, 0U, duty);
			p_motor->CalibrationSubstateStep = 5U;
			break;

		case 5U:
			Hall_CalibratePhaseC(&p_motor->Hall);
			Phase_ActivateDuty(&p_motor->Phase, duty, 0U, duty);
			p_motor->CalibrationSubstateStep = 6U;
			break;

		case 6U:
			Hall_CalibratePhaseInvB(&p_motor->Hall);
			Phase_Float(&p_motor->Phase);
			isComplete = true;
			break;

		default:
			break;
		}
	}
#else
	if (Timer_Poll(&p_motor->ControlTimer) == true)
	{
		p_motor->HallDebug[p_motor->CalibrationSubstateStep] = Hall_ReadSensors(&p_motor->Hall); //PhaseA starts at 1

		switch (p_motor->CalibrationSubstateStep)
		{
			case 0U :
				Phase_Polar_ActivateA(&p_motor->Phase, duty);
				break;

			case 1U :
				Hall_CalibratePhaseA(&p_motor->Hall);
				Phase_Polar_ActivateAC(&p_motor->Phase, duty);
				break;

			case 2U :
				Phase_Polar_ActivateInvC(&p_motor->Phase, duty);
				break;

			case 3U :
				Hall_CalibratePhaseInvC(&p_motor->Hall);
				Phase_Polar_ActivateBC(&p_motor->Phase, duty);
				break;

			case 4U :
				Phase_Polar_ActivateB(&p_motor->Phase, duty);
				break;

			case 5U :
				Hall_CalibratePhaseB(&p_motor->Hall);
				Phase_Polar_ActivateBA(&p_motor->Phase, duty);
				break;

			case 6U :
				Phase_Polar_ActivateInvA(&p_motor->Phase, duty);
				break;

			case 7U :
				Hall_CalibratePhaseInvA(&p_motor->Hall);
				Phase_Polar_ActivateCA(&p_motor->Phase, duty);
				break;

			case 8U :
				Phase_Polar_ActivateC(&p_motor->Phase, duty);
				break;

			case 9U :
				Hall_CalibratePhaseC(&p_motor->Hall);
				Phase_Polar_ActivateCB(&p_motor->Phase, duty);
				break;

			case 10U :
				Phase_Polar_ActivateInvB(&p_motor->Phase, duty);
				break;

			case 11U :
				Hall_CalibratePhaseInvB(&p_motor->Hall);
				Phase_Polar_ActivateAB(&p_motor->Phase, duty);
				break;

			case 12U :
				Phase_Float(&p_motor->Phase);
				isComplete = true;
				break;

			default :
				break;
		}

		p_motor->CalibrationSubstateStep++;
	}
#endif

	return isComplete;
}

/******************************************************************************/
/*! @} */
/******************************************************************************/

/******************************************************************************/
/*!
	Extern
*/
/******************************************************************************/
extern void Motor_Init(Motor_T * p_motor);
extern void Motor_InitReboot(Motor_T * p_motor);

#endif
