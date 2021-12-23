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
	//	MOTOR_DIRECTION_N,
} Motor_Direction_T;

//typedef enum
//{
//	MOTOR_FORWARD_IS_CW,
//	MOTOR_FORWARD_IS_CCW,
//} Motor_DirectionMode_T;

//typedef enum
//{
//	MOTOR_BEMF_STATE_OPEN_LOOP,
//	MOTOR_BEMF_STATE_USE_ZCD,
//} Motor_BemfState_T;

//typedef enum
//{
//	MOTOR_STATUS_OKAY,
//	MOTOR_STATUS_ERROR_1,
//} Motor_Status_T;

/*
 * User Config
 */
typedef enum
{
	MOTOR_COMMUTATION_MODE_FOC,
	MOTOR_COMMUTATION_MODE_SIX_STEP,
} Motor_CommutationMode_T;

typedef enum
{
//	MOTOR_CONTROL_MODE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,
} Motor_ControlMode_T; //FeedbackControlVariableMode

typedef enum
{
	MOTOR_SENSOR_MODE_OPEN_LOOP,
	MOTOR_SENSOR_MODE_HALL,
	MOTOR_SENSOR_MODE_ENCODER,
	MOTOR_SENSOR_MODE_BEMF,
} Motor_SensorMode_T;

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
	//	MOTOR_BRAKE_MODE_ACTIVE,
	MOTOR_BRAKE_MODE_SCALAR,
	MOTOR_BRAKE_MODE_REGEN_OPTIMAL,
	MOTOR_BRAKE_MODE_REGEN_PROPRTIONAL,
	MOTOR_BRAKE_MODE_REGEN_SCALAR,
} Motor_BrakeMode_T;

typedef enum
{
	MOTOR_CALIBRATION_STATE_ADC,
	MOTOR_CALIBRATION_STATE_HALL,
	MOTOR_CALIBRATION_STATE_ENCODER,
} Motor_CalibrationState_T;


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

//typedefstruct
//{
// uint8_t Position: 1; 	//0 -> Openloop, 1 -> position feedback encoder or bemf
// uint8_t Sensorless: 1; 	//0 -> encoder, 1 -> bemf
// uint8_t Current: 1; 		//0 -> voltage, 1-> current feedback PID loop
// uint8_t Speed: 1; 		//0 -> const voltage or current, 1 -> speed feedback
// uint8_t SpeedPID: 1; 	//0 -> speed scalar, 1-> speed feedback PID loop
//} Motor_ConfigMode_T;

//#ifdef CONFIG_MOTOR_ADC_8
//	typedef uint16_t adc_t;
//#elif defined(CONFIG_MOTOR_ADC_16)
//	typedef uint16_t adc_t;
//#endif
//
//typedef const struct
//{
//	volatile const adc_t * const P_VBUS_ADCU;
//	volatile const adc_t * const P_VA_ADCU;
//	volatile const adc_t * const P_VB_ADCU;
//	volatile const adc_t * const P_VC_ADCU;
//	volatile const adc_t * const P_IA_ADCU;
//	volatile const adc_t * const P_IB_ADCU;
//	volatile const adc_t * const P_IC_ADCU;
//	volatile const adc_t * const P_HEAT_MOTOR_ADCU;
//	volatile const adc_t * const P_HEAT_MOSFETS_ADCU; //if per motor mosfet sensor is implemented
//}
//Motor_AdcMap_T;

//shared between instances
//typedef const struct
//{
//	volatile const adc_t * const P_VBUS_ADCU;
//	volatile const adc_t * const P_VACC_ADCU;
//	volatile const adc_t * const P_VSENSE_ADCU;
//	volatile const adc_t * const P_HEAT_PCB_ADCU;
//	volatile const adc_t * const P_HEAT_MOSFETS_H_ADCU;
//	volatile const adc_t * const P_HEAT_MOSFETS_L_ADCU;
//	volatile const adc_t * const P_THROTTLE_ADCU;
//	volatile const adc_t * const P_BRAKE_ADCU;
//}
//Motor_AdcMapCommon_T;

/*!
	@brief Motor Parameters
	Runtime variable configuration

	load from flash
 */
typedef struct __attribute__ ((aligned (4U)))
{
    uint8_t PolePairs;


//	uint16_t AdcRefBrake;		//max adc read value for brake
//	uint16_t AdcRefThrottle;

	uint16_t SpeedRef; //max speed for throttle calibration
	uint16_t VBusRef;

	uint16_t OpenLoopZcdTransition;

	uint8_t BrakeCoeffcient;

	uint32_t SpeedControlPeriod;


	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_ControlMode_T 		ControlMode;
	Motor_BrakeMode_T 			BrakeMode;
	Motor_AlignMode_T 			AlignMode;

	Phase_Mode_T				PhasePwmMode;
	BEMF_SampleMode_T			BemfSampleMode;

//	DirectionCalibration_T

//	Motor_InputMode_T 			InputMode; //UserMode

	qfrac16_t FocOpenLoopVq;
	qfrac16_t FocAlignVd;
	//	qfrac16_t OpenLoopVHzGain; //vhz scale

	uint16_t Imax_Amp;
	uint16_t IaZero_ADCU;
	uint16_t IbZero_ADCU;
	uint16_t IcZero_ADCU;


	//todo account for copy to ram twice
	Hall_SensorsId_T SensorsTable[HALL_SENSORS_TABLE_LENGTH];
	uint32_t EncoderCountsPerRevolution;
	uint32_t EncoderDistancePerCount;
	bool EncoderIsQuadratureModeEnabled;
	bool EncoderIsALeadBPositive;
}
Motor_Parameters_T;

typedef struct __attribute__ ((aligned (4U)))
{

}
Motor_ParametersCommon_T;

typedef struct
{
	//PhaseMode mode
	//bemf sample
}
Motor_SixStep_Sensorless_T;

/*
	@brief Motor Init
	Compile time const configuration
 */
typedef const struct Motor_Init_Tag
{
	//const using fixed resistor values
	const Linear_T UNIT_V_POS;
	const Linear_T UNIT_V_ABC; 	//Bemf V and mV conversion

	AnalogN_T * const P_ANALOG_N;

	const AnalogN_Conversion_T CONVERSION_BEMF_A;
	const AnalogN_Conversion_T CONVERSION_BEMF_A_REPEAT;
	const AnalogN_Conversion_T CONVERSION_BEMF_B;
	const AnalogN_Conversion_T CONVERSION_BEMF_B_REPEAT;
	const AnalogN_Conversion_T CONVERSION_BEMF_C;
	const AnalogN_Conversion_T CONVERSION_BEMF_C_REPEAT;
	const AnalogN_Conversion_T CONVERSION_BEMF_REMAINDER;
	const AnalogN_Conversion_T CONVERSION_FOC_IABC;
	const AnalogN_Conversion_T CONVERSION_FOC_REMAINDER;
	const AnalogN_Conversion_T CONVERSION_IDLE;

 	const Motor_Parameters_T * const P_PARAMETERS; //eeprom copy
}
Motor_Config_T;

typedef struct
{
 	const Motor_Config_T CONFIG;		//compile time const, unique per motor

 	Motor_Parameters_T Parameters; 		//runtime variable load from eeprom

 	uint16_t AnalogResults[MOTOR_ANALOG_CHANNEL_MOTOR_COUNT];
 	uint16_t * p_AnalogResultsCommon;

 	AnalogN_AdcFlags_T SignalBufferBemfA;
 	AnalogN_AdcFlags_T SignalBufferBemfARepeat;
 	AnalogN_AdcFlags_T SignalBufferBemfB;
 	AnalogN_AdcFlags_T SignalBufferBemfC;
 	AnalogN_AdcFlags_T SignalBufferRemainder;
 	AnalogN_AdcFlags_T SignalBufferFocIabc;
 	AnalogN_AdcFlags_T SignalBufferFocRemainder;
 	AnalogN_AdcFlags_T SignalBufferIdle;

	/*
	 * Hardware Wrappers
	 */
	Encoder_T Encoder;
	Phase_T Phase;
	Hall_T Hall;
	BEMF_T Bemf;

//	Blinky_T * p_Buzzer;
	Thermistor_T Thermistor;

	StateMachine_T StateMachine;

	/*
	 * SW config
	 */
	//not const due to adc calibration
	Linear_T UnitIa; 	//Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;

	/******************************************************************************/
	/*
	 * Runtime vars
	 */
	/******************************************************************************/
	volatile uint32_t ControlTimerBase;	 /* Control Freq ~ 20kHz, calibration, commutation, angle control */
	Timer_T ControlTimer;

//	volatile uint32_t MillisTimerBase;	 /* Millis, UI */
	Timer_T MillisTimer;
	Timer_T SecondsTimer;

	Motor_Direction_T Direction; 		//active spin direction
	Motor_Direction_T DirectionInput; ///buffered
	//	volatile bool IsDirectionNeutral;
	bool IsActiveControl;

	/*
	 * Control Variable
	 */
	uint16_t UserCmd; 			//Fraction16 SetPoint pre ramp, VReq/IReq/SpeedReq,  user input throttle or brake,
	uint16_t UserCmdPrev;
	Linear_T Ramp;
	uint32_t RampIndex;
	uint16_t RampCmd;				//SetPoint after ramp
	uint16_t VPwm; 				//Control Variable

	/* Feedback */
	uint16_t Speed_RPM;	//Feedback Variable
	uint32_t IBus_ADCU; 			//phase positive current
	uint32_t IBus_Frac16;			//0-65535

	//	PID_T PidSpeed;
	//	PID_T PidId;
	//	PID_T PidIq;
	//	volatile qfrac16_t IdReq; /* PID setpoint */
	//	volatile qfrac16_t IqReq;

	/*
	 * Open-loop
	 */
	Linear_T OpenLoopRamp;
	uint32_t OpenLoopRampIndex;
	uint32_t OpenLoopCommutationPeriod;
	uint16_t OpenLoopSpeed_RPM;
	uint16_t OpenLoopVPwm;

	/*
	 * FOC
	 */
	FOC_T Foc;
	qangle16_t ElectricalAngle; //same as foc.theta
	//	volatile qangle16_t MechanicalAngle;

	/* interpolated angle */
	qangle16_t HallAngle;
	qangle16_t ElectricalDeltaPrev;
	uint32_t InterpolatedAngleIndex;

	/*
	 * Six-Step
	 */
	Motor_SectorId_T NextPhase;		 //for 6 step openloop/sensorless
	Motor_SectorId_T CommutationPhase;	 //for 6 step openloop/sensorless
//	Analog_Conversion_T * p_BemfConversionActive;

	//	volatile bool IsStartUp;			//bemf substate

//	AnalogN_Conversion_T * p_BemfConversionActive ;


	Motor_CalibrationState_T CalibrationState; /* Substate, selection for calibration */
	uint8_t CalibrationSubstateStep;

//	uint32_t JogSteps;
//	uint32_t StepCount;
}
Motor_T;

/*******************************************************************************/
/*!
	Common
*/
/*******************************************************************************/
static inline void Motor_SetZero(Motor_T * p_motor)
{
//	Encoder_Reset(&p_motor->Encoder);
	p_motor->Speed_RPM = 0U;
	p_motor->VPwm = 0;
}

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

static inline void Motor_PollStop(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true)
		{
			p_motor->Speed_RPM = 0U; //todo stop flag
		}
	}
//	else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP)
//	{
//		if ((p_motor->UserCmd == 0U)) //no braking/freewheel in openloop
//		{
////			if(p_motor->UserCmdPrev > 0U)
////			{
////				p_motor->Speed_RPM = 0U;
////			}
//		}
//	}
//	else //other modes covered by always capture DeltaD
//	{
//
//	}
}

static inline void Motor_CaptureSpeed(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode != MOTOR_SENSOR_MODE_OPEN_LOOP)
	{
		p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	}
//	else
//	{
//		p_motor->Speed_RPM = p_motor->OpenLoopSpeed_RPM;
//	}
}

static inline void Motor_PollAnalogStartAll(Motor_T * p_motor)
{
	if (Timer_Poll(&p_motor->MillisTimer) == true)
	{
//		MotorAnalog_EnqueueIdle(p_motor);
	}
}

/*******************************************************************************/
/*!
	Wrappers
*/
/*******************************************************************************/
static inline uint32_t Motor_GetSpeed(Motor_T * p_motor)
{
//	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	return p_motor->Speed_RPM;
}

static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

/*******************************************************************************/
/*!
	Extern
*/
/*******************************************************************************/
// todo monitor heat functions
extern void Motor_Init(Motor_T * p_motor);

/*
 * Referenced by Analog
 */
void Motor_CaptureIBusIa(Motor_T * p_motor);
void Motor_CaptureIBusIb(Motor_T * p_motor);
void Motor_CaptureIBusIc(Motor_T * p_motor);

/*
 * Referenced by State Machine
 */
extern void Motor_SetCalibrationStateAdc(Motor_T * p_motor);
extern void Motor_SetCalibrationStateHall(Motor_T * p_motor);
extern void Motor_SetCalibrationStateEncoder(Motor_T * p_motor);
extern bool Motor_CalibrateAdc(Motor_T * p_motor);
extern bool Motor_CalibrateHall(Motor_T * p_motor);
extern bool Motor_CalibrateEncoder(Motor_T * p_motor);

//extern void Motor_StartAlign(Motor_T * p_motor);

#endif
