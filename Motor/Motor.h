/**************************************************************************/
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
/**************************************************************************/
/**************************************************************************/
/*!
    @file 	Motor.h
    @author FireSoucery
    @brief  Motor control functions. Common to each variation
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H


#include "HAL.h"

#include "Peripheral/Pin/Pin.h"
#include "Peripheral/Analog/Analog.h"

#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Encoder/Encoder_Motor.h"

#include "Motor/Transducer/Phase/Phase.h"
#include "Motor/Transducer/Hall/Hall.h"
#include "Motor/Transducer/BEMF/BEMF.h"

#include "System/StateMachine/StateMachine.h"
#include "System/Thread/Thread.h"

#include "Motor/Math/FOC.h"

#include "Math/Q/Q.h"
#include "Math/Linear/Linear.h"
#include "Math/PID/PID.h"

#include <stdint.h>
#include <stdbool.h>





/*
 * All modules independently conform to same ID
 */
typedef enum
{
	MOTOR_SECTOR_ID_0 = 0,
	MOTOR_SECTOR_ID_1 = 1, //Phase AC
	MOTOR_SECTOR_ID_2 = 2,
	MOTOR_SECTOR_ID_3 = 3,
	MOTOR_SECTOR_ID_4 = 4,
	MOTOR_SECTOR_ID_5 = 5,
	MOTOR_SECTOR_ID_6 = 6,
	MOTOR_SECTOR_ID_7 = 7,

	MOTOR_PHASE_AC = 1,
	MOTOR_PHASE_BC = 2,
	MOTOR_PHASE_BA = 3,
	MOTOR_PHASE_CA = 4,
	MOTOR_PHASE_CB = 5,
	MOTOR_PHASE_AB = 6,
} Motor_SectorId_T;

typedef enum
{
	MOTOR_DIRECTION_CW,
	MOTOR_DIRECTION_CCW,
} Motor_Direction_T;

typedef enum
{
	MOTOR_CONTROL_MODE_FOC,
	MOTOR_CONTROL_MODE_SIX_STEP,
} Motor_CommutationMode_T;

typedef enum
{
	MOTOR_CONTROL_MODE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,		//foc only
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,	//foc only
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

//typedefstruct
//{
// uint8_t Position: 1; 	//0 -> Openloop, 1 -> position feedback encoder or bemf
// uint8_t Sensorless: 1; 	//0 -> encoder, 1 -> bemf
// uint8_t Current: 1; 		//0 -> voltage, 1-> current feedback PID loop
// uint8_t Speed: 1; 		//0 -> const voltage or current, 1 -> speed feedback
// uint8_t SpeedPID: 1; 	//0 -> speed scalar, 1-> speed feedback PID loop
//} Motor_FocMode_T;

//typedef enum
//{
//	MOTOR_STATUS_OKAY,
//	MOTOR_STATUS_ERROR_1,
//} Motor_Status_T;

#define MOTOR_ANALOG_ADC_CHANNEL_COUNT 			13U

/*!
	@brief Virtual channel identifiers, index into arrays containing ADC channel data
 */
typedef enum
{
	/* BEMF sensorless */
	MOTOR_ANALOG_CHANNEL_VBUS, 	/* V battery, V in */
	MOTOR_ANALOG_CHANNEL_VA,
	MOTOR_ANALOG_CHANNEL_VB,
	MOTOR_ANALOG_CHANNEL_VC,

	/* FOC */
	MOTOR_ANALOG_CHANNEL_IA,
	MOTOR_ANALOG_CHANNEL_IB,
	MOTOR_ANALOG_CHANNEL_IC,

	/* analog sensor input */
	MOTOR_ANALOG_CHANNEL_THROTTLE,
	MOTOR_ANALOG_CHANNEL_BRAKE,

	/* Temperature */
	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,
	MOTOR_ANALOG_CHANNEL_HEAT_PCB,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,

	/* Error checking */
	MOTOR_ANALOG_CHANNEL_VACC,		/* V accessories */
	MOTOR_ANALOG_CHANNEL_VSENSE,	/* V analog sensors */
} Motor_AnalogChannel_T;

/*!
	@brief Parameters load from flash else load default
 */
typedef struct
{
	qfrac16_t FocOpenLoopVq;
	qfrac16_t FocAlignVd;

    uint8_t PolePairs;
	uint32_t EncoderCountsPerRevolution;
	uint32_t EncoderDistancePerCount;
}
Motor_Parameters_T;

/*
	Compile time const Hw config
 */
typedef const struct Motor_Init_Tag
{
    const HAL_Phase_T HAL_PHASE;
	const HAL_Encoder_T HAL_ENCODER;
	const HAL_Hall_T HAL_HALL;
	const HAL_BEMF_T HAL_BEMF;

	const HAL_Pin_T HAL_PIN_BRAKE;
	const HAL_Pin_T HAL_PIN_THROTTLE;
	const HAL_Pin_T HAL_PIN_FORWARD;
	const HAL_Pin_T HAL_PIN_REVERSE;

	const uint32_t HALL_ENCODER_TIMER_COUNTER_MAX; //Hall mode only
	const uint32_t HALL_ENCODER_TIMER_COUNTER_FREQ; //Hall mode only
	const uint32_t ENCODER_ANGLE_RES_BITS;
	const uint32_t MOTOR_PWM_FREQ;
	const uint32_t PHASE_PWM_PERIOD;
	const uint32_t LINEAR_V_ABC_R1;
	const uint32_t LINEAR_V_ABC_R2;
	const uint32_t LINEAR_V_BUS_R1;
	const uint32_t LINEAR_V_BUS_R2;
} Motor_Init_T;


typedef struct
{
	//todo alarm and thermistor


	// No wrapper layer for pins
	const HAL_Pin_T * p_PinBrake;
	const HAL_Pin_T * p_PinThrottle;
	const HAL_Pin_T * p_PinForward;
	const HAL_Pin_T * p_PinReverse;

	/*
	 * Hardware Wrappers
	 */
	/* if Analog_T abstracted to controls all adc.(nfixed and nmultimuxed modes). Analog_T scale 1 per motor, can be controlled within motor module */
	//	Analog_T Analog;

	/* Analog Channel Results Scales 1 per motor instance */
	volatile uint16_t AnalogChannelResults[MOTOR_ANALOG_ADC_CHANNEL_COUNT];

	Encoder_T Encoder;
	Phase_T Phase;
	Hall_T Hall;
	BEMF_T Bemf;
	//	Flash_T Flash; // flash/eeprom access

	FOC_T Foc;
	qangle16_t ElectricalAngle;
//	qangle16_t MechanicalAngle;
	uint32_t InterpolatedAngleIndex;

	/* Mode selection. Substates */
	Motor_Parameters_T 		Parameters;
	Motor_Direction_T 		Direction;
	Motor_ControlMode_T 	ControlMode;
	Motor_SensorMode_T 		SensorMode;
	Motor_CommutationMode_T CommutationMode;

	StateMachine_T StateMachine;
	uint32_t ControlTimerBase;	 /* Control Freq */
	Thread_T ControlTimerThread;

	Thread_T Timer1Ms;

	//	//PID_T PidSpeed;
	uint32_t SpeedControlTimer;
	uint32_t SpeedControlPeriod;
	//	PID_T PidId;
	//	PID_T PidIq;
	//	qfrac16_t IdReq; /* PID setpoint */
	//	qfrac16_t IqReq;

	uint32_t CommutationPeriodCmd; //CommutationPeriodCmd
	uint32_t OpenLoopZcdCount;
	//	qfrac16_t OpenLoopVHzGain; //vhz scale

	Linear_T UnitIa; //Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;

	Linear_T UnitVabc; //Bemf
	Linear_T UnitVBus;

	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	Linear_T Ramp;
	uint32_t RampIndex;

	uint32_t SpeedReq_RPM; ///req or setpoint
	uint32_t SpeedFeedback_RPM;


	uint16_t VReq; //User input, set point

	uint16_t VCtrl; // same as PWM duty



	/*
	 * SixStep Settings
	 */
	Motor_SectorId_T NextSector; //for 6 step openloop/sensorless

//	uint32_t JogSteps;
//	uint32_t StepCount;

	/* UI report */
	uint16_t VBus;
}
Motor_T;


static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

static inline void Motor_BrakeDynamic(Motor_T * p_motor)
{
	Phase_Short(&p_motor->Phase); //set pwm = 0
}

static inline void Motor_BrakePlugging(Motor_T * p_motor)
{
	Motor_SetDirectionReverse(p_motor);
}


/*
 * Speed PID Feedback Loop
 *
 * Inputs by pointer:
 * 	p_motor->Speed_RPM
 * 	p_motor->SpeedReq_RPM
 *
 * Output by pointer:
 * 	FOC Mode - sets iqReq, Vcmd,
 * 	SixStep -
 *
 * generalize to scalar feedback variable e.g. temperature
 *
 */
static inline void Motor_ProcSpeedLoop(Motor_T * p_motor)
{
	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
//	//p_motor->Speed_RPM = Filter_MovAvg(p_motor->Speed_RPM, coef, coef);
//	PID_Proc(&p_motor->PidSpeed);
}


static inline void Motor_PollSpeedLoop(Motor_T * p_motor)
{
	if (p_motor->SpeedControlTimer > p_motor->SpeedControlPeriod)
	{
		Motor_ProcSpeedLoop(p_motor);
		p_motor->SpeedControlTimer = 0;
	}
	else
	{
		p_motor->SpeedControlTimer++;
	}
}

/*
 * changes/set occurs during motor inactive time
 */
//static inline void Motor_AssignSpeedLoop(Motor_T * p_motor, uint32_t * p_setPoint, uint32_t * p_feedback, uint32_t * p_controlSignal)
//{
//
//}

extern void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInitStruct);

extern void Motor_StartAlign(Motor_T * p_motor);

#endif

//Throttle is VUserSetPoint
//poll throttle or set from outter module functions?
//static inline void Motor_GetThrottle(Motor_T *p_motor)
//{
//	switch (p_motor->InputMode)
//	{
//	case MOTOR_SHELL:
//		p_motor->Throttle =
//		break;
//
//	case MOTOR_ANALOG:
//		p_motor->Throttle = Linear_ADC_CalcUnsignedFraction16(&p_motor->UnitThrottle, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_THROTTLE]);
//		break;
//
//	case MOTOR_COM:
//		p_motor->Throttle = SerialApp_GetThrottle();
//		break;
//
//	default:
//		break;
//	}
//}

//static inline void Motor_ReadSensor(Motor_T *p_motor)
//{
//	switch (p_motor->SensorMode)
//	{
//	case MOTOR_SENSOR_MODE_ENCODER:
//
//		break;
//
//	case MOTOR_SENSOR_MODE_BEMF:
//
//		break;
//
//	case MOTOR_SENSOR_MODE_HALL:
//
//		break;
//
//	default:
//		break;
//	}
//}
