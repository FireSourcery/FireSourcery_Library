/**************************************************************************/
/*!
	@section LICENSE

	Copyright (C) 2021 FireSoucery / The Firebrand Forge Inc

	This file is part of FireSourcery_Library (https://github.com/FireSourcery/FireSourcery_Library).

	This program is free software: you can redistribute it and/or modify
	it under the terupdateInterval of the GNU General Public License as published by
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


#include "HAL.h" //for Digital IO Pins

#include "Peripheral/Analog/Analog.h"

#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Encoder/Encoder_Motor.h"

#include "Transducer/Phase/Phase.h"

#include "OS/StateMachine/StateMachine.h"

#include "Math/FOC.h"
#include "Math/Q/Q.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_ANALOG_ADC_CHANNEL_COUNT 			13U

/*!
	@brief Virtual channel identifiers, index into arrays containing ADC channel data
 */
typedef enum
{
	/* Must implement for BEMF sensorless */
	MOTOR_ANALOG_CHANNEL_VA,
	MOTOR_ANALOG_CHANNEL_VB,
	MOTOR_ANALOG_CHANNEL_VC,

	/* Must implement for FOC */
	MOTOR_ANALOG_CHANNEL_IA,
	MOTOR_ANALOG_CHANNEL_IB,
	MOTOR_ANALOG_CHANNEL_IC,

	/* Error checking */
	MOTOR_ANALOG_CHANNEL_VBUS, 	/* V battery, V in */
	MOTOR_ANALOG_CHANNEL_VACC,		/* V accessories */
	MOTOR_ANALOG_CHANNEL_VSENSE,	/* V analog sensors */

	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,
	MOTOR_ANALOG_CHANNEL_HEAT_PCB,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,

	/* Optional implement for analog sensor input */
	MOTOR_ANALOG_CHANNEL_THROTTLE,
	MOTOR_ANALOG_CHANNEL_BRAKE,
} Motor_AnalogChannel_T;


/*
 * All modules independently conform to same ID
 */
//typedef enum
//{
//	MOTOR_SECTOR_ID_1, //Phase AC
//} Motor_SectorId_T;
//
//typedef enum
//{
//	MOTOR_SENSOR_MODE_ENCODER,
//	MOTOR_SENSOR_MODE_BEMF,
//	MOTOR_SENSOR_MODE_OPEN_LOOP,
//	MOTOR_SENSOR_MODE_HALL,
//} Motor_SensorMode_T;
//
//typedef enum
//{
//	MOTOR_CONTROL_FOC,
//	MOTOR_CONTROL_SIX_STEP,
//} Motor_ControlMode_T;
//typedef enum
//{
//	MOTOR_FEEDBACK_OPEN,
//	MOTOR_FEEDBACK_POSITION,
//	MOTOR_FEEDBACK_CURRENT,
//	MOTOR_FEEDBACK_SPEED_VOLTAGE,
//	MOTOR_FEEDBACK_SPEED_CURRENT,

//} Motor_FeedbackMode_T;

/*
	Motor run modes
	Mode			Feedback
	Openloop		None
	Voltage			Position
	Current			Position Current
	VoltageFreq		Position 			Speed (Scalar)
	SpeedVoltage	Position 			Speed
	SpeedCurrent	Position Current	Speed

 */
//typedef struct
//{
// uint8_t Position: 1; 		//0 -> Openloop, 1 -> position feedback encoder or bemf
// uint8_t Sensorless: 1; 	//0 -> encoder, 1 -> bemf
// uint8_t Current: 1; 		//0 -> voltage, 1-> current feedback PID loop
// uint8_t Speed: 1; 			//0 -> const voltage or current, 1 -> speed feedback
// uint8_t SpeedPID: 1; 		//0 -> speed scalar, 1-> speed feedback PID loop
//} Motor_FocMode_T;

typedef enum
{
	MOTOR_FOC_MODE_OPENLOOP,
	MOTOR_FOC_MODE_CONSTANT_VOLTAGE,
	MOTOR_FOC_MODE_CONSTANT_CURRENT,
	MOTOR_FOC_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_FOC_MODE_CONSTANT_SPEED_CURRENT,
	MOTOR_FOC_MODE_SCALAR_VOLTAGE_FREQ,

//	MOTOR_FOC_MODE_SENSORLESS_CONSTANT_VOLTAGE,
//	MOTOR_FOC_MODE_SENSORLESS_CONSTANT_CURRENT,
//	MOTOR_FOC_MODE_SENSORLESS_CONSTANT_SPEED_VOLTAGE,
//	MOTOR_FOC_MODE_SENSORLESS_CONSTANT_SPEED_CURRENT,
//	MOTOR_FOC_MODE_SENSORLESS_SCALAR_VOLTAGE_FREQ,

} Motor_FocMode_T;


typedef enum
{
	MOTOR_DRIVE_MODE_FOC,
	MOTOR_DRIVE_MODE_SIX_STEP,
} Motor_DriveMode_T;


/*!
	@brief Parameters load from flash else load default
 */
typedef struct
{
	//runtime use
	qfrac16_t FocOpenLoopVq;
	qfrac16_t FocAlignVd;

	//init use only
    uint8_t PolePairs;
    uint32_t ControlFreq_Hz;

	uint32_t EncoderCountsPerRevolution;
	uint32_t EncoderDistancePerCount;
}
Motor_Parameters_T;

/*
	Compile time const Hw config
 */
typedef const struct Motor_Init_Tag
{
	HAL_ADC_T * p_AdcRegMap;
	uint8_t AdcNCount;
	uint8_t AdcMLengthBuffer;
	const uint32_t * p_AdcChannelPinMap;

	const HAL_Encoder_T * p_HalEncoder;
    uint32_t EncoderTimerCounterMax;

    const HAL_PWM_T * p_PhasePwmA;
    const HAL_PWM_T * p_PhasePwmB;
    const HAL_PWM_T * p_PhasePwmC;
	uint32_t PhasePwmMax;
//	void (*PhaseOnAB)(void * phaseData);
//	void (*PhaseOnAC)(void * phaseData);
//	void (*PhaseOnBC)(void * phaseData);
//	void (*PhaseOnBA)(void * phaseData);
//	void (*PhaseOnCA)(void * phaseData);
//	void (*PhaseOnCB)(void * phaseData);

	const HAL_Pin_T * p_PinBrake;
	const HAL_Pin_T * p_PinThrottle;
	const HAL_Pin_T * p_PinForward;
	const HAL_Pin_T * p_PinReverse;
} Motor_Init_T;


typedef struct
{
	/*
	 * Hardware Wrappers
	 */
	/* Analog_T scale 1 per motor, if Analog_T controls all adc. else if controls 1 adc, motor import HAL_start ADC to abtract adc selection */
	Analog_T Analog;
	/* Analog Channel Results Scales 1 per motor instance */
	uint16_t AnalogChannelResults[MOTOR_ANALOG_ADC_CHANNEL_COUNT];

	Encoder_T Encoder;
	Phase_T Phase;
//	Flash_T Flash; //flash/eeprom access
//	Hall_T Hall;

	//No wrapper layer for pins
//	HAL_Pin_T PinBrake;
//	HAL_Pin_T PinThrottle;
//	HAL_Pin_T PinForward;
//	HAL_Pin_T PinReverse;


	Motor_Parameters_T 	Parameters;
	Motor_FocMode_T 	FocMode;

	StateMachine_T StateMachine;
	uint32_t TimerCounter; /* Control Freq */

	/* FOC Mode */
	FOC_T		 Foc;
	//	//PID_T 	PidSpeed;

	uint32_t SpeedControlTimer;
	uint32_t SpeedControlPeriod;

	//	PID_T PidId;
	//	PID_T PidIq;
	//	qfrac16_t IdReq; /* PID setpoint */
	//	qfrac16_t IqReq;

	//	//Monitor_T 		Monitor;
	//	//BEMF_T		 Bemf;
	Linear_T UnitIa_Frac16;
	Linear_T UnitIb_Frac16;
	Linear_T UnitIc_Frac16;

	Linear_T UnitIa_Amp;
	Linear_T UnitIb_Amp;
	Linear_T UnitIc_Amp;

	//	Linear_T SpeedRamp;
	//	uint32_t SpeedRampIndex;
	uint16_t VCmd;
	uint32_t Speed_RPM;

	qfrac16_t ElectricalAngle;
	qfrac16_t MechanicalAngle;

//	/*
//	 * FOC Open Loop Settings,
//	 */
//	qfrac16_t VHzGain; //vhz scale

//	uint32_t JogSteps;
//	uint32_t StepCount;

}
Motor_T;

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
	p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
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

#endif
