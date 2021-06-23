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


#include "HAL_Motor.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"
#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Flash/Flash.h"

#include "Transducer/Encoder/Encoder.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder_IO.h"

#include "Motor/Transducer/Phase/Phase.h"
#include "Motor/Transducer/Hall/Hall.h"
#include "Motor/Transducer/BEMF/BEMF.h"

#include "System/StateMachine/StateMachine.h"
#include "System/Thread/Thread.h"

#include "Motor/Math/FOC.h"

#include "Math/Q/Q.h"
#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"
#include "Math/PID/PID.h"

#include <stdint.h>
#include <stdbool.h>



/*
 * Runtime Variable
 */

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

	MOTOR_SECTOR_ERROR_000 = 0,
	MOTOR_SECTOR_ERROR_111 = 7,

	MOTOR_PHASE_AC = 1,
	MOTOR_PHASE_BC = 2,
	MOTOR_PHASE_BA = 3,
	MOTOR_PHASE_CA = 4,
	MOTOR_PHASE_CB = 5,
	MOTOR_PHASE_AB = 6,
} Motor_SectorId_T;

/*
 * Commutation Direction
 */
typedef enum
{
//	MOTOR_DIRECTION_N,
	MOTOR_DIRECTION_CW = 0,
	MOTOR_DIRECTION_CCW = 1,
} Motor_Direction_T;

//typedef enum
//{
//	MOTOR_FORWARD_IS_CW,
//	MOTOR_FORWARD_IS_CCW,
//} Motor_DirectionMode_T;

//currently active
typedef enum
{
	MOTOR_POSITION_FEEDBACK_OPEN_LOOP,
	MOTOR_POSITION_FEEDBACK_HALL,
	MOTOR_POSITION_FEEDBACK_ENCODER,
	MOTOR_POSITION_FEEDBACK_BEMF,
} Motor_PositionFeedback_T; //bemf/openloop state

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

typedef enum
{
	MOTOR_INPUT_CMD_BRAKE,
} Motor_InputCmd_T;

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
	MOTOR_CONTROL_MODE_OPEN_LOOP,
	MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE,
	MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE,
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,		//foc only
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,	//foc only
} Motor_ControlMode_T; //ControlVariableMode

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
	MOTOR_INPUT_MODE_ANALOG,
	MOTOR_INPUT_MODE_SERIAL,
	MOTOR_INPUT_MODE_CAN,
} Motor_InputMode_T;

typedef enum
{
	MOTOR_BRAKE_MODE_PASSIVE,
	//	MOTOR_BRAKE_MODE_ACTIVE,
	MOTOR_BRAKE_MODE_SCALAR,
	MOTOR_BRAKE_MODE_REGEN_OPTIMAL,
	MOTOR_BRAKE_MODE_REGEN_PROPRTIONAL,
	MOTOR_BRAKE_MODE_REGEN_SCALAR,

} Motor_BrakeMode_T;




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




#define MOTOR_ANALOG_ADC_CHANNEL_COUNT 			14U

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
	@brief Motor Parameters
	Runtime variable configuration

	load from flash
 */
typedef  __attribute__ ((aligned (4U))) struct
{
    uint8_t PolePairs;
	uint32_t EncoderCountsPerRevolution;
	uint32_t EncoderDistancePerCount;

	uint16_t AdcRefBrake;		//max adc read value for brake
	uint16_t AdcRefThrottle;

	uint16_t SpeedRef; //max speed for throttle calibration
	uint16_t VBusRef;


	uint16_t OpenLoopZcdTransition;

	uint8_t BrakeCoeffcient;

	uint32_t SpeedControlPeriod;



	uint8_t HallVirtualSensorAMap;
	uint8_t HallVirtualSensorBMap;
	uint8_t HallVirtualSensorCMap;
	uint8_t HallVirtualSensorInvAMap;
	uint8_t HallVirtualSensorInvBMap;
	uint8_t HallVirtualSensorInvCMap;

	Motor_SensorMode_T 			SensorMode;
	Motor_AlignMode_T 			AlignMode;
	Motor_InputMode_T 			InputMode;
	Motor_CommutationMode_T 	CommutationMode;
	Motor_ControlMode_T 		ControlMode;
	Motor_BrakeMode_T 			BrakeMode;
	Phase_Mode_T				PhasePwmMode;

	qfrac16_t FocOpenLoopVq;
	qfrac16_t FocAlignVd;
	//	qfrac16_t OpenLoopVHzGain; //vhz scale
}
Motor_Parameters_T;

/*
	@brief Motor Init
	Compile time const configuration
 */
typedef const struct Motor_Init_Tag
{
    const HAL_Phase_T HAL_PHASE;
	const HAL_Encoder_T HAL_ENCODER;
	const HAL_Hall_T HAL_HALL;
	const HAL_BEMF_T HAL_BEMF;
	const HAL_Flash_T HAL_FLASH;

	const HAL_Pin_T HAL_PIN_BRAKE;
	const HAL_Pin_T HAL_PIN_THROTTLE;
	const HAL_Pin_T HAL_PIN_FORWARD;
	const HAL_Pin_T HAL_PIN_REVERSE;

	const uint32_t MOTOR_PWM_FREQ;
	const uint32_t PHASE_PWM_PERIOD;

	const uint32_t HALL_ENCODER_TIMER_COUNTER_MAX; //Hall mode only
	const uint32_t HALL_ENCODER_TIMER_COUNTER_FREQ; //Hall mode only
//	const uint32_t ENCODER_ANGLE_RES_BITS;

	const uint32_t LINEAR_V_ABC_R1;
	const uint32_t LINEAR_V_ABC_R2;
	const uint32_t LINEAR_V_BUS_R1;
	const uint32_t LINEAR_V_BUS_R2;

	const uint32_t LINEAR_V_ADC_VREF;
	const uint32_t LINEAR_V_ADC_BITS;

	uint8_t * const P_EEPROM; 	//or flash partition struct
}
Motor_Init_T;


typedef struct
{

	/* if Analog_T abstracted to controls all adc.(nfixed and nmultimuxed modes). Analog_T scale 1 per motor, can be controlled within motor module */
	//	Analog_T Analog;

	// throttle and brake may need to be pointers, when 1 adc channel controls multiple motors
	volatile uint16_t AnalogChannelResults[MOTOR_ANALOG_ADC_CHANNEL_COUNT];

	const Motor_Init_T * p_Init;			//compile time const
	Motor_Parameters_T	Parameters;			//Programmable parameters, runtime variable load from eeprom
	//	uint32_t * 	p_Eeprom;
	//	Flash_Partition_T  p_FlashPartition;

	/*
	 * Hardware Wrappers
	 */
	Encoder_T Encoder;
	Phase_T Phase;
	Hall_T Hall;
	BEMF_T Bemf;

	/*
	 * SW config
	 */
	StateMachine_T StateMachine;

	Linear_T UnitVBus;
	Linear_T UnitVabc; //Bemf
	Linear_T UnitIa; //Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;

	volatile uint32_t ControlTimerBase;	 /* Control Freq, calibration, commutation, angle control */
	Thread_T ControlTimerThread;
	volatile uint32_t MillisTimerBase;	 /* Millis, UI */
	Thread_T MillisTimerThread;
	Thread_T SecondsTimerThread;

	Linear_T Ramp;
	volatile uint32_t RampIndex;


	Linear_T OpenLoopRamp;
	volatile uint32_t OpenLoopRampIndex;

	//	//PID_T PidSpeed;
	//	PID_T PidId;
	//	PID_T PidIq;
	//	volatile qfrac16_t IdReq; /* PID setpoint */
	//	volatile qfrac16_t IqReq;

	/*
	 * Runtime vars
	 */
	volatile Motor_Direction_T 			Direction; //active spin direction
	volatile Motor_Direction_T 			DirectionInput; ///buffered
	volatile bool IsDirectionNeutral;
	volatile Motor_PositionFeedback_T 	PositionFeedback;

	volatile bool IsThrottleRelease;

	volatile uint16_t UserCmd; 		// from user input throttle or brake, VReq/IReq/SpeedReq
	volatile uint16_t UserCmdPrev;
//	volatile int32_t UserCmdDelta;

	volatile uint16_t RampCmd;
	volatile uint16_t VPwm; 		//VPwm for SixStep Control
	volatile uint16_t SpeedFeedback_RPM;
	volatile uint16_t Openloop_RPM;

	volatile uint16_t RampCmdTemp;

	volatile FOC_T Foc;
	volatile qangle16_t ElectricalAngle;
//	volatile qangle16_t MechanicalAngle;

	volatile qangle16_t ElectricalDeltaPrev;

	volatile uint32_t InterpolatedAngleIndex;
	volatile qangle16_t HallAngle;

	volatile Motor_SectorId_T NextSector; //for 6 step openloop/sensorless
	volatile Motor_SectorId_T CommutationSector; //for 6 step openloop/sensorless
	volatile uint32_t CommutationPeriodCmd; //CommutationPeriodCtrl openloop and backemf
	volatile uint32_t OpenLoopZcdCount;


	//move to UI
	//UI - change to 1 per all motor?
	//todo alarm and thermistor
	Debounce_T PinBrake;
	Debounce_T PinThrottle;
	Debounce_T PinForward;
	Debounce_T PinReverse;

	Linear_T UnitThrottle;
	Linear_T UnitBrake;

	volatile bool InputSwitchBrake;
	volatile bool InputSwitchThrottle;
	volatile bool InputSwitchForward;
	volatile bool InputSwitchReverse;
	volatile bool InputSwitchNeutral;
	volatile uint16_t InputValueThrottle; // serial or adc
	volatile uint16_t InputValueThrottlePrev;
	volatile uint16_t InputValueBrake;
	volatile uint16_t InputValueBrakePrev;

//	uint32_t JogSteps;
//	uint32_t StepCount;

	/* UI report */
//	volatile uint16_t VBus;
}
Motor_T;


//per motor
static inline void Motor_SetUserCmd(Motor_T * p_motor, uint16_t userCommand)
{
	p_motor->UserCmdPrev = p_motor->UserCmd;
	p_motor->UserCmd = userCommand;
}

static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

static inline uint32_t Motor_GetSpeed(Motor_T * p_motor)
{
//	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	return p_motor->SpeedFeedback_RPM;
}

static inline void Motor_ProcSpeed(Motor_T * p_motor)
{
	if(p_motor->Parameters.SensorMode != MOTOR_SENSOR_MODE_OPEN_LOOP)
	{
		p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	}
//	else
//	{
//		p_motor->SpeedFeedback_RPM = p_motor->Openloop_RPM;
//	}
}

static inline void Motor_PollStop(Motor_T * p_motor)
{
	if (p_motor->Parameters.CommutationMode == MOTOR_COMMUTATION_MODE_SIX_STEP || p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_HALL)
	{
		//Capture deltaT mode need
		if (Encoder_PollDeltaTStop_IO(&p_motor->Encoder))
		{
			p_motor->SpeedFeedback_RPM = 0;
		}
	}
	else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP)
	{
		if ((p_motor->UserCmd == 0U)) //todo fix
		{
			if(p_motor->UserCmdPrev > 0U)
			{
//
//				Thread_RestartTimer(&StopPollTimer);
//
//			}
//			else if(Thread_PollTimerCompletePeriodic(&StopPollTimer))
//			{
				p_motor->SpeedFeedback_RPM = 0;
			}
		}
	}
	else //other modes covered by always capture DeltaD
	{

	}
}


static inline void Motor_ProcRamp(Motor_T * p_motor) // input voltage/speed cmd
{
	p_motor->RampCmd = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, 1U);
}


//proc ramp update ~millis
static inline void Motor_SetRampAccelerate(Motor_T * p_motor, uint16_t acceration)
{

	if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE || p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{

	}
	else if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT)
	{

	}
	else
	{
		//Ramp to throttle over 1s
//		if (p_motor->UserCmd > p_motor->UserCmdPrev)
		{
			Linear_Ramp_InitAcceleration(&p_motor->Ramp, 20000U, p_motor->UserCmdPrev, p_motor->UserCmd, (int32_t)p_motor->UserCmd - (int32_t)p_motor->UserCmdPrev);
		}

		//todo match  bemf
	}
	p_motor->RampIndex = 0;
}

static inline void Motor_SetRampDecelerate(Motor_T * p_motor, uint16_t deceleration)
{
	if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE || p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
	{
		//ramp unit is speed
	}
	else if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT)
	{

	}
	else //ramp unit is pwm voltage
	{
		//Decel by Brake per 1 s
		//todo match  bemf
		Linear_Ramp_InitAcceleration(&p_motor->Ramp, 20000U, p_motor->VPwm, 0, -(int32_t)p_motor->UserCmd);

	}
	p_motor->RampIndex = 0;
}


static inline void Motor_SetOpenLoopSpeedRamp(Motor_T * p_motor, uint16_t acceration)
{
	int32_t speedNew;
	int32_t speedPrev;
	int32_t delta;

	if (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_OPEN_LOOP)
	{
		speedNew = p_motor->UserCmd / 10U;
		speedPrev = Motor_GetSpeed(p_motor);

		delta = speedNew - speedPrev;

		//		Linear_Ramp_InitMillis(&p_motor->OpenLoopRamp, 20000U, Motor_GetSpeed(p_motor), p_motor->UserCmd/6, 1000U);

		Linear_Ramp_InitAcceleration(&p_motor->OpenLoopRamp, 20000U, speedPrev, speedNew, delta);

		p_motor->OpenLoopRampIndex = 0U;
	}

}
//static inline void Motor_SetRamp(Motor_T * p_motor)
//{
//	if (p_motor->InputSwitchBrake == true)
//	{
//		Motor_SetRampDecel(p_motor);
//	}
//	else
//	{
//		Motor_SetRampAccel(p_motor);
//	}
//}



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
static inline void Motor_ProcSpeedFeedback(Motor_T * p_motor)
{
	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
//	//p_motor->SpeedFeedback_RPM = Filter_MovAvg(p_motor->Speed_RPM, coef, coef);
//	PID_Proc(&p_motor->PidSpeed, p_motor->SpeedFeedback_RPM, setpoint);
}

static inline void Motor_PollSpeedFeedback(Motor_T * p_motor)
{
	if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
	{
		 Motor_ProcSpeedFeedback(p_motor);
	}
}

//static inline void Motor_SetControlVariable(Motor_T * p_motor, mode, cmd) // input voltage/speed cmd

static inline void Motor_ProcControlVariable(Motor_T * p_motor) // input voltage/speed cmd
{

	switch (p_motor->Parameters.ControlMode)
	{

	case MOTOR_CONTROL_MODE_OPEN_LOOP:
		p_motor->VPwm = p_motor->UserCmd/2U;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE:
		p_motor->VPwm = p_motor->RampCmd;
		break;

	case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ:
		//p_motor->VPwm = p_motor->RampCmd * p_motor->SpeedReq_RPM * p_motor->VRpmGain;
		break;

	case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE:
		if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
		{
			 Motor_ProcSpeedFeedback(p_motor); //get from PID
		}
		break;

	default:
		break;
	}

}



//passive "control"
static inline void Motor_ObserveSensors(Motor_T * p_motor)
{

//	switch (p_motor->Parameters.SensorMode)
//	{
////	case MOTOR_SENSOR_MODE_NONE:
//
//	case MOTOR_SENSOR_MODE_BEMF:
//		break;
//
//	case MOTOR_SENSOR_MODE_HALL:
//		if(Hall_PollEdge_IO(&p_motor->Hall))
//		{
//			BEMF_SetNewCycle_IO(&p_motor->Bemf); //always observe bemf
//			Encoder_CaptureDeltaT_IO(&p_motor->Encoder);
//			Encoder_CaptureExtendedDeltaT_IO(&p_motor->Encoder);
//	//		p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder); //more responsive to always calc speed
//
//		}
//
//	default:
//		break;
//	}

}


static inline bool Motor_IsCurrentFeedbackMode(Motor_T * p_motor)
{
	return  (
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) ||
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
			) ? true : false;
}


static inline bool Motor_PollToggleDirectionError(Motor_T * p_motor)
{
	//proc direction
	//		 if (p_motor->Direction != p_motor->InputDirection)//direction reversed
	//		{
	//			Blinky_SetOnTime(&p_Motor->Alarm, 1000)
	//		}
}


//move to ui?

static inline bool Motor_PollToggleDirectionUpdate(Motor_T * p_motor)
{
	if (p_motor->Direction != p_motor->DirectionInput)
	{
		Motor_SetDirection(p_motor->DirectionInput);
	}
}

static inline bool Motor_PollStopToSpin(Motor_T * p_motor)
{
	bool transition = true;

	transition &= (p_motor->InputSwitchBrake == false) || ((p_motor->InputSwitchBrake == true) && (Motor_GetSpeed(p_motor) > 0));
	transition &= (p_motor->IsDirectionNeutral == false);
	transition &= (p_motor->UserCmd > 0U);
	transition &= (p_motor->IsThrottleRelease == false);

	return transition;
}

static inline bool Motor_PollSpinToFreewheel(Motor_T * p_motor)
{
	bool transition = false;

	transition |= (p_motor->IsDirectionNeutral == true);
	transition |= (p_motor->UserCmd == 0); 	//brake 0 and throttle 0
	transition |= (p_motor->IsThrottleRelease == true);
	transition |= ((p_motor->InputSwitchBrake == true) && (p_motor->Parameters.BrakeMode == MOTOR_BRAKE_MODE_PASSIVE));

	return transition;
}

static inline bool Motor_PollFreewheelToSpin(Motor_T * p_motor)
{
	bool transition = true;

	transition &= (p_motor->InputSwitchBrake == false) || ((p_motor->InputSwitchBrake == true) && (p_motor->Parameters.BrakeMode != MOTOR_BRAKE_MODE_PASSIVE) && (Motor_GetSpeed(p_motor) > 0));
	transition &= (p_motor->UserCmd > 0U);
	transition &= (p_motor->IsDirectionNeutral == false);
	transition &= (p_motor->IsThrottleRelease == false);

	return transition;
}





extern void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInitStruct);
extern void Motor_StartAlign(Motor_T * p_motor);
extern void Motor_OnBlock(Motor_T * p_motor);


#endif
