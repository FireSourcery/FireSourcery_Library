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


//#include "HAL_Motor.h"

#include "Config.h"

#include "Peripheral/Pin/Debounce.h"
#include "Peripheral/Pin/Pin.h"
#include "Peripheral/Analog/Analog.h"
#include "Peripheral/Flash/Flash.h"

#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_DeltaD.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"


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
	MOTOR_CONTROL_MODE_CONSTANT_CURRENT,
	MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT,
} Motor_ControlMode_T; //ControlFeedbackVariableMode

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



	/* Temperature */
	MOTOR_ANALOG_CHANNEL_HEAT_MOTOR,
	MOTOR_ANALOG_CHANNEL_HEAT_MOSFETS,

	MOTOR_ANALOG_CHANNEL_HEAT_PCB,

	/* Error checking */
	MOTOR_ANALOG_CHANNEL_VACC,		/* V accessories */
	MOTOR_ANALOG_CHANNEL_VSENSE,	/* V analog sensors */

	/* analog sensor input */
	MOTOR_ANALOG_CHANNEL_THROTTLE,
	MOTOR_ANALOG_CHANNEL_BRAKE,
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
	bool EncoderIsQuadratureModeEnabled;
	bool EncoderIsALeadBPositive;

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

	Motor_CommutationMode_T 	CommutationMode;
	Motor_SensorMode_T 			SensorMode;
	Motor_ControlMode_T 		ControlMode;
	Motor_BrakeMode_T 			BrakeMode;
	Motor_AlignMode_T 			AlignMode;

	Phase_Mode_T				PhasePwmMode;
	BEMF_SampleMode_T			BemfSampleMode;

	Motor_InputMode_T 			InputMode; //UserMode

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

	const uint32_t LINEAR_V_ABC_R1;
	const uint32_t LINEAR_V_ABC_R2;
	const uint32_t LINEAR_V_BUS_R1;
	const uint32_t LINEAR_V_BUS_R2;

	const uint32_t LINEAR_V_ADC_VREF;
	const uint32_t LINEAR_V_ADC_BITS;

	uint8_t * const P_EEPROM; 	//or flash partition struct


	//static across motor and sub module instances, todo
	const uint32_t MOTOR_PWM_FREQ;
	const uint32_t PHASE_PWM_PERIOD;
	//	const uint32_t ENCODER_ANGLE_RES_BITS;
	//	const bool ENCODER_IS_A_LEAD_B_INCREMENT;
	const uint32_t HALL_ENCODER_TIMER_COUNTER_MAX; //Hall mode only
	const uint32_t HALL_ENCODER_TIMER_COUNTER_FREQ; //Hall mode only
}
Motor_Init_T;


#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif

typedef struct
{

	/* if Analog_T abstracted to controls all adc.(nfixed and nmultimuxed modes). Analog_T scale 1 per motor, can be controlled within motor module */
	//	Analog_T Analog;

	// throttle and brake may need to be pointers, when 1 adc channel controls multiple motors
	volatile uint16_t AnalogChannelResults[MOTOR_ANALOG_ADC_CHANNEL_COUNT];
	volatile const adc_t * p_Ia_ADCU;
	volatile const adc_t * p_Ib_ADCU;
	volatile const adc_t * p_Ic_ADCU;
//	volatile const adc_t * p_VBus_ADCU;


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


	/*
	 * Runtime vars
	 */
	volatile uint32_t ControlTimerBase;	 /* Control Freq, calibration, commutation, angle control */
	Thread_T ControlTimerThread;
	volatile uint32_t MillisTimerBase;	 /* Millis, UI */
	Thread_T MillisTimerThread;
	Thread_T SecondsTimerThread;



	volatile Motor_Direction_T 			Direction; //active spin direction
	volatile bool IsActiveControl; 		//six step observe/control
	volatile bool IsStartUp;			//bemf substate

	//control and feedback
	Linear_T Ramp;
	volatile uint32_t RampIndex;

	//	//PID_T PidSpeed;
	//	PID_T PidId;
	//	PID_T PidIq;
	//	volatile qfrac16_t IdReq; /* PID setpoint */
	//	volatile qfrac16_t IqReq;

	volatile uint16_t UserCmd; 				// SetPoint pre ramp, VReq/IReq/SpeedReq,  user input throttle or brake,
	volatile uint16_t UserCmdPrev;
//	volatile int32_t UserCmdDelta;
	volatile uint16_t RampCmd;				//SetPoint after ramp
	volatile uint16_t VPwm; 				//Control Variable
	volatile uint16_t SpeedFeedback_RPM;	//Feedback Variable

	volatile uint32_t PhaseCurrent_ADCU;
	volatile uint32_t PhaseCurrentPeak_ADCU;

	volatile uint32_t PhaseCurrentFiltered_Frac16;

	//openloop
	Linear_T OpenLoopRamp;
	volatile uint32_t OpenLoopRampIndex;
	volatile uint32_t OpenLoopCommutationPeriod;		 //CommutationPeriodCtrl openloop
	volatile uint16_t OpenLoopSpeed_RPM;
	volatile uint16_t OpenLoopVPwm;

	//FOC
	volatile FOC_T Foc;
	volatile qangle16_t ElectricalAngle; //same as foc.theta
	//	volatile qangle16_t MechanicalAngle;


	//interpolated angle
	volatile qangle16_t HallAngle;
	volatile qangle16_t ElectricalDeltaPrev;
	volatile uint32_t InterpolatedAngleIndex;

	//sixstep
	volatile Motor_SectorId_T NextSector;			 //for 6 step openloop/sensorless
	volatile Motor_SectorId_T CommutationSector;	 //for 6 step openloop/sensorless

	//move to UI
	volatile Motor_Direction_T 			DirectionInput; ///buffered
	volatile bool IsDirectionNeutral;

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

	volatile bool IsThrottleRelease;
//	uint32_t JogSteps;
//	uint32_t StepCount;

	/* UI report */
//	volatile uint16_t VBus;
}
Motor_T;


static inline uint32_t Motor_GetSpeed(Motor_T * p_motor)
{
//	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	return p_motor->SpeedFeedback_RPM;
}

static inline void Motor_ProcSpeed(Motor_T * p_motor)
{
//	if(p_motor->Parameters.SensorMode != MOTOR_SENSOR_MODE_OPEN_LOOP)
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
		if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder))
		{
			p_motor->SpeedFeedback_RPM = 0; //todo stop flag
		}
	}
	else if (p_motor->Parameters.SensorMode == MOTOR_SENSOR_MODE_OPEN_LOOP)
	{
		if ((p_motor->UserCmd == 0U)) //no braking/freewheel in openloop
		{
//			if(p_motor->UserCmdPrev > 0U)
//			{
				p_motor->SpeedFeedback_RPM = 0;
//			}
		}
	}
//	else //other modes covered by always capture DeltaD
//	{
//
//	}
}


static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}











static inline bool Motor_IsCurrentFeedbackMode(Motor_T * p_motor)
{
	return  (
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) ||
				(p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT)
			) ? true : false;
}

//move to ui?
static inline bool Motor_PollToggleDirectionError(Motor_T * p_motor)
{
	//proc direction
	//		 if (p_motor->Direction != p_motor->InputDirection)//direction reversed
	//		{
	//			Blinky_SetOnTime(&p_Motor->Alarm, 1000)
	//		}
}

static inline bool Motor_PollToggleDirectionUpdate(Motor_T * p_motor)
{
	if (p_motor->Direction != p_motor->DirectionInput)
	{
		Motor_SetDirection(p_motor->DirectionInput);
	}
}





extern void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInitStruct);
extern void Motor_InitReboot(Motor_T * p_motor);
//extern void Motor_StartAlign(Motor_T * p_motor);
//extern void Motor_OnBlock(Motor_T * p_motor);


#endif
