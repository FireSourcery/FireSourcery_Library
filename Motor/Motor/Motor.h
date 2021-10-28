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

#include "System/Debug/Debug.h"

#include "HAL_Motor.h"

#include "Config.h"

#include "Transducer/Phase/Phase.h"
#include "Transducer/Hall/Hall.h"
#include "Transducer/BEMF/BEMF.h"

#include "Math/FOC.h"

#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_DeltaD.h"
#include "Transducer/Encoder/Encoder.h"

#include "System/StateMachine/StateMachine.h"
#include "System/Thread/Thread.h"

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
	MOTOR_SECTOR_ID_1 = 1U, //Phase AC
	MOTOR_SECTOR_ID_2 = 2U,
	MOTOR_SECTOR_ID_3 = 3U,
	MOTOR_SECTOR_ID_4 = 4U,
	MOTOR_SECTOR_ID_5 = 5U,
	MOTOR_SECTOR_ID_6 = 6U,
	MOTOR_SECTOR_ID_7 = 7U,

	MOTOR_SECTOR_ERROR_000 = 0U,
	MOTOR_SECTOR_ERROR_111 = 7U,

	MOTOR_PHASE_AC = 1U,
	MOTOR_PHASE_BC = 2U,
	MOTOR_PHASE_BA = 3U,
	MOTOR_PHASE_CA = 4U,
	MOTOR_PHASE_CB = 5U,
	MOTOR_PHASE_AB = 6U,
} Motor_SectorId_T;

/*
 * Commutation Direction
 */
typedef enum
{
//	MOTOR_DIRECTION_N,
	MOTOR_DIRECTION_CW = 0U,
	MOTOR_DIRECTION_CCW = 1U,
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

#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif


typedef const struct
{
	volatile const adc_t * const P_VBUS_ADCU;
	volatile const adc_t * const P_VA_ADCU;
	volatile const adc_t * const P_VB_ADCU;
	volatile const adc_t * const P_VC_ADCU;
	volatile const adc_t * const P_IA_ADCU;
	volatile const adc_t * const P_IB_ADCU;
	volatile const adc_t * const P_IC_ADCU;
	volatile const adc_t * const P_HEAT_MOTOR_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_ADCU; //if per motor mosfet sensor is implemented
}
Motor_AdcMap_T;

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
//
/*!
	@brief Motor Parameters
	Runtime variable configuration

	load from flash
 */
typedef struct __attribute__ ((aligned (4U)))
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

//	DirectionCalibration_T

//	Motor_InputMode_T 			InputMode; //UserMode

	qfrac16_t FocOpenLoopVq;
	qfrac16_t FocAlignVd;
	//	qfrac16_t OpenLoopVHzGain; //vhz scale

	const volatile uint16_t IA_ZERO_ADCU;
	const volatile uint16_t IB_ZERO_ADCU;
	const volatile uint16_t IC_ZERO_ADCU;
}
Motor_Parameters_T;

/*
	@brief Motor Init
	Compile time const configuration
 */
typedef const struct Motor_Init_Tag
{
    const HAL_Phase_T 		HAL_PHASE;
	const HAL_Encoder_T 	HAL_ENCODER;
	const HAL_Hall_T 		HAL_HALL;
//	const HAL_BEMF_T HAL_BEMF;

	const uint32_t LINEAR_V_ABC_R1;
	const uint32_t LINEAR_V_ABC_R2;
	const uint32_t LINEAR_V_BUS_R1;
	const uint32_t LINEAR_V_BUS_R2;
	const uint32_t LINEAR_V_ADC_VREF;
	const uint32_t LINEAR_V_ADC_BITS;

	const Motor_Parameters_T * const P_PARAMS; //user define location in Nv mem
	//	Flash_Partition_T  p_FlashPartition;
	const Motor_Parameters_T * const P_PARAMETERS_DEFAULT;

//	const Motor_AdcMap_T ADC_MAP;

	volatile const adc_t * const P_VBUS_ADCU;
	volatile const adc_t * const P_VA_ADCU;
	volatile const adc_t * const P_VB_ADCU;
	volatile const adc_t * const P_VC_ADCU;
	volatile const adc_t * const P_IA_ADCU;
	volatile const adc_t * const P_IB_ADCU;
	volatile const adc_t * const P_IC_ADCU;
	volatile const adc_t * const P_HEAT_MOTOR_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_ADCU; //if per motor mosfet sensor is implemented
//	volatile const adc_t * const P_HEAT_PCB_ADCU;

	//use define macro if static across motor and sub module instances, todo
	const uint32_t MOTOR_PWM_FREQ;
	const uint32_t PHASE_PWM_PERIOD;
	//	const uint32_t ENCODER_ANGLE_RES_BITS;
	//	const bool ENCODER_IS_A_LEAD_B_INCREMENT;
	const uint32_t HALL_ENCODER_TIMER_COUNTER_MAX;
	const uint32_t HALL_ENCODER_TIMER_COUNTER_FREQ;
}
Motor_Constants_T;

typedef struct
{
 	const Motor_Constants_T * p_Constants;		//compile time const, unique per motor

// 	const Motor_Config_T CONFIG;		//compile time const, unique per motor

 	//#ifdef ram copy
 	Motor_Parameters_T Parameters;				//Programmable parameters, runtime variable load from eeprom

	/*
	 * Hardware Wrappers
	 */
	Encoder_T Encoder;
	Phase_T Phase;
	Hall_T Hall;
	BEMF_T Bemf;

	/*
	 * State Machine
	 */
	StateMachine_T StateMachine;

	/*
	 * SW config
	 */
	//not const due to adc calibration
	Linear_T UnitVBus;
	Linear_T UnitVabc; //Bemf V and mV conversion
	Linear_T UnitIa; //Frac16 and UserUnits (Amp)
	Linear_T UnitIb;
	Linear_T UnitIc;

	/******************************************************************************/
	/*
	 * Runtime vars
	 */
	/******************************************************************************/
//	volatile Motor_AdcConversionFlag_T  //move hal motor analog outside

	volatile uint32_t ControlTimerBase;	 /* Control Freq ~ 20kHz, calibration, commutation, angle control */
	Thread_T ControlTimerThread;

	//can be shared across motor instances, not reset in motor module
	volatile uint32_t MillisTimerBase;	 /* Millis, UI */
	Thread_T MillisTimerThread;
	Thread_T SecondsTimerThread;

	volatile Motor_Direction_T 			Direction; //active spin direction
	volatile Motor_Direction_T 			DirectionInput; ///buffered
//	volatile bool IsDirectionNeutral;
	volatile bool IsActiveControl;

	/*
	 * Control Variable
	 */
	volatile uint16_t UserCmd; 				//Fraction16 SetPoint pre ramp, VReq/IReq/SpeedReq,  user input throttle or brake,
	volatile uint16_t UserCmdPrev;
	volatile Linear_T Ramp;
	volatile uint32_t RampIndex;
	volatile uint16_t RampCmd;				//SetPoint after ramp
	volatile uint16_t VPwm; 				//Control Variable


	/* Feedback */
	volatile uint16_t Speed_RPM;	//Feedback Variable
	volatile uint32_t IBus_ADCU; 			//phase positive current
	volatile uint32_t IBus_Frac16;			//0-65535

	//	PID_T PidSpeed;
	//	PID_T PidId;
	//	PID_T PidIq;
	//	volatile qfrac16_t IdReq; /* PID setpoint */
	//	volatile qfrac16_t IqReq;

	/*
	 * Open-loop
	 */
	Linear_T OpenLoopRamp;
	volatile uint32_t OpenLoopRampIndex;
	volatile uint32_t OpenLoopCommutationPeriod;		 //CommutationPeriodCtrl openloop
	volatile uint16_t OpenLoopSpeed_RPM;
	volatile uint16_t OpenLoopVPwm;

	/*
	 * FOC
	 */
	volatile FOC_T Foc;
	volatile qangle16_t ElectricalAngle; //same as foc.theta
	//	volatile qangle16_t MechanicalAngle;

	/* interpolated angle */
	volatile qangle16_t HallAngle;
	volatile qangle16_t ElectricalDeltaPrev;
	volatile uint32_t InterpolatedAngleIndex;

	/*
	 * Six-Step
	 */
	volatile Motor_SectorId_T NextSector;			 //for 6 step openloop/sensorless
	volatile Motor_SectorId_T CommutationSector;	 //for 6 step openloop/sensorless
	//	volatile bool IsStartUp;			//bemf substate

	//todo alarm and thermistor

//	uint32_t JogSteps;
//	uint32_t StepCount;

	/* UI Unit report */
	volatile uint16_t VBus_mV;
	volatile uint16_t VBemfPeak_mV;
	volatile uint16_t VBemfA_mV;
	volatile uint16_t IBus_Amp;
}
Motor_T;

static inline void Motor_PollAnalogStartAll(Motor_T * p_motor) //run doing stop state
{
	if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
	{
#ifdef CONFIG_MOTOR_ANALOG_USE_HAL
#elif defined(CONFIG_MOTOR_ANALOG_USE_POINTER)
		Analog_Enqueue(p_motor->p_Analog, p_motor->p_ConversionIdle);
#endif

		HAL_Motor_EnqueueConversionIdle(p_motor);
	}
}

//todo check 65536 boundary case
static inline void Motor_CaptureIBusIa_IO(Motor_T * p_motor)
{
	//Filter here if needed
	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIa, *p_motor->p_Constants->P_IA_ADCU);

//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IA_ADCU;
//	p_motor->IBus_ADCU  Filter_MovAvg(&p_motor->FilterIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA]);
}

static inline void Motor_CaptureIBusIb_IO(Motor_T * p_motor)
{
	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIb, *p_motor->p_Constants->P_IB_ADCU);

//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IB_ADCU;
	Debug_CaptureElapsed(4);
}

static inline void Motor_CaptureIBusIc_IO(Motor_T * p_motor)
{
	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIc, *p_motor->p_Constants->P_IC_ADCU);

//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IC_ADCU;
}



//Wrappers

static inline uint32_t Motor_GetSpeed(Motor_T * p_motor)
{
//	p_motor->SpeedFeedback_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	return p_motor->Speed_RPM;
}

static inline void Motor_CaptureSpeed(Motor_T * p_motor)
{
//	if(p_motor->Parameters.SensorMode != MOTOR_SENSOR_MODE_OPEN_LOOP)
	{
		p_motor->Speed_RPM = Encoder_Motor_GetMechanicalRpm(&p_motor->Encoder);
	}
//	else
//	{
//		p_motor->SpeedFeedback_RPM = p_motor->Openloop_RPM;
//	}
}

static inline uint32_t Motor_GetBemf_Frac16(Motor_T * p_motor)
{
	return Linear_Voltage_CalcUnsignedFraction16(&p_motor->UnitVabc, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));
}


//static inline uint32_t Motor_LoadParameterAll(Motor_T * p_motor)
//{
//	Flash_EEPROM_ReadBytes(&MotorFlashMain, &p_motor->Parameters, p_motor->p_Constants->P_EEPROM, sizeof(Motor_Parameters_T));
//}
//
//static inline uint32_t Motor_SaveParametersAll(Motor_T * p_motor)
//{
// 	Flash_EEPROM_WriteAlignedBytes(&MotorFlashMain, p_motor->p_Constants->P_EEPROM, &p_motor->Parameters, sizeof(Motor_Parameters_T));
//}



//UI
static inline void Motor_ProcUnitOutputs(Motor_T * p_motor)
{
	p_motor->VBus_mV 		= Linear_Voltage_CalcMilliV(&p_motor->UnitVBus, *p_motor->p_Constants->P_VBUS_ADCU);
	p_motor->VBemfPeak_mV 	= Linear_Voltage_CalcMilliV(&p_motor->UnitVabc, BEMF_GetVBemfPeak_ADCU(&p_motor->Bemf));
}


static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}


// todo monitor heat functions

extern void Motor_Init_Default(Motor_T * p_motor, const Motor_Constants_T * p_motorInit);

extern void Motor_Init(Motor_T * p_motor, const Motor_Constants_T * p_motorInitStruct, const Motor_Parameters_T * p_parameters);
extern void Motor_InitConsts(Motor_T * p_motor, const Motor_Constants_T * p_motorInit);
extern void Motor_InitParameters(Motor_T * p_motor, const Motor_Parameters_T * p_parameters);
extern void Motor_InitReboot(Motor_T * p_motor);
//extern void Motor_StartAlign(Motor_T * p_motor);
//extern void Motor_OnBlock(Motor_T * p_motor);

#endif
