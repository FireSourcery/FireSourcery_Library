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
    @brief  Per Motor State Control.
    @version V0
*/
/**************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H


#include "HAL_Motor.h"

#include "Config.h"

//#include "Peripheral/Pin/Debounce.h"
//#include "Peripheral/Pin/Pin.h"
//#include "Peripheral/Analog/Analog.h"
//#include "Peripheral/Flash/Flash.h"

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
#include "Math/Linear/Linear_ADC.h"
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

//	Motor_InputMode_T 			InputMode; //UserMode

	qfrac16_t FocOpenLoopVq;
	qfrac16_t FocAlignVd;
	//	qfrac16_t OpenLoopVHzGain; //vhz scale
}
Motor_Parameters_T;

#ifdef CONFIG_MOTOR_ADC_8
	typedef uint16_t adc_t;
#elif defined(CONFIG_MOTOR_ADC_16)
	typedef uint16_t adc_t;
#endif

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

	const uint8_t * const P_EEPROM; 	//or flash partition struct
	//	Flash_Partition_T  p_FlashPartition;

	volatile const adc_t * const P_VBUS_ADCU;
	volatile const adc_t * const P_VA_ADCU;
	volatile const adc_t * const P_VB_ADCU;
	volatile const adc_t * const P_VC_ADCU;
	volatile const adc_t * const P_IA_ADCU;
	volatile const adc_t * const P_IB_ADCU;
	volatile const adc_t * const P_IC_ADCU;

	volatile const adc_t * const P_HEAT_MOTOR_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_ADCU;
	volatile const adc_t * const P_HEAT_PCB_ADCU;

	//use define macro if static across motor and sub module instances, todo
	const uint32_t MOTOR_PWM_FREQ;
	const uint32_t PHASE_PWM_PERIOD;
	//	const uint32_t ENCODER_ANGLE_RES_BITS;
	//	const bool ENCODER_IS_A_LEAD_B_INCREMENT;
	const uint32_t HALL_ENCODER_TIMER_COUNTER_MAX;
	const uint32_t HALL_ENCODER_TIMER_COUNTER_FREQ;
}
Motor_Init_T;

typedef struct
{
 	const Motor_Init_T * p_Init;			//compile time const, unique per moter
	Motor_Parameters_T	Parameters;			//Programmable parameters, runtime variable load from eeprom

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

//	volatile const adc_t * p_HeatMotor_ADCU;
//	volatile const adc_t * p_HeatMosfets_ADCU;
//	volatile const adc_t * p_HeatPcb_ADCU;
//	volatile const adc_t * p_Ia_ADCU;
//	volatile const adc_t * p_Ib_ADCU;
//	volatile const adc_t * p_Ic_ADCU;
//	volatile const adc_t * p_VBus_ADCU;

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
	volatile Motor_Direction_T 			DirectionInput; ///buffered
//	volatile bool IsDirectionNeutral;
	volatile bool IsActiveControl; 		//merge

	//Control Variable
	volatile uint16_t UserCmd; 				// SetPoint pre ramp, VReq/IReq/SpeedReq,  user input throttle or brake,
	volatile uint16_t UserCmdPrev;
//	volatile int32_t UserCmdDelta;
	Linear_T Ramp;
	volatile uint32_t RampIndex;
	volatile uint16_t RampCmd;				//SetPoint after ramp
	volatile uint16_t VPwm; 				//Control Variable
	volatile uint16_t SpeedFeedback_RPM;	//Feedback Variable
	volatile uint32_t IBus_ADCU; 			//phase positive current
	volatile uint32_t IBus_Frac16;
//	volatile uint32_t PhaseCurrentPeak_ADCU;
//	volatile uint32_t PhaseCurrentFiltered_Frac16;
	//	PID_T PidSpeed;
	//	PID_T PidId;
	//	PID_T PidIq;
	//	volatile qfrac16_t IdReq; /* PID setpoint */
	//	volatile qfrac16_t IqReq;

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

//	volatile bool IsStartUp;			//bemf substate

	//todo alarm and thermistor

//	uint32_t JogSteps;
//	uint32_t StepCount;

	/* UI report */
	volatile uint16_t VBus_mV;
	volatile uint16_t VBemfPeak_mV;
	volatile uint16_t IBus_Amp;
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



static inline void Motor_Float(Motor_T * p_motor)
{
	Phase_Float(&p_motor->Phase);
}

static inline void Motor_ProcRamp(Motor_T * p_motor) // input voltage/speed cmd
{
	p_motor->RampCmd = Linear_Ramp_CalcTarget_IncIndex(&p_motor->Ramp, &p_motor->RampIndex, 1U);
}

static inline void Motor_PollSpeedFeedback(Motor_T * p_motor)
{
	if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
	{
		Motor_ProcSpeedFeedback(p_motor);
	}
}


static inline void Motor_PollAnalogStartAll(Motor_T * p_motor)
{
	if (Thread_PollTimerCompletePeriodic(&p_motor->MillisTimerThread) == true)
	{
		HAL_Motor_EnqueueConversionIdle(p_motor);
	}
}



static inline void CaptureIBus(Motor_T * p_motor, qfrac16_t i_temp)
{
 	if(i_temp > 0)
	{
		p_motor->IBus_Frac16 = i_temp;// * 2U;
	}
	else
	{
		p_motor->IBus_Frac16 = (-i_temp);// * 2U;
	}
//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IA_ADCU;
}


static inline void Motor_CaptureIa_IO(Motor_T * p_motor)
{
	//Filter here if needed
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIa, *p_motor->p_Init->P_IA_ADCU);

	CaptureIBus(p_motor, i_temp);
//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IA_ADCU;
}

static inline void Motor_CaptureIb_IO(Motor_T * p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIb, *p_motor->p_Init->P_IB_ADCU);

	CaptureIBus(p_motor, i_temp);
//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IB_ADCU;
}

static inline void Motor_CaptureIc_IO(Motor_T * p_motor)
{
	qfrac16_t i_temp = Linear_ADC_CalcFractionSigned16(&p_motor->UnitIc, *p_motor->p_Init->P_IC_ADCU);

	CaptureIBus(p_motor, i_temp);
//	p_motor->IBus_ADCU = *p_motor->p_Init->P_IC_ADCU;
}




// monitor heat functions



extern void Motor_Init(Motor_T * p_motor, const Motor_Init_T * p_motorInitStruct);
extern void Motor_InitReboot(Motor_T * p_motor);
//extern void Motor_StartAlign(Motor_T * p_motor);
//extern void Motor_OnBlock(Motor_T * p_motor);


#endif
