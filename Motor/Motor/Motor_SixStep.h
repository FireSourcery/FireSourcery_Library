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
    @file 	Motor_SixStep.h
    @author FireSoucery
    @brief  Motor Six Step submodule.
    		Six Step Commutation strategy. Polar PWM

    		Current design - Only SixStep module tracks phase

    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_SIXSTEP_H
#define MOTOR_SIXSTEP_H

#include "Motor.h"
#include "Config.h"

#include "Transducer/Hall/Hall.h"
//#include "Transducer/BEMF/BEMF.h"
#include "Transducer/Phase/Phase.h"

#include "Transducer/Encoder/Encoder_DeltaT.h"
#include "Transducer/Encoder/Encoder_Motor.h"
#include "Transducer/Encoder/Encoder.h"

#include "Peripheral/Analog/AnalogN/AnalogN.h"

#include "Math/Linear/Linear_Ramp.h"
#include "Math/Linear/Linear.h"
#include "Math/Filter/Filter_MovAvg.h"
#include "Math/Filter/Filter.h"

#include <stdint.h>
#include <stdbool.h>

#include "System/SysTime/SysTime.h"
#include "Utility/Debug/Debug.h"


static inline void CaptureDebugPwmOn(Motor_T * p_motor, uint16_t adcu)
{
	if(p_motor->DebugCounterPwmOn < 200)
	{
		p_motor->SampleTimesPwmOn[p_motor->DebugCounterPwmOn] = SysTime_GetMicros() - p_motor->MicrosRef;
		p_motor->VSamplesPwmOn[p_motor->DebugCounterPwmOn] = adcu;
		p_motor->DebugCounterPwmOn++;
	}
}

static inline void CaptureDebugPwmOff(Motor_T * p_motor, uint16_t adcu)
{
	if(p_motor->DebugCounterPwmOff < 200)
	{
		p_motor->SampleTimesPwmOff[p_motor->DebugCounterPwmOff] = SysTime_GetMicros() - p_motor->MicrosRef;
		p_motor->VSamplesPwmOff[p_motor->DebugCounterPwmOff] = adcu;
		p_motor->DebugCounterPwmOff++;
	}
}


/*
 * On Adc measure of phase complete
 */
static inline void Motor_SixStep_CaptureBemfA(Motor_T * p_motor)
{
//	if(BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
//		Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
//		if (p_motor->IsPwmOn == true)
//		{
			CaptureDebugPwmOn( p_motor, p_motor->AnalogResults.Va_ADCU);
//
//		}
//		else
//		{
//			CaptureDebugPwmOff( p_motor, p_motor->AnalogResults.Va_ADCU);
////			Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
//		}
	}

	p_motor->AnalogResults.Va_ADCU = 0;
//	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
}

static inline void Motor_SixStep_CaptureBemfB(Motor_T * p_motor)
{
//	if(BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
//		Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU);
//		if (p_motor->IsPwmOn == true)
//		{
			CaptureDebugPwmOn( p_motor, p_motor->AnalogResults.Vb_ADCU);
//		}
//		else
//		{
//			CaptureDebugPwmOff( p_motor, p_motor->AnalogResults.Vb_ADCU);
////			Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU);
//		}
	}
	p_motor->AnalogResults.Vb_ADCU = 0;
//	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
}

static inline void Motor_SixStep_CaptureBemfC(Motor_T * p_motor)
{
//	if(BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
//		Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU);
//		if (p_motor->IsPwmOn == true)
//		{
			CaptureDebugPwmOn(p_motor, p_motor->AnalogResults.Vc_ADCU);
//		}
//		else
//		{
//			CaptureDebugPwmOff(p_motor, p_motor->AnalogResults.Vc_ADCU);
////			Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU);
//		}
	}

	p_motor->AnalogResults.Vc_ADCU = 0;
//	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
}

static inline void Motor_SixStep_CaptureIBusA(Motor_T * p_motor)
{
//	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIa,  p_motor->AnalogResults.Ia_ADCU);
	p_motor->IBusSum_Frac16 += Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIa, p_motor->AnalogResults.Ia_ADCU); //todo seperate capture
	//Filter here if needed
//	p_motor->IBus_ADCU  Filter_MovAvg(&p_motor->FilterIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA]);
}

static inline void Motor_SixStep_CaptureIBusB(Motor_T * p_motor)
{
//	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU);
	p_motor->IBusSum_Frac16 += Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU);
}

static inline void Motor_SixStep_CaptureIBusC(Motor_T * p_motor)
{
//	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU);
	p_motor->IBusSum_Frac16 += Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU);
}


/*
 * Case Observe: Adc
 * Case Control: Adc, pwm, cv
 * Case Observe Commutation: bemf
 * Case Control Commutation: bemf, phase, cv, pwm
 *
 * let complier optimize
 */

/*
 *  Once per commutation, both observe and control modes
 */
static inline void MapMotorSixStepBemfPhase(Motor_T * p_motor)
{
// 	switch (p_motor->CommutationPhase)
//	{
//		case MOTOR_PHASE_ERROR_0:
//			//set error
//			break;
//
//		case MOTOR_PHASE_AC:
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_BC; BEMF_MapCcwPhaseAC(&p_motor->Bemf);}
//			else											{p_motor->NextPhase = MOTOR_PHASE_AB; BEMF_MapCwPhaseAC(&p_motor->Bemf);}
//			break;
//
//		case MOTOR_PHASE_BC:
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_BA; BEMF_MapCcwPhaseBC(&p_motor->Bemf);}
//			else											{p_motor->NextPhase = MOTOR_PHASE_AC; BEMF_MapCwPhaseBC(&p_motor->Bemf);}
//			break;
//
//		case MOTOR_PHASE_BA:
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_CA; BEMF_MapCcwPhaseBA(&p_motor->Bemf);}
//			else											{p_motor->NextPhase = MOTOR_PHASE_BC; BEMF_MapCwPhaseBA(&p_motor->Bemf);}
//			break;
//
//		case MOTOR_PHASE_CA:
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_CB; BEMF_MapCcwPhaseCA(&p_motor->Bemf);}
//			else											{p_motor->NextPhase = MOTOR_PHASE_BA; BEMF_MapCwPhaseCA(&p_motor->Bemf);}
//			break;
//
//		case MOTOR_PHASE_CB:
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_AB; BEMF_MapCcwPhaseCB(&p_motor->Bemf);}
//			else											{p_motor->NextPhase = MOTOR_PHASE_CA; BEMF_MapCwPhaseCB(&p_motor->Bemf);}
//			break;
//
//		case MOTOR_PHASE_AB:
//			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_AC; BEMF_MapCcwPhaseAB(&p_motor->Bemf);}
//			else											{p_motor->NextPhase = MOTOR_PHASE_CB; BEMF_MapCwPhaseAB(&p_motor->Bemf);}
//			break;
//
//		case MOTOR_PHASE_ERROR_7:
//			//set error
//			break;
//
//		default:
//			break;
//	}

 	//alternatively
//	if (p_motor->Direction == MOTOR_DIRECTION_CCW)
//	{
//		p_motor->NextSector = (p_motor->CommutationSector + 1U) % 6U;
//	}
//	else
//	{
//		p_motor->NextSector = (p_motor->CommutationSector - 1U) % 6U;
//		p_motor->Bemf.IsBemfRising = !p_motor->Bemf.IsBemfRising;

 	// 	p_motor->NextSector = (p_motor->NextSector + 4U) % 6U;
 	// 	p_motor->Bemf.IsBemfRising = !p_motor->Bemf.IsBemfRising;
//	}
}

/*
 *  Once per commutation, control mode only
 */
static inline void ActivateMotorSixStepCommutation(Motor_T * p_motor)
{
 	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0: 	break;
		case MOTOR_PHASE_AC: Phase_Polar_ActivateAC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BC: Phase_Polar_ActivateBC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BA: Phase_Polar_ActivateBA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CA: Phase_Polar_ActivateCA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CB: Phase_Polar_ActivateCB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_AB: Phase_Polar_ActivateAB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_ERROR_7: 	break;
		default: break;
	}
}

/*
 *  Every Pwm, control mode only
 */
static inline void ActivateMotorSixStepPhaseDuty(Motor_T * p_motor)
{
	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0: break;
		case MOTOR_PHASE_AC: Phase_Polar_ActivateDutyAC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BC: Phase_Polar_ActivateDutyBC(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_BA: Phase_Polar_ActivateDutyBA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CA: Phase_Polar_ActivateDutyCA(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_CB: Phase_Polar_ActivateDutyCB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_AB: Phase_Polar_ActivateDutyAB(&p_motor->Phase, p_motor->VPwm); break;
		case MOTOR_PHASE_ERROR_7: break;
		default: break;
	}
}

static void ActivateMotorSixStepAnalogPhase(Motor_T * p_motor, const AnalogN_Conversion_T * p_vPhase, const AnalogN_Conversion_T * p_iPhase)
{
	AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

	AnalogN_EnqueueConversionOptions_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_RESTORE);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VPOS);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_iPhase);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_vPhase);

	//should not start until middle of pwm cycle
//	AnalogN_EnqueueConversionOptions_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_PWM_ON);
//	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_vPhase);
//	AnalogN_EnqueueConversionOptions_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_RESTORE);
//	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_iPhase);
//	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VPOS);

	AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);
}

/*
 * All cases
 */
static inline void ActivateMotorSixStepAnalog(Motor_T * p_motor)
{
	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0: break;
		case MOTOR_PHASE_AC : ActivateMotorSixStepAnalogPhase(p_motor, &p_motor->CONFIG.CONVERSION_VB, &p_motor->CONFIG.CONVERSION_IC); break;
		case MOTOR_PHASE_BC : ActivateMotorSixStepAnalogPhase(p_motor, &p_motor->CONFIG.CONVERSION_VA, &p_motor->CONFIG.CONVERSION_IC); break;
		case MOTOR_PHASE_BA : ActivateMotorSixStepAnalogPhase(p_motor, &p_motor->CONFIG.CONVERSION_VC, &p_motor->CONFIG.CONVERSION_IA); break;
		case MOTOR_PHASE_CA : ActivateMotorSixStepAnalogPhase(p_motor, &p_motor->CONFIG.CONVERSION_VB, &p_motor->CONFIG.CONVERSION_IA); break;
		case MOTOR_PHASE_CB : ActivateMotorSixStepAnalogPhase(p_motor, &p_motor->CONFIG.CONVERSION_VA, &p_motor->CONFIG.CONVERSION_IB); break;
		case MOTOR_PHASE_AB : ActivateMotorSixStepAnalogPhase(p_motor, &p_motor->CONFIG.CONVERSION_VC, &p_motor->CONFIG.CONVERSION_IB); break;
		case MOTOR_PHASE_ERROR_7: break;
		default: break;
	}
}

/*
 * Open Loop Functions
 */
static inline bool Motor_SixStep_PollOpenLoop(Motor_T * p_motor)
{
	bool commutation = Timer_Poll(&p_motor->ControlTimer);

	if (commutation == true)
	{
//		p_motor->OpenLoopSpeed_RPM = Linear_Ramp_ProcIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopCommutationPeriod);
		p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
		Timer_StartPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);
	}

	return commutation;
}

static inline void Motor_SixStep_StartOpenLoop(Motor_T * p_motor)
{
	// if pwm update is only on commutation cycle, must start at sufficient speed for timer driven fixed angle displacements
//	Linear_Ramp_Init_Millis(&p_motor->OpenLoopRamp, 20000U, p_motor->Parameters.OpenLoopSpeedStart, p_motor->Parameters.OpenLoopSpeedFinal, 1000U);
//	Linear_Ramp_InitAcceleration(&p_motor->OpenLoopRamp, 20000U, p_motor->Parameters.OpenLoopSpeedStart, p_motor->Parameters.OpenLoopSpeedFinal, 100U);
	p_motor->OpenLoopRampIndex = 0U;

//	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_ProcIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, 0U);
	p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);

	//motor aligned to phase A
	p_motor->NextPhase = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? MOTOR_PHASE_BC : MOTOR_PHASE_AB;
}


static inline bool Motor_SixStep_GetBemfReliable(Motor_T * p_motor) {
//	return BEMF_GetIsReliable(&p_motor->Bemf);
}

static inline bool PollMotorSixStepCommutation(Motor_T * p_motor)
{
	bool commutation = false;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP :
			//	p_motor->VPwm = p_motor->RampCmd; //p_motor->Parameters.OpenLoopVPwm_Frac16 +
			//
			//	// bound to 2.5 to 10 percent
			////	if 		(p_motor->VPwm < p_motor->Parameters.OpenLoopVPwmMin)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMin;}
			////	else if (p_motor->VPwm > p_motor->Parameters.OpenLoopVPwmMax)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMax;}
			//
			if(Motor_SixStep_PollOpenLoop(p_motor) == true)
			{
				p_motor->CommutationPhase = p_motor->NextPhase;
				commutation = true;
			}
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			if(Motor_SixStep_GetBemfReliable(p_motor) == true)
			{
//				if(BEMF_CheckPhasePeriod(&p_motor->Bemf) == true)
//				{
//					p_motor->CommutationPhase = p_motor->NextPhase;
//					commutation = true;
//				}
			}
			else
			{
				commutation = Motor_SixStep_PollOpenLoop(p_motor);
			}
			break;

		case MOTOR_SENSOR_MODE_HALL :
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				p_motor->CommutationPhase = (Motor_SectorId_T)Hall_GetCommutationId(&p_motor->Hall); //hall id match motor id
				commutation = true;
			}
			break;

//	case MOTOR_SENSOR_MODE_ENCODER:	 //encoder use foc
//		break;

		default :
			break;
	}

	return commutation;
}

static inline bool ProcMotorSixStepSensorFeedback(Motor_T * p_motor)
{
	bool commutation = PollMotorSixStepCommutation(p_motor);

	if (commutation == true)
	{
//		MapMotorSixStepBemfPhase(p_motor);
//		BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference

		Encoder_DeltaT_Capture(&p_motor->Encoder);
		Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
		p_motor->IBus_Frac16 = p_motor->IBusSum_Frac16/(p_motor->ControlTimerBase - p_motor->CommutationTimeRef); //minus one if new conversion hasnt completed
//		p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIa,  p_motor->IBusSum_ADCU);
		p_motor->CommutationTimeRef = p_motor->ControlTimerBase;
		p_motor->IBusSum_Frac16		= 0U;

		//debug
		p_motor->MicrosRef = SysTime_GetMicros();
		p_motor->DebugCounterEndPwmOn = p_motor->DebugCounterPwmOn;
		p_motor->DebugCounterPwmOn = 0;
//		p_motor->DebugCounterEndPwmOff = p_motor->DebugCounterPwmOff;
//		p_motor->DebugCounterPwmOff = 0;
	}
	else
	{
//		BEMF_ProcZeroCrossingDetection(&p_motor->Bemf); //  poll zcd with prev capture
	}

//	if (Motor_PollSpeedFeedback(p_motor) == true)
//	{
//		if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) //once per millis
//		{
//			p_motor->Speed_RPM = 0U;
//			p_motor->Speed_Frac16 = 0U;
//		}
//	}

	return commutation;
}



static inline bool PollMotorSixStepIBusOverLimit(Motor_T * p_motor)
{
	bool isOverLimit = false;

	if (p_motor->IBus_Frac16 > p_motor->Parameters.IBusLimit_Frac16)
	{
//		//			p_motor->VPwm = p_motor->VPwm - ((p_motor->IBus_Frac16 - 58982U) * p_motor->VPwm >> 16U)
		if(p_motor->IOverLimitFlag == false) //	if (p_motor->IBus_Frac16 >= p_motor->IBusPrev_Frac16)  only on current increase
		{
			p_motor->IOverLimitFlag = true;
			PID_SetIntegral(&p_motor->PidIBus, p_motor->VPwm);
		}
		p_motor->VPwm = PID_Calc(&p_motor->PidIBus, p_motor->Parameters.IBusLimit_Frac16, p_motor->IBus_Frac16);
		isOverLimit = true;
	}
	else
	{
		p_motor->IOverLimitFlag = false;
	}

	return isOverLimit;
}


static inline void ProcMotorSixStepControlFeedback(Motor_T * p_motor)
{
	switch(p_motor->Parameters.ControlMode)
	{
		case MOTOR_CONTROL_MODE_OPEN_LOOP:
			p_motor->VPwm = p_motor->RampCmd;
			// bound to 2.5 to 10 percent
			//	if 		(p_motor->VPwm < p_motor->Parameters.OpenLoopVPwmMin)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMin;}
			//	else if (p_motor->VPwm > p_motor->Parameters.OpenLoopVPwmMax)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMax;}
				break;

		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE :
			if (PollMotorSixStepIBusOverLimit(p_motor) == false)
			{
				p_motor->VPwm = p_motor->RampCmd;
			}
			break;

//		case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ :
//			// p_motor->VPwm = p_motor->RampCmd * p_motor->Speed_RPM * p_motor->VRpmGain;
//			break;

		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE :
			if (PollMotorSixStepIBusOverLimit(p_motor) == false)
			{
				p_motor->VPwm = p_motor->SpeedControl;
			}
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_CURRENT :
//			if (p_motor->RampCmd > p_motor->Parameters.IBusLimit_Frac16)
//			{
//				p_motor->VPwm = PID_Calc(&p_motor->PidIBus, p_motor->Parameters.IBusLimit_Frac16, p_motor->IBus_Frac16);
//			}
//			else
			{
				p_motor->VPwm = PID_Calc(&p_motor->PidIBus, p_motor->RampCmd, p_motor->IBus_Frac16);
			}
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT :
			p_motor->VPwm = PID_Calc(&p_motor->PidIBus, p_motor->SpeedControl, p_motor->IBus_Frac16);
			break;
		default :
			break;
	}
}

/*
 * Passive Observe
 */
static inline bool Motor_SixStep_ProcPhaseObserve(Motor_T * p_motor)
{
	ActivateMotorSixStepAnalog(p_motor);
	return ProcMotorSixStepSensorFeedback(p_motor);
}

static inline void Motor_SixStep_StartPhaseObserve(Motor_T * p_motor)
{
//	p_motor->IOverLimitFlag = false;
//	BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_PASSIVE); //no blank time
}

/*
 * State loop in 20khz pwm thread
 * Active Control
 */
static inline void Motor_SixStep_ProcPhaseControl(Motor_T * p_motor)
{
	bool commutation = Motor_SixStep_ProcPhaseObserve(p_motor);

	ProcMotorSixStepControlFeedback(p_motor);

	if(commutation == true)
	{
		ActivateMotorSixStepCommutation(p_motor);
	}
	else
	{
		ActivateMotorSixStepPhaseDuty(p_motor); /* update pwm with control variable value every pwm cycle */
	}
}

static inline void Motor_SixStep_ResumePhaseControl(Motor_T * p_motor)
{
	if((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE))
	{
		Motor_ResumeSpeedFeedback(p_motor);
	}

	if ((p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_CURRENT) || (p_motor->Parameters.ControlMode == MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT))
	{
		PID_SetIntegral(&p_motor->PidIBus, p_motor->IBus_Frac16); //set vpwm proportional to current ibus
	}

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP :
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			break;

		case MOTOR_SENSOR_MODE_HALL:
//			BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_COMMUTATION);
			break;

	//	case MOTOR_SENSOR_MODE_ENCODER:
	//		resume =  true;
	//		break;

		default :
			break;
	}
}

/*
 * From 0 speed always
 */
static inline void Motor_SixStep_StartPhaseControl(Motor_T * p_motor)
{
	Motor_SixStep_ResumePhaseControl(p_motor);
	Encoder_Reset(&p_motor->Encoder);
	/* soften start from initial speed calc
	 * 625khz Timer, 0xFFFF counter,  60 polepairs,  9 rpm is lowest read value
	 */
	Encoder_DeltaT_SetInitial(&p_motor->Encoder, 10U);

//	p_motor->Bemf.ZeroCrossingCounter  = 0U;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP :
			Motor_SixStep_StartOpenLoop(p_motor);
			break;

		case MOTOR_SENSOR_MODE_SENSORLESS:
			//	 bbemf mode must change timer period from openloop rpm to 1 tick
			//if reliable 	BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_COMMUTATION);
//			BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_STARTUP);
//			Motor_SixStep_StartOpenLoop(p_motor);
	//		p_motor->IsOpenLoopStartUp = true;
			break;

		case MOTOR_SENSOR_MODE_HALL:
			Hall_ResetCapture(&p_motor->Hall);
			break;

	//	case MOTOR_SENSOR_MODE_ENCODER:
	//		resume =  true;
	//		break;

		default :
			break;
	}
}

//void Motor_SixStep_Jog(Motor_T *p_motor)
//{
//	if (p_motor->JogSteps)
//	{
//		p_motor->JogSteps--;
//	Motor_SixStep_ActivateSector(p_motor, p_motor->NextSector);
//	}
//}


#endif
