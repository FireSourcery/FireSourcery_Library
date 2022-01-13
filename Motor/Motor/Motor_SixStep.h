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
    		Six Step commutation strategy. Polar PWM

    		Current design - Only SixStep module tracks phase

    @version V0
*/
/******************************************************************************/
#ifndef MOTOR_SIXSTEP_H
#define MOTOR_SIXSTEP_H

#include "Motor.h"
#include "Config.h"

#include "Transducer/Hall/Hall.h"
#include "Transducer/BEMF/BEMF.h"
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


#include "Utility/Debug/Debug.h"

/*
 * On Adc measure of phase complete
 */
static inline void Motor_SixStep_CaptureBemfA(Motor_T * p_motor)
{
	if(BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
		if (p_motor->IsPwmOn == true)
		{
			Debug_CaptureElapsed(8);
			Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
		}
		else
		{
			Debug_CaptureElapsed(7);
			Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);
		}
	}

//	(p_motor->IsPwmOn == false) ?
//		Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU) :
//		Bemf_PollCaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Va_ADCU);

	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
}

static inline void Motor_SixStep_CaptureBemfB(Motor_T * p_motor)
{
	if(BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
		if (p_motor->IsPwmOn == true)
		{
			Debug_CaptureElapsed(8);
			Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU);
		}
		else
		{
			Debug_CaptureElapsed(7);
			Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU);
		}
	}

//	(p_motor->IsPwmOn == false) ?
//		Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU) :
//		Bemf_PollCaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vb_ADCU);

	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
}

static inline void Motor_SixStep_CaptureBemfC(Motor_T * p_motor)
{
	if(BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{
		if (p_motor->IsPwmOn == true)
		{
			Debug_CaptureElapsed(8);
			Bemf_CaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU);
		}
		else
		{
			Debug_CaptureElapsed(7);
			Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU);
		}
	}

//	(p_motor->IsPwmOn == false) ?
//		Bemf_CaptureVPhasePwmOff(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU) :
//		Bemf_PollCaptureVPhase(&p_motor->Bemf, p_motor->AnalogResults.Vc_ADCU);

	Bemf_CaptureVPos(&p_motor->Bemf, p_motor->AnalogResults.VPos_ADCU);
}

static inline void Motor_SixStep_CaptureIBusA(Motor_T * p_motor)
{
	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIa,  p_motor->AnalogResults.Ia_ADCU);

	//Filter here if needed
	if (p_motor->IsPwmOn == true)
	{
		Debug_CaptureElapsed(10);
		p_motor->IBusPwmOn_Frac16 = p_motor->IBus_Frac16;
	}
	else
	{
		Debug_CaptureElapsed(9);
		p_motor->IBusPwmOff_Frac16 = p_motor->IBus_Frac16;
	}

//	p_motor->IBus_ADCU  Filter_MovAvg(&p_motor->FilterIa, p_motor->AnalogChannelResults[MOTOR_ANALOG_CHANNEL_IA]);
}

static inline void Motor_SixStep_CaptureIBusB(Motor_T * p_motor)
{
	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIb, p_motor->AnalogResults.Ib_ADCU);

	if (p_motor->IsPwmOn == true)
	{
		Debug_CaptureElapsed(10);
		p_motor->IBusPwmOn_Frac16 = p_motor->IBus_Frac16;
	}
	else
	{
		Debug_CaptureElapsed(9);
		p_motor->IBusPwmOff_Frac16 = p_motor->IBus_Frac16;
	}
}

static inline void Motor_SixStep_CaptureIBusC(Motor_T * p_motor)
{
	p_motor->IBus_Frac16 = Linear_ADC_CalcFractionUnsigned16_Abs(&p_motor->UnitIc, p_motor->AnalogResults.Ic_ADCU);

	if (p_motor->IsPwmOn == true)
	{
		Debug_CaptureElapsed(10);
		p_motor->IBusPwmOn_Frac16 = p_motor->IBus_Frac16;
	}
	else
	{
		Debug_CaptureElapsed(9);
		p_motor->IBusPwmOff_Frac16 = p_motor->IBus_Frac16;
	}
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
 	switch (p_motor->CommutationPhase)
	{
		case MOTOR_PHASE_ERROR_0:
			//set error
			break;

		case MOTOR_PHASE_AC:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_BC; BEMF_MapCcwPhaseAC(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_AB; BEMF_MapCwPhaseAC(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_BC:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_BA; BEMF_MapCcwPhaseBC(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_AC; BEMF_MapCwPhaseBC(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_BA:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_CA; BEMF_MapCcwPhaseBA(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_BC; BEMF_MapCwPhaseBA(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_CA:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_CB; BEMF_MapCcwPhaseCA(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_BA; BEMF_MapCwPhaseCA(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_CB:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_AB; BEMF_MapCcwPhaseCB(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_CA; BEMF_MapCwPhaseCB(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_AB:
			if (p_motor->Direction == MOTOR_DIRECTION_CCW) 	{p_motor->NextPhase = MOTOR_PHASE_AC; BEMF_MapCcwPhaseAB(&p_motor->Bemf);}
			else											{p_motor->NextPhase = MOTOR_PHASE_CB; BEMF_MapCwPhaseAB(&p_motor->Bemf);}
			break;

		case MOTOR_PHASE_ERROR_7:
			//set error
			break;

		default:
			break;
	}

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
static inline void ActivateMotorSixStepPhase(Motor_T * p_motor)
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

static inline void ActivateMotorSixStepAnalogPhase(Motor_T * p_motor, const AnalogN_Conversion_T * p_vPhase, const AnalogN_Conversion_T * p_iPhase)
{
	Debug_CaptureElapsed(4);

	p_motor->IsPwmOn = false;

	AnalogN_PauseQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

	AnalogN_EnqueueConversionOptions_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_RESTORE);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_vPhase);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_iPhase);

	//should not start until middle of pwm cycle
	AnalogN_EnqueueConversionOptions_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_PWM_ON);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_vPhase);
	AnalogN_EnqueueConversionOptions_Group(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_OPTION_RESTORE);
	AnalogN_EnqueueConversion_Group(p_motor->CONFIG.P_ANALOG_N, p_iPhase);
//	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VPOS);

	AnalogN_ResumeQueue(p_motor->CONFIG.P_ANALOG_N, p_motor->CONFIG.ADCS_ACTIVE_PWM_THREAD);

	Debug_CaptureElapsed(5);
}

/*
 * All cases
 */
static inline void ActivateMotorSixStepAnalog(Motor_T * p_motor)
{

//	if (BEMF_CheckBlankTime(&p_motor->Bemf) == true)
	{


//		switch (p_motor->CommutationPhase)
//		{
//			case MOTOR_PHASE_ERROR_0: break;
//			case MOTOR_PHASE_AC: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_B); break;
//			case MOTOR_PHASE_BC: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A); break;
//			case MOTOR_PHASE_BA: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_C); break;
//			case MOTOR_PHASE_CA: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_B); break;
//			case MOTOR_PHASE_CB: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_A); break;
//			case MOTOR_PHASE_AB: AnalogN_EnqueueFrontConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_BEMF_C); break;
//			case MOTOR_PHASE_ERROR_7: break;
//			default: break;
//		}

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
		//switch (CurrentSensingMode){}
}

/*
 * Open Loop Functions
 */
static inline bool Motor_SixStep_PollOpenLoop(Motor_T * p_motor)
{
	bool commutation = Timer_Poll(&p_motor->ControlTimer);

	if (commutation == true)
	{
		p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTargetIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, p_motor->OpenLoopCommutationPeriod);
		p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
		Timer_SetPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);
	}

	return commutation;
}

static inline void Motor_SixStep_StartOpenLoop(Motor_T * p_motor)
{

	// if pwm update is only on commutation cycle, must start at sufficient speed for timer driven fixed angle displacements
//	Linear_Ramp_Init_Millis(&p_motor->OpenLoopRamp, 20000U, p_motor->Parameters.OpenLoopSpeedStart, p_motor->Parameters.OpenLoopSpeedFinal, 1000U);
	Linear_Ramp_InitSlope(&p_motor->OpenLoopRamp, 20000U, p_motor->Parameters.OpenLoopSpeedStart, p_motor->Parameters.OpenLoopSpeedFinal, 100U);
	p_motor->OpenLoopRampIndex = 0U;

	p_motor->OpenLoopSpeed_RPM = Linear_Ramp_CalcTargetIncIndex(&p_motor->OpenLoopRamp, &p_motor->OpenLoopRampIndex, 0U);
	p_motor->OpenLoopCommutationPeriod = Encoder_Motor_ConvertMechanicalRpmToInterpolationFreq(&p_motor->Encoder, p_motor->OpenLoopSpeed_RPM);
	Timer_StartPeriod(&p_motor->ControlTimer, p_motor->OpenLoopCommutationPeriod);

	//motor aligned to phase A
	p_motor->NextPhase = (p_motor->Direction == MOTOR_DIRECTION_CCW) ? MOTOR_PHASE_BC : MOTOR_PHASE_AB;
}

static inline void Motor_SixStep_ProcOpenLoop(Motor_T * p_motor)
{
	p_motor->VPwm = p_motor->RampCmd; //p_motor->Parameters.OpenLoopVPwm_Frac16 +

	// bound to 2.5 to 10 percent
	if 		(p_motor->VPwm < p_motor->Parameters.OpenLoopVPwmMin)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMin;}
	else if (p_motor->VPwm > p_motor->Parameters.OpenLoopVPwmMax)	{ p_motor->VPwm = p_motor->Parameters.OpenLoopVPwmMax;}

	if(Motor_SixStep_PollOpenLoop(p_motor) == true)
	{
		p_motor->CommutationPhase = p_motor->NextPhase;

//		Encoder_DeltaT_Capture(&p_motor->Encoder);
//		Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);

		MapMotorSixStepBemfPhase(p_motor);
		BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference
		ActivateMotorSixStepPhase(p_motor);
	}
	else
	{
		BEMF_ProcZeroCrossingDetection(&p_motor->Bemf); //  poll zcd with prev capture
	}

	ActivateMotorSixStepAnalog(p_motor);
}

static inline bool Motor_SixStep_GetBemfReliable(Motor_T * p_motor) {return BEMF_GetIsReliable(&p_motor->Bemf);}

static inline bool PollMotorSixStepCommutation(Motor_T * p_motor)
{
	bool commutation = false;

	switch(p_motor->Parameters.SensorMode)
	{
//		case MOTOR_SENSOR_MODE_OPEN_LOOP :
//			if(Motor_SixStep_PollOpenLoop(p_motor) == true)
//			{
//				p_motor->CommutationPhase = p_motor->NextPhase;
//				commutation = true;
//			}
//			break;

		case MOTOR_SENSOR_MODE_BEMF:
//			if(Motor_SixStep_GetBemfReliable(p_motor) == true)
//			{
				if(BEMF_CheckPhasePeriod(&p_motor->Bemf) == true)
				{
					p_motor->CommutationPhase = p_motor->NextPhase;

					if (BEMF_GetIsZeroCrossingDetectionComplete(&p_motor->Bemf) == true)
					{
						Encoder_DeltaT_Capture(&p_motor->Encoder);
						Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);
					}

					MapMotorSixStepBemfPhase(p_motor);
					BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference
					commutation = true;
				}
				else
				{
					BEMF_ProcZeroCrossingDetection(&p_motor->Bemf); //  poll zcd with prev capture
				}
//			}
			break;

		case MOTOR_SENSOR_MODE_HALL :
			if(Hall_PollCaptureSensors(&p_motor->Hall) == true)
			{
				p_motor->CommutationPhase = (Motor_SectorId_T)Hall_GetCommutationId(&p_motor->Hall); //hall id match motor id
 				Encoder_DeltaT_Capture(&p_motor->Encoder);
				Encoder_DeltaT_CaptureExtendedTimer(&p_motor->Encoder);

				MapMotorSixStepBemfPhase(p_motor);
				BEMF_CapturePhaseReference(&p_motor->Bemf); //set reference
				commutation = true;
			}
			else
			{
				BEMF_ProcZeroCrossingDetection(&p_motor->Bemf); // debug
			}
			break;

//	case MOTOR_SENSOR_MODE_ENCODER:	 //encoder use foc
//		break;

		default :
			break;
	}

	return commutation;
}


/*
 * State loop in 20khz pwm thread
 * Active Control
 */
static inline void Motor_SixStep_ProcPhaseControl(Motor_T * p_motor)
{
	ActivateMotorSixStepAnalog(p_motor);

	if (Motor_PollSpeedFeedback(p_motor) == true)
	{
		if (Encoder_DeltaT_PollWatchStop(&p_motor->Encoder) == true) //once per millis
		{
			p_motor->Speed_RPM = 0U;
		}
	}

	switch(p_motor->Parameters.ControlMode)
	{
		//	case MOTOR_CONTROL_MODE_OPEN_LOOP:
		//		p_motor->VPwm = p_motor->UserCmd / 4U;
		//		break;

		case MOTOR_CONTROL_MODE_CONSTANT_VOLTAGE :
			//on overcurrent set outmax
			if (p_motor->IBus_Frac16 > 58982U)
			{
//				p_motor->VPwm = p_motor->VPwm - ((p_motor->IBus_Frac16 - 58982U) * p_motor->VPwm >> 16U)
				p_motor->VPwm = PID_Calc(&p_motor->PidIBus, 58982U, p_motor->IBus_Frac16);
			}
			else
			{
				PID_Calc(&p_motor->PidIBus, 58982U, p_motor->IBus_Frac16);
				p_motor->VPwm = p_motor->RampCmd;
			}
			break;

		case MOTOR_CONTROL_MODE_SCALAR_VOLTAGE_FREQ :
			//p_motor->VPwm = p_motor->RampCmd * p_motor->Speed_RPM * p_motor->VRpmGain;
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_VOLTAGE :
			if (p_motor->IBus_Frac16 > 58982U)
			{
				p_motor->VPwm = PID_Calc(&p_motor->PidIBus, 58982U, p_motor->IBus_Frac16);
			}
			else
			{
				PID_Calc(&p_motor->PidIBus, 58982U, p_motor->IBus_Frac16);
				p_motor->VPwm = p_motor->SpeedControl;
			}
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_CURRENT :
			p_motor->VPwm = PID_Calc(&p_motor->PidIBus, p_motor->RampCmd, p_motor->IBus_Frac16);
			break;

		case MOTOR_CONTROL_MODE_CONSTANT_SPEED_CURRENT :
			p_motor->VPwm = PID_Calc(&p_motor->PidIBus, p_motor->SpeedControl, p_motor->IBus_Frac16);
			break;
		default :
			break;
	}

	if(PollMotorSixStepCommutation(p_motor) == true)
	{
		ActivateMotorSixStepPhase(p_motor);
	}
	else
	{
		ActivateMotorSixStepPhaseDuty(p_motor); /* update pwm with control variable value every pwm cycle */
	}
}


static inline void Motor_SixStep_ResumePhaseControl(Motor_T * p_motor)
{
	PID_Reset(&p_motor->PidSpeed);
	PID_Reset(&p_motor->PidIBus);

	switch(p_motor->Parameters.SensorMode)
	{
//		case MOTOR_SENSOR_MODE_OPEN_LOOP :
//			break;

		case MOTOR_SENSOR_MODE_BEMF:
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

	BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_COMMUTATION);
}

static inline bool Motor_SixStep_CheckResumePhaseControl(Motor_T * p_motor)
{
	bool resume = false;

	switch(p_motor->Parameters.SensorMode)
	{
		case MOTOR_SENSOR_MODE_OPEN_LOOP :
//			resume = false;
//			break;

		case MOTOR_SENSOR_MODE_BEMF :
			resume = Motor_SixStep_GetBemfReliable(p_motor);
			break;

		case MOTOR_SENSOR_MODE_HALL :
			resume = true;
			break;

//	case MOTOR_SENSOR_MODE_ENCODER:
//		resume =  true;
//		break;

		default :
			break;
	}

	return resume;
}

/*
 * From 0 speed always
 */
static inline void Motor_SixStep_StartPhaseControl(Motor_T * p_motor)
{
	PID_Reset(&p_motor->PidSpeed);
	PID_Reset(&p_motor->PidIBus);

	Encoder_Reset(&p_motor->Encoder);

//	Encoder_DeltaT_SetInitial(&p_motor->Encoder, 5U);
	Timer_StartPeriod(&p_motor->ControlTimer, 20U);

	p_motor->Bemf.ZeroCrossingCounter  = 0U;

	switch (p_motor->Parameters.SensorMode)
	{
//	case MOTOR_SENSOR_MODE_OPEN_LOOP:
//		Motor_SixStep_StartOpenLoop(p_motor);
//		break;

	case MOTOR_SENSOR_MODE_BEMF:
		BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_STARTUP);
		Motor_SixStep_StartOpenLoop(p_motor);
//			p_motor->IsOpenLoopStartUp = true;
		break;

	case MOTOR_SENSOR_MODE_HALL:
		BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_COMMUTATION);
		Hall_ResetCapture(&p_motor->Hall);
		break;

//	case MOTOR_SENSOR_MODE_ENCODER: //encoder always use foc
//		break;

	default:
		break;
	}
}

/*
 * Passive Observe
 */
static inline void Motor_SixStep_ProcPhaseObserve(Motor_T * p_motor)
{
	PollMotorSixStepCommutation(p_motor);
	ActivateMotorSixStepAnalog(p_motor);
}

static inline void Motor_SixStep_StartPhaseObserve(Motor_T * p_motor)
{
	BEMF_SetCycleMode(&p_motor->Bemf, BEMF_CYCLE_MODE_PASSIVE); //no blank time
}

//extern void Motor_SixStep_CaptureBemfA(Motor_T * p_motor);
//extern void Motor_SixStep_CaptureBemfB(Motor_T * p_motor);
//extern void Motor_SixStep_CaptureBemfC(Motor_T * p_motor);

//void Motor_SixStep_Jog(Motor_T *p_motor)
//{
//	if (p_motor->JogSteps)
//	{
//		p_motor->JogSteps--;
//	Motor_SixStep_ActivateSector(p_motor, p_motor->NextSector);
//	}
//}


/*
 * Calibrate Current ADC
 */
static inline void Motor_SixStep_StartCalibrateAdc(Motor_T * p_motor)
{
	Timer_StartPeriod(&p_motor->ControlTimer, 20000U); // Motor.Parameters.AdcCalibrationTime
	Phase_Ground(&p_motor->Phase); //activates abc
	p_motor->CalibrationSubstateStep = 0U;

	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
	AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);

	Filter_MovAvg_InitN(&p_motor->FilterA, 2048U, 100U);
	Filter_MovAvg_InitN(&p_motor->FilterB, 2048U, 100U);
	Filter_MovAvg_InitN(&p_motor->FilterC, 2048U, 100U);
}

static inline bool Motor_SixStep_CalibrateAdc(Motor_T *p_motor)
{
//	bool isComplete = false;

//	 Timer_Poll(&p_motor->ControlTimer);

//	switch(p_motor->CalibrationSubstateStep)
//	{
//		case 0U :
//			Filter_MovAvg_InitN(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU, 1000U);
//			Filter_MovAvg_InitN(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU, 1000U);
//			Filter_MovAvg_InitN(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU, 1000U);
//			p_motor->CalibrationSubstateStep = 1U;
//			break;
//
//		case 1U:
//			if (Timer_Poll(&p_motor->ControlTimer) == false)
//			{
//				p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
//				p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
//				p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
//
//				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//			}
//			else
//			{
//				p_motor->CalibrationSubstateStep = 2U;
//			}
//			break;
//
//		case 2U:
//
//			if (Timer_Poll(&p_motor->ControlTimer) == true)
//			{
//				Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//				Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//				Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
//				p_motor->CalibrationSubstateStep = 4U;
//			}
//			else
//			{
//				p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
//				p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
//				p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);
//
//				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
//				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
//				AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
//			}
//
//			//get bemf offset
////		case 3U:
////			Filter_MovAvg_InitN(&p_motor->FilterA, p_motor->AnalogResults.Va_ADCU, 1000U);
////			Filter_MovAvg_InitN(&p_motor->FilterB, p_motor->AnalogResults.Vb_ADCU, 1000U);
////			Filter_MovAvg_InitN(&p_motor->FilterC, p_motor->AnalogResults.Vc_ADCU, 1000U);
//
//		case 4U:
//
//			isComplete = true;
//
//
//		default :
//			break;
//	}

	bool isComplete = Timer_Poll(&p_motor->ControlTimer);

	if (isComplete == true)
	{
		Linear_ADC_Init(&p_motor->UnitIa, p_motor->Parameters.IaZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Linear_ADC_Init(&p_motor->UnitIb, p_motor->Parameters.IbZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Linear_ADC_Init(&p_motor->UnitIc, p_motor->Parameters.IcZero_ADCU, 4095U, p_motor->Parameters.Imax_Amp);
		Phase_Float(&p_motor->Phase);
//		save params
	}
	else
	{
		p_motor->Parameters.IaZero_ADCU = Filter_MovAvg(&p_motor->FilterA, p_motor->AnalogResults.Ia_ADCU);
		p_motor->Parameters.IbZero_ADCU = Filter_MovAvg(&p_motor->FilterB, p_motor->AnalogResults.Ib_ADCU);
		p_motor->Parameters.IcZero_ADCU = Filter_MovAvg(&p_motor->FilterC, p_motor->AnalogResults.Ic_ADCU);

		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_IC);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VA);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VB);
		AnalogN_EnqueueConversion(p_motor->CONFIG.P_ANALOG_N, &p_motor->CONFIG.CONVERSION_VC);
	}

	return isComplete;
}


#endif
