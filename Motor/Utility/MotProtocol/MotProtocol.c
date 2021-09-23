//motor protocol registers access

#include "MotProtocol.h"

//#include "Motor/Motor/Motor.h"
//
//#include "Math/Linear/Linear_ADC.h"
#include "Math/Linear/Linear_Voltage.h"
#include "Math/Linear/Linear.h"

#include <stdint.h>
#include <stdbool.h>


void MotProtocol_Input_Init(MotProtocol_Input_T * p_input, const MotProtocol_Config_T * p_config)
{

}

void MotProtocol_Input_SetMotorThrottle(MotProtocol_Input_T * p_input, uint8_t motorId, uint16_t throttle)
{
	p_input->Motors[motorId].ValueThrottle = throttle;
}

void MotProtocol_Input_SetMainThrottle(MotProtocol_Input_T * p_input, uint16_t throttle)
{
//	p_input->MainValueThrottlePrev 	= p_input->MainValueThrottle;
	p_input->MainValueThrottle 		= throttle;

//	p_input->InputIsThrottleRelease = CheckThrottleRelease(p_input->MainValueThrottle, p_input->MainValueThrottlePrev);
}

void MotProtocol_Input_SetMainBrake(MotProtocol_Input_T * p_input, uint16_t throttle)
{
//	p_input->MainValueThrottlePrev 	= p_input->MainValueThrottle;
	p_input->MainValueThrottle 		= throttle;
}



void MotProtocol_Output_Init(MotProtocol_Output_T * p_output, const MotProtocol_Config_T * p_config)
{
	p_output->p_Map = &p_config->OUTPUT_MAP;

//	p_output->p_BoardAdcMap = p_config->P_ADC_MAP_BOARD;
//
//	for(uint8_t iMotor = 0U; iMotor < p_config->MOTORS_COUNT; iMotor++)
//	{
//		p_output->Motors[iMotor].p_Motor = &p_config->P_MOTORS[iMotor]; //temp
//		p_output->Motors[iMotor].p_Map = &p_config->OUTPUT_MAPS[iMotor];
//	}
}

void MotProtocol_Output_GetBoardVBus_mV			(const MotProtocol_Output_T * p_output){return p_output->BoardVBus_mV;}
void MotProtocol_Output_GetBoardVBus_ADCU		(const MotProtocol_Output_T * p_output){return *p_output->p_Map->P_VBUS_ADCU;}
void MotProtocol_Output_GetMotorVBemfPeak_mV	(const MotProtocol_Output_T * p_output, uint8_t motorId){return p_output->Motors[motorId].VBemfPeak_mV;}

void MotProtocol_Output_CaptureBoard(MotProtocol_Output_T * p_output)
{
	p_output->BoardVBus_mV 		= Linear_Voltage_CalcMilliV(p_output->p_Map->P_UNIT_VBUS, *p_output->p_Map->P_VBUS_ADCU);
}

//set from motor read only
//void MotProtocol_Output_SetMotorVBemfPeak_mV	(const MotProtocol_Output_T * p_output, uint16_t value){p_output->Motors[motorId].VBemfPeak_mV = value;}
