#ifndef MOT_PROTOCOL_H
#define MOT_PROTOCOL_H

#include "Motor/Motor/Motor.h"

#include <stdint.h>
#include <stdbool.h>

/*
 * Input Write Registers
 */

typedef struct
{
//	volatile bool OnOffDirection;

	//	volatile bool SwitchBrake;
	//	volatile bool SwitchThrottle;
	//	volatile bool SwitchForward;
	//	volatile bool SwitchReverse;
	//	volatile bool SwitchNeutral;
	//	volatile bool IsThrottleRelease;

	volatile uint16_t ValueThrottle;
	//	volatile uint16_t ValueThrottlePrev;
	volatile uint16_t ValueBrake;
	//	volatile uint16_t ValueBrakePrev;

//	volatile bool DirectionForward;
}
MotProtocol_InputMotor_T;

typedef enum
{
	 MOT_PROTOCOL_INPUT_PROCESS_A,
} MotProtocol_InputProcess_T;

typedef struct
{
	MotProtocol_InputProcess_T FunctionCmd; //converted function cmd

	MotProtocol_InputMotor_T Motors[1];
	bool UseMain[1]; //write main input or per motor input

//	volatile bool MainSwitchBrake;
//	volatile bool MainSwitchThrottle;
//	volatile bool MainSwitchForward;
//	volatile bool MainSwitchReverse;
//	volatile bool MainSwitchNeutral;
//	volatile bool MainIsThrottleRelease;

	volatile uint16_t MainValueThrottle;
//	volatile uint16_t MainValueThrottlePrev;
	volatile uint16_t MainValueBrake;
//	volatile uint16_t MainValueBrakePrev;

	volatile bool MainDirectionForward;

	volatile bool AuxSafety;

	volatile bool SwitchShutdown;
	volatile bool SwitchCoil;
	volatile bool SettingUseX;

	//flash context
}
MotProtocol_Input_T;


/*
 * Output Read Registers
 */

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
//MotProtocol_BoardAdcMap_T;

//limit scope at compile time, use primitives
typedef const struct
{
//	volatile const adc_t * const P_VBUS_ADCU;
	volatile const adc_t * const P_VA_ADCU;
	volatile const adc_t * const P_VB_ADCU;
	volatile const adc_t * const P_VC_ADCU;
	volatile const adc_t * const P_IA_ADCU;
	volatile const adc_t * const P_IB_ADCU;
	volatile const adc_t * const P_IC_ADCU;
	volatile const adc_t * const P_HEAT_MOTOR_ADCU;
//	volatile const adc_t * const P_HEAT_MOSFETS_ADCU; //if per motor mosfet sensor is implemented
//	const Motor_Parameters_T * const P_PARAMETERS;
	volatile const uint16_t * const P_SPEED_RPM;
}
MotProtocol_OutputMotorMap_T;

typedef const struct
{
	const MotProtocol_OutputMotorMap_T MOTOR_MAPS[1];

//	const MotProtocol_BoardAdcMap_T BOARD_MAP;
	volatile const adc_t * const P_VBUS_ADCU;
	volatile const adc_t * const P_VACC_ADCU;
	volatile const adc_t * const P_VSENSE_ADCU;
	volatile const adc_t * const P_HEAT_PCB_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_H_ADCU;
	volatile const adc_t * const P_HEAT_MOSFETS_L_ADCU;
	volatile const adc_t * const P_THROTTLE_ADCU;
	volatile const adc_t * const P_BRAKE_ADCU;

	Linear_T * P_UNIT_VBUS;

	const Motor_T * const  P_MOTORS; // temporary, read-only
}
MotProtocol_OutputMap_T;

typedef struct
{
//	const Motor_T * p_Motor; // temporary, read-only
//	MotProtocol_OutputMotorMap_T * p_Map;

	//processed outputs;
	/* UI Unit report */
	volatile uint16_t VBus_mV;
	volatile uint16_t VBemfPeak_mV;
	volatile uint16_t VBemfA_mV;
	volatile uint16_t IBus_Amp;
}
MotProtocol_OutputMotor_T;

typedef struct
{
	MotProtocol_OutputMotor_T Motors[1]; 	//processed outputs;
	const MotProtocol_OutputMap_T * p_Map;

//	const MotProtocol_OutputMotorMap_T * p_Maps;
//	volatile const MotAnalogMonitor_AdcMap_T * p_BoardAdcMap;
//	Linear_T * p_UnitVBus;

	volatile uint16_t BoardVBus_mV;
	volatile uint16_t BoardITotal_Amp;
}
MotProtocol_Output_T;

typedef const struct
{
//	const Motor_T * const  P_MOTORS; // temporary, read-only
//	const uint8_t MOTORS_COUNT;
//	const MotAnalogMonitor_AdcMap_T * const P_ADC_MAP_BOARD;
	const MotProtocol_OutputMap_T OUTPUT_MAP;
}
MotProtocol_Config_T;

#endif
