extern bool Hall_CalibrateSensorsTable
(
	Hall_T * p_hall,
	void (*activatePhaseABC)(void * p_context, uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	void * p_context
);


/*
 * User enable all 3 phases prior to running
 */
//bool Hall_CalibrateSensorsTable
//(
//	Hall_T * p_hall,
//	void (*activatePhaseABC)(void * p_context, uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
//	void * p_context
//)
//{
//	static uint8_t state = 0U; /* limits calibration to 1 at a time, no multi thread / multi motor calibration */
//	uint16_t pwm = 65536U/20U; //p_hall->CONFIG.CALIBRATION_PWM;
//	bool isComplete = false;
//
//	//offset states for wait
//	switch (state)
//	{
//	case 0U: activatePhaseABC(p_context, pwm, 0, 0); 	Hall_CalibratePhaseA(p_hall);		state++; 	break;
//	case 1U: activatePhaseABC(p_context, pwm, pwm, 0); 	Hall_CalibratePhaseInvC(p_hall);	state++; 	break;
//	case 2U: activatePhaseABC(p_context, 0, pwm, 0);	Hall_CalibratePhaseB(p_hall);		state++;	break;
//	case 3U: activatePhaseABC(p_context, 0, pwm, pwm);	Hall_CalibratePhaseInvA(p_hall);	state++;	break;
//	case 4U: activatePhaseABC(p_context, 0, 0, pwm);	Hall_CalibratePhaseC(p_hall);		state++; 	break;
//	case 5U: activatePhaseABC(p_context, pwm, 0, pwm);	Hall_CalibratePhaseInvB(p_hall);	state = 0U;	isComplete = true; break;
//	default: break;
//	}
//
//	return isComplete;
//}

/*

 */
//void Hall_CalibrateCommuntationTable_Blocking
//(
//	Hall_T * p_hall,
//
//	Hall_CommutationPhase_T phaseAC,
//	Hall_CommutationPhase_T phaseBC,
//	Hall_CommutationPhase_T phaseBA,
//	Hall_CommutationPhase_T phaseCA,
//	Hall_CommutationPhase_T phaseCB,
//	Hall_CommutationPhase_T phaseAB,
//
//	void (*activatePwmValuePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
//	uint16_t pwm,
//	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC),
//
//	void (*delay)(uint32_t time),
//	uint32_t delayTime
//)
//{
//	if(activatePwmStatePhaseABC) {activatePwmStatePhaseABC(1,1,1);}
//
//	/*
//	 * Calibrating for CCW, while rotating CCW, if ABC are connected as intended
//	 */
//	activatePwmValuePhaseABC(pwm, 0, 0);
//	delay(delayTime);
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseBC;
////	if (returnIndexBC) *returnIndexBC = Hall_ReadSensors(p_hall->p_HAL_Hall);
//
//	activatePwmValuePhaseABC(pwm, pwm, 0);
//	delay(delayTime);
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseBA;
////	if (returnIndexBA) *returnIndexBA = Hall_ReadSensors(p_hall->p_HAL_Hall);
//
//	activatePwmValuePhaseABC(0, pwm, 0);
//	delay(delayTime);
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseCA;
////	if (returnIndexCA) *returnIndexCA = Hall_ReadSensors(p_hall->p_HAL_Hall);
//
//	activatePwmValuePhaseABC(0, pwm, pwm);
//	delay(delayTime);
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseCB;
////	if (returnIndexCB) *returnIndexCB = Hall_ReadSensors(p_hall->p_HAL_Hall);
//
//	activatePwmValuePhaseABC(0, 0, pwm);
//	delay(delayTime);
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseAB;
////	if (returnIndexAB) *returnIndexAB = Hall_ReadSensors(p_hall->p_HAL_Hall);
//
//	activatePwmValuePhaseABC(pwm, 0, pwm);
//	delay(delayTime);
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall) ] = phaseAC;
////	if (returnIndexAC) *returnIndexAC = Hall_ReadSensors(p_hall->p_HAL_Hall);
//
//	if (activatePwmStatePhaseABC){activatePwmStatePhaseABC(0,0,0);}
//}



# if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
void Hall_CalibrateCommuntationTable
(
	Hall_T * p_hall,
	Hall_CommutationId_T phaseAC,
	Hall_CommutationId_T phaseBC,
	Hall_CommutationId_T phaseBA,
	Hall_CommutationId_T phaseCA,
	Hall_CommutationId_T phaseCB,
	Hall_CommutationId_T phaseAB,

	void (*activatePhaseABC)(uint16_t pwmA, uint16_t pwmB, uint16_t pwmC),
	uint16_t pwm,
	void (*activatePwmStatePhaseABC)(bool enA, bool enB, bool enC)
)

/*
 * In cases where Hall_CommutationId_T is a function pointer
 */
void Hall_MapCommuntationTable_PhaseDefault
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T phaseAC,
	Hall_CommutationPhase_T phaseBC,
	Hall_CommutationPhase_T phaseBA,
	Hall_CommutationPhase_T phaseCA,
	Hall_CommutationPhase_T phaseCB,
	Hall_CommutationPhase_T phaseAB
)
{
//	/*
//	 * Calibrating for CW
//	 */
//	Hall_MapCommuntationTable
//	(
//		p_hall,
//		phaseAC,
//		phaseBC,
//		phaseBA,
//		phaseCA,
//		phaseCB,
//		phaseAB,
//		HALL_SENSORS_B,
//		HALL_SENSORS_NOT_A,
//		HALL_SENSORS_C,
//		HALL_SENSORS_NOT_B,
//		HALL_SENSORS_A,
//		HALL_SENSORS_NOT_C
//	);
	p_hall->CommuntationTable[sensorIndexPhaseAC] = phaseAC;
	p_hall->CommuntationTable[sensorIndexPhaseBC] = phaseBC;
	p_hall->CommuntationTable[sensorIndexPhaseBA] = phaseBA;
	p_hall->CommuntationTable[sensorIndexPhaseCA] = phaseCA;
	p_hall->CommuntationTable[sensorIndexPhaseCB] = phaseCB;
	p_hall->CommuntationTable[sensorIndexPhaseAB] = phaseAB;

	p_hall->SensorsTable[sensorIndexPhaseAC] = HALL_VIRTUAL_SENSORS_INV_B;
	p_hall->SensorsTable[sensorIndexPhaseBC] = HALL_VIRTUAL_SENSORS_A;
	p_hall->SensorsTable[sensorIndexPhaseBA] = HALL_VIRTUAL_SENSORS_INV_C;
	p_hall->SensorsTable[sensorIndexPhaseCA] = HALL_VIRTUAL_SENSORS_B;
	p_hall->SensorsTable[sensorIndexPhaseCB] = HALL_VIRTUAL_SENSORS_INV_A;
	p_hall->SensorsTable[sensorIndexPhaseAB] = HALL_VIRTUAL_SENSORS_C;
	/*
	 * Calibrating for CCW
	 */
	Hall_MapCommuntationTable
	(
		p_hall,
		phaseAC,
		phaseBC,
		phaseBA,
		phaseCA,
		phaseCB,
		phaseAB,
		HALL_SENSORS_NOT_B,
		HALL_SENSORS_A,
		HALL_SENSORS_NOT_C,
		HALL_SENSORS_B,
		HALL_SENSORS_NOT_A,
		HALL_SENSORS_C
	);
}

void Hall_MapCommuntationTableFaultStates
(
	Hall_T * p_hall,
	Hall_CommutationPhase_T fault000,
	Hall_CommutationPhase_T fault111
)
{
	p_hall->CommuntationTable[0] = fault000;
	p_hall->CommuntationTable[7] = fault111;
}
//void Hall_CalibrateSensorAPhaseBC(Hall_T * p_hall, Hall_CommutationPhase_T phaseBC)
//{
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseBC;
//}
//
//void Hall_CalibrateSensorInvCPhaseBA(Hall_T * p_hall, Hall_CommutationPhase_T phaseBA)
//{
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseBA;
//}
//
//void Hall_CalibrateSensorBPhaseCA(Hall_T * p_hall, Hall_CommutationPhase_T phaseCA)
//{
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseCA;
//}
//
//void Hall_CalibrateSensorInvAPhaseCB(Hall_T * p_hall, Hall_CommutationPhase_T phaseCB)
//{
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseCB;
//}
//
//void Hall_CalibrateSensorCPhaseAB(Hall_T * p_hall, Hall_CommutationPhase_T phaseAB)
//{
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseAB;
//}
//
//void Hall_CalibrateSensorInvBPhaseAC(Hall_T * p_hall, Hall_CommutationPhase_T phaseAC)
//{
//	p_hall->CommuntationTable[Hall_ReadSensors(p_hall)] = phaseAC;
//}
#endif


#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)

	static inline void Hall_ProcCommutation_ISR(Hall_T * p_hall)
	{
		if (p_hall->Direction == HALL_DIRECTION_CW)
		{
			p_hall->CommuntationTable[InverseHall(Hall_ReadSensors(p_hall))](p_hall->p_UserData);
		}
		else
		{
			p_hall->CommuntationTable[Hall_ReadSensors(p_hall)](p_hall->p_UserData);
		}
	}

	static inline bool Hall_PollCommutation(Hall_T * p_hall)
	{
		bool isEdge;

		uint8_t hall = Hall_ReadSensors(p_hall);

		if (hall != p_hall->SensorsRef)
		{
//			p_hall->SensorsSaved = hall;
			Hall_ProcCommutation_ISR(p_hall);

			isEdge = true;
		}
		else
		{
			isEdge = false;
		}

		return (isEdge);
	}
	static inline Hall_CommutationPhase_T Hall_ReadSensorsCommutation(Hall_T * p_hall)
	{
		return Hall_ConvertCommutation(p_hall, Hall_ReadSensors(p_hall));
	}
	static inline Hall_CommutationPhase_T Hall_ConvertCommutation(Hall_T * p_hall, uint8_t hallSensors)
	{
	#if defined(CONFIG_HALL_COMMUTATION_TABLE_FUNCTION)
		//	const uint16_t COMMUTATION_TABLE[] =  // 1 extra dereference using sensor angle, or 2nd table
		//	{
		//		[HALL_ANGLE_CCW_30] 	= HALL_SECTOR_2,
		//		[HALL_ANGLE_CCW_90] 	= HALL_SECTOR_3,
		//		[HALL_ANGLE_CCW_150] 	= HALL_SECTOR_4,
		//		[HALL_ANGLE_CCW_210] 	= HALL_SECTOR_5,
		//		[HALL_ANGLE_CCW_270] 	= HALL_SECTOR_6,
		//		[HALL_ANGLE_CCW_330] 	= HALL_SECTOR_1,
		//	};
		return (p_hall->Direction == HALL_DIRECTION_CW) ? p_hall->CommuntationTable[InverseHall(hallSensors)] : p_hall->CommuntationTable[hallSensors];
	#endif

	}

	static inline Hall_CommutationPhase_T Hall_GetCommutation(Hall_T * p_hall)
	{
		return Hall_ConvertCommutation(p_hall, p_hall->SensorsRef);
	}

#endif
