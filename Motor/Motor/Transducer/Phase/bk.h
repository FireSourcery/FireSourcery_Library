	/*
	 * Buffered data
	 */
	 uint16_t DutyA_Ticks; /* Phase PWM duty peroid in ticks */
	 uint16_t DutyB_Ticks;
	 uint16_t DutyC_Ticks;

	 uint16_t DutyA;
	 uint16_t DutyB;
	 uint16_t DutyC;

	 bool StateA; /* On/Off State */
	 bool StateB;
	 bool StateC;



	bool UseSinusoidalInterpolation;
	uint32_t AngularSpeedTime; // (384*HallBaseTimerFreq/ISRFreq)
	  uint16_t Angle;
	  uint16_t AngleOffset;
	  uint32_t * HallTimerDelta;
	  uint32_t ISRCount;

/*
	Actuate arguments immediately
 */
/*
	Duty in ticks
 */
static inline void ActuateDutyCycle_Ticks(const Phase_T * p_phase, uint32_t pwmDutyA, uint32_t pwmDutyB, uint32_t pwmDutyC)
{
//#if  	defined(CONFIG_PHASE_PWM)
	PWM_WriteDuty(p_phase->p_PwmA, pwmDutyA);	PWM_WriteDuty(p_phase->p_PwmB, pwmDutyB);	PWM_WriteDuty(p_phase->p_PwmC, pwmDutyC);
//#elif 	defined(CONFIG_PHASE_HAL_PHASE)
//	HAL_Phase_WriteDuty(p_phase->p_HAL_Phase, pwmDutyA, pwmDutyB, pwmDutyC);
//#endif
}

static inline void Phase_ActuateState(const Phase_T * p_phase, Phase_Id_T id)
static inline void Phase_ActuateState(const Phase_T * p_phase, bool a, bool b, bool c)
{
//#if  	defined(CONFIG_PHASE_PWM)
	PWM_WriteState(p_phase->p_PwmA, a);
	PWM_WriteState(p_phase->p_PwmB, b);
	PWM_WriteState(p_phase->p_PwmC, c);
//	#ifdef CONFIG_PHASE_HAL_EXTERNAL_SWITCH
		Pin_WriteState(p_phase->p_PinSwitchA, a);
		Pin_WriteState(p_phase->p_PinSwitchB, b);
		Pin_WriteState(p_phase->p_PinSwitchC, c);
//	#endif
//#elif 	defined(CONFIG_PHASE_HAL_PHASE)
//	HAL_Phase_WriteState(p_phase->p_HAL_Phase, a, b, c);
//#endif
}

static inline void Phase_ActuateInvertPolarity(const Phase_T * p_phase, bool isInvA, bool isInvB, bool isInvC)
{
#if  	defined(CONFIG_PHASE_PWM)
	PWM_WriteInvertPolarity(p_phase->p_PwmA, isInvA);
	PWM_WriteInvertPolarity(p_phase->p_PwmB, isInvB);
	PWM_WriteInvertPolarity(p_phase->p_PwmC, isInvC);
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	HAL_Phase_WriteInvertPolarity(p_phase->p_HAL_Phase, isInvA, isInvB, isInvC);
#endif
}

/*
	Actuate buffered data
 */
static inline void Phase_Actuate(const Phase_T * p_phase)
{
#if  	defined(CONFIG_PHASE_PWM)
	PWM_WriteDuty(p_phase->p_PwmA, p_phase->DutyA);
	PWM_WriteDuty(p_phase->p_PwmB, p_phase->DutyB);
	PWM_WriteDuty(p_phase->p_PwmC, p_phase->DutyC);
	PWM_WriteState(p_phase->p_PwmA, a);
	PWM_WriteState(p_phase->p_PwmB, b);
	PWM_WriteState(p_phase->p_PwmC, c);
	#ifdef CONFIG_PHASE_HAL_EXTERNAL_SWITCH
		Pin_WriteState(p_phase->p_PinSwitchA, p_phase->StateA);
		Pin_WriteState(p_phase->p_PinSwitchB, p_phase->StateB);
		Pin_WriteState(p_phase->p_PinSwitchC, p_phase->StateC);
	#endif
#elif 	defined(CONFIG_PHASE_HAL_PHASE)
	HAL_Phase_WriteDuty(p_phase->p_HAL_Phase, p_phase->DutyA_Ticks, p_phase->DutyB_Ticks, p_phase->DutyC_Ticks);
	HAL_Phase_WriteState(p_phase->p_HAL_Phase, p_phase->StateA, p_phase->StateB, p_phase->StateC);
#endif
}



/*
	Set buffered data
	Duty cycle in 16 bits. e.g. 65536 == 100%
 */
static inline void Phase_SetDutyCyle(Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	p_phase->DutyA_Ticks = (uint32_t)pwmDutyA * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U;
	p_phase->DutyB_Ticks = (uint32_t)pwmDutyB * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U;
	p_phase->DutyC_Ticks = (uint32_t)pwmDutyC * (uint32_t)p_phase->PwmPeriod_Ticks / 65536U;
}

static inline void Phase_SetDutyCyle_15(Phase_T * p_phase, uint16_t pwmDutyA, uint16_t pwmDutyB, uint16_t pwmDutyC)
{
	p_phase->DutyA_Ticks = (uint32_t)pwmDutyA * (uint32_t)p_phase->PwmPeriod_Ticks / 32768U;
	p_phase->DutyB_Ticks = (uint32_t)pwmDutyB * (uint32_t)p_phase->PwmPeriod_Ticks / 32768U;
	p_phase->DutyC_Ticks = (uint32_t)pwmDutyC * (uint32_t)p_phase->PwmPeriod_Ticks / 32768U;
}

static inline void Phase_SetState(Phase_T * p_phase, bool a, bool b, bool c)
{
	p_phase->StateA = a;
	p_phase->StateB = b;
	p_phase->StateC = c;
}
