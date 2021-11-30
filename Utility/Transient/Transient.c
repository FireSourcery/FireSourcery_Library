//typedef struct
//{
//	void * p_Var;
//	uint8_t Size;
//
//	uint16_t SampleCount;
//	uint16_t SampleFreq;
//	volatile uint8_t SampleBuffer[];	//sample count max
//	volatile uint16_t SampleBufferIndex;
//	volatile uint16_t SampleMissedCount;
//
//	void * p_Trigger;
//	uint8_t TriggerSize;
//
//	volatile uint32_t TriggerTime;
//	volatile uint32_t TriggerPeriod;
//
//	volatile bool TriggerEnabled; //disable after buffer full, or next trigger
//
//
//} Transient_T;
//
//typedef struct
//{
//	uint8_t VarsActiveCount;
//
//	Transient_T Transient[];
//} Transient_Table_T;
//
//void Transient_Start(Transient_T * p_trans)
//{
//	p_trans->TriggerTime = Micros();
//	p_trans->SampleBufferIndex = 0U;
//	p_trans->IsActive = true;
//}
//
//
//void Transient_PollTrigger(Transient_T * p_trans)
//{
//	if (p_Trigger > threshold)
//	{
//		Transient_Start
//
//		//disable trigger
//	}
//}
//
//void Transient_ProcRecord(Transient_T * p_trans)
//{
//	if (		p_trans->IsActive == true)
//	{
//		p_trans->SampleBuffer[] = p_trans
//	}
//}




