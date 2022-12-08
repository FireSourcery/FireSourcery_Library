/*!
	@brief Private Micros Helper.
	todo untested
	Determine micros with max precision, when timerTicks * 1000000 overflows

	micros = 1000000 * (timerTicks / timerFreq);
	micros = timerTicks * (1000000 / timerFreq);
	micros = 1000000 / (timerFreq / timerTicks);
	micros = timerTicks / (timerFreq / 1000000);
*/
static inline uint32_t _Encoder_MicrosHelper(uint32_t timerTicks, uint32_t timerFreq)
{
	uint32_t micros;

	/* overflows if DeltaT > 4294 */
	if (timerTicks > (UINT32_MAX / 1000000))
	{
		// todo optimize
		/* divide largest by smallest */
		if 		((timerTicks > 1000000) && (1000000 > timerFreq))
		{
			micros = 1000000 * (timerTicks / timerFreq);
		}
		else if ((1000000 > timerTicks) && (timerTicks > timerFreq))
		{
			micros = timerTicks * (1000000 / timerFreq);
		}
		else if((timerTicks > timerFreq) && (timerFreq > 1000000))
		{
//			micros = 1000000 * (timerTicks / timerFreq);
			micros = timerTicks / (timerFreq / 1000000);
		}
		else if ((1000000 > timerFreq) && (timerFreq > timerTicks))
		{
//			micros = timerTicks * (1000000 / timerFreq);
			micros = 1000000 / (timerFreq / timerTicks);
		}
		else if ((timerFreq > timerTicks) && (timerTicks > 1000000))
		{
			micros = timerTicks / (timerFreq / 1000000);
		}
		else if ((timerFreq > 1000000) && (1000000 > timerTicks))
		{
			micros = 1000000 / (timerFreq / timerTicks);
		}
	}
	else
	{
		micros = timerTicks * 1000000 / timerFreq;
	}

	return micros;
}

//static inline uint32_t Encoder_GetTotalT_Micros(Encoder_T * p_encoder)	{return _Encoder_MicrosHelper(p_encoder->TotalT,  p_encoder->UnitT_Freq);}
/******************************************************************************/
/*!
	T Unit Conversions - Variable DeltaT (DeltaD is fixed, == 1).
*/
/******************************************************************************/
//static inline uint32_t Encoder_ConvertToTime_Millis(Encoder_T * p_encoder, uint32_t deltaT_Ticks)		{return deltaT_Ticks * 1000U / p_encoder->UnitT_Freq;}
//static inline uint32_t Encoder_ConvertToTime_Seconds(Encoder_T * p_encoder, uint32_t deltaT_Ticks)	{return deltaT_Ticks / p_encoder->UnitT_Freq;}
//static inline uint32_t Encoder_ConvertToFreq(Encoder_T * p_encoder, uint32_t deltaT_Ticks)			{return (deltaT_Ticks == 0U) ? 0U : p_encoder->UnitT_Freq / deltaT_Ticks;}
//static inline uint32_t Encoder_ConvertToFreq_CPM(Encoder_T * p_encoder, uint32_t deltaT_Ticks)		{return (deltaT_Ticks == 0U) ? 0U : p_encoder->UnitT_Freq * 60U / deltaT_Ticks;}
////static inline uint32_t Encoder_ConvertFreqTo(Encoder_T * p_encoder, uint32_t deltaT_FreqHz)			{return (deltaT_FreqHz == 0U) ? 0U : p_encoder->UnitT_Freq / deltaT_FreqHz;}
//
///*!
//	@brief DeltaT period. unit in raw timer ticks.
// */
//static inline uint32_t Encoder_DeltaT_Get(Encoder_T * p_encoder)			{return p_encoder->DeltaT;}
//
///*!
//	@brief DeltaT period. unit in milliseconds
// */
//static inline uint32_t Encoder_DeltaT_Get_Millis(Encoder_T * p_encoder)	{return Encoder_ConvertToTime_Millis(p_encoder, p_encoder->DeltaT);}
//
///*!
//	@brief DeltaT period. unit in microseconds
// */
//static inline uint32_t Encoder_DeltaT_Get_Micros(Encoder_T * p_encoder)	{return _Encoder_MicrosHelper(p_encoder->DeltaT, p_encoder->UnitT_Freq);}
//
///*!
//	@brief DeltaT freq.	unit in Hz
// */
//static inline uint32_t Encoder_DeltaT_GetFreq(Encoder_T * p_encoder)		{return Encoder_ConvertToFreq(p_encoder, p_encoder->DeltaT);}
//
///*!
//	@brief DeltaT freq.	unit in cycles per minute
// */
//static inline uint32_t Encoder_DeltaT_GetFreq_CPM(Encoder_T * p_encoder)	{return Encoder_ConvertToFreq_CPM(p_encoder, p_encoder->DeltaT);}


/*
	Integral Functions
*/
/*! Same as integral D */
static inline uint32_t Encoder_GetTotalD_Units(Encoder_T * p_encoder) { return p_encoder->TotalD * p_encoder->UnitLinearD; }
static inline uint32_t Encoder_GetTotalD_Angle(Encoder_T * p_encoder) { return p_encoder->TotalD * p_encoder->UnitAngularD; }

static inline uint32_t Encoder_GetTotalT_Freq(Encoder_T * p_encoder)		{return p_encoder->UnitT_Freq / p_encoder->TotalT;}
static inline uint32_t Encoder_GetTotalT_Millis(Encoder_T * p_encoder) { return p_encoder->TotalT * 1000U / p_encoder->UnitT_Freq; }

/*
	Integral angle
*/
static inline uint32_t Encoder_GetTotalRevolutions(Encoder_T * p_encoder)
{
	return p_encoder->TotalD / p_encoder->Params.CountsPerRevolution;
}

static inline uint32_t Encoder_GetTotalAngle(Encoder_T * p_encoder)
{
	return Encoder_ConvertCounterDToAngle(p_encoder, p_encoder->TotalD);
}

static inline void Encoder_ResetTotalAngle(Encoder_T * p_encoder)
{
	p_encoder->TotalD = 0U;
}

//static inline uint32_t Encoder_GetLinearDeltaDistance(Encoder_T * p_encoder)	{return Encoder_GetDeltaD_Units(p_encoder);}
static inline uint32_t Encoder_GetLinearTotalDistance(Encoder_T * p_encoder) { return Encoder_GetTotalD_Units(p_encoder); }

/******************************************************************************/
/*!
	Signed Version using quadrature mode - todo
 */
 /******************************************************************************/
// static inline bool Encoder_GetDirection(Encoder_T * p_encoder)
// {
// 	// if(p_encoder->Params.IsALeadBPositive ^ Pin_Input_Read(&p_encoder->PhaseB)) { return false; }
// 	// else { return true; }
// }

// static inline int16_t Encoder_GetAngularVelocity(Encoder_T * p_encoder)
// {
// // 	if(Encoder_GetDirection(p_encoder)) { return (int32_t)Encoder_GetAngularSpeed(p_encoder); }
// // 	else { return (int32_t)0 - (int32_t)Encoder_GetAngularSpeed(p_encoder); }
// }

// static inline int32_t Encoder_GetRotationalVelocity(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetLinearVelocity(Encoder_T * p_encoder)
// {
// 	if(Encoder_GetDirection(p_encoder)) 	{ return (int32_t)Encoder_GetLinearSpeed(p_encoder); }
// 	else 									{ return (int32_t)0 - (int32_t)Encoder_GetLinearSpeed(p_encoder); }
// }

// static inline int32_t Encoder_GetDeltaAngularDisplacement(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetDeltaRotationalDisplacement(Encoder_T * p_encoder)
// {

// }

// // treat displacement as signed scalar quantity
// static inline int32_t Encoder_GetDeltaLinearDisplacement(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetTotalAngularDisplacement(Encoder_T * p_encoder)
// {

// }

// static inline int32_t Encoder_GetTotalLinearDisplacement(Encoder_T * p_encoder)
// {

// }





