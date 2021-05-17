#include "Filter.h"
#include "math_filter.h"

#include "Math/Q/QFrac16.h"


void Filter_MovAvg_Init(Filter_T * p_filter, int32_t y0, qfrac16_t lambda)
{
	p_filter->Accumulator = y0;
	p_filter->Coeffcient = qfrac16_div(1, lambda);
}

void Filter_MovAvg_InitN(Filter_T * p_filter, int32_t y0, uint8_t n)
{
	p_filter->Accumulator = y0;
	p_filter->Coeffcient = n;
}

int32_t Filter_MovAvg(Filter_T * p_filter, int32_t in)
{
	p_filter->Accumulator = filter_movavgn(p_filter->Accumulator, in, p_filter->Coeffcient);

	return p_filter->Accumulator;
}

int32_t Filter_MovAvg2(Filter_T * p_filter, int32_t in)
{
	p_filter->Accumulator = filter_movavg2(p_filter->Accumulator, in);

	return p_filter->Accumulator;
}

