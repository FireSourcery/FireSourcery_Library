#include "Math/Q/QFrac16.h"


/*
 * y[n] = (1-lambda)*y[n-1] + lambda*x
 *
 * lambda = 1/N
 *
 * e.g.
 *
 * 0.015625 -> 64 size array
 * 0.03125  -> 32 size array
 *
 */
inline int16_t filter_movavg(int16_t yPrev, int16_t x, qfrac16_t lambda)
{
	int16_t y;

	y = (((int32_t)yPrev * ((int32_t)0x10000 - lambda)) + ((int32_t)x * lambda)) >> 15;

	return y;
}

/*
 * y[k] = (1-lambda)*y[k-1] + lambda*x
 *
 * lambda = 1/N
 */
inline int32_t filter_movavgn(int32_t yPrev, int32_t x, uint8_t n)
{
	int32_t y;

	y = (yPrev*(n - 1) + x)/n;

	return y;
}

//int32_t Filter_MovAvgN_Pow2(int32_t yPrev, int32_t x, uint8_t n)

/*
 * Moving Average with N of 2. i.e. weighs previous value and input 50-50
 */
inline int32_t filter_movavg2(int32_t yPrev, int32_t x)
{
	int32_t y;

	y = (yPrev + x) / 2;

	return y;
}
