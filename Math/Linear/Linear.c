/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery / The Firebrand Forge Inc

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
    @file     Linear.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "Linear.h"
#include "Math/Q/Q.h"

/* Shift <= log2(INT32_MAX / ((x_max - x0) * Slope)) */
uint8_t _Linear_GetMaxSlopeShift_Signed(int32_t factor, int32_t divisor, int32_t maxInputDelta)
{
    return q_maxshift_signed((maxInputDelta * 2 - 1) * factor / divisor); /* divide first rounds up log2 */
}

/******************************************************************************/
/*!
    Linear
    f(x) = (factor * (x - x0) / divisor) + y0
    f(xRef) = yRef

    Overflow: factor, divisor > 131,071. (divisor * (yRef - y0)) > INT32_MAX

    max x input at least [X0 + 2*(XRef - X0)]
    if slope < 1, (factor < divisor), shift 14, => max input [65536+65535]
    if slope > 1, (factor > divisor), bound with yRef.

    @param[in] y0 - y-intercept, x0 = 0
    @param[in] yRef - y output as 100 percent
*/
/******************************************************************************/
void Linear_Init(Linear_T * p_linear, int32_t factor, int32_t divisor, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
    p_linear->YReference        = yRef;
    p_linear->XReference        = linear_invf(factor, divisor, y0, yRef);     /* (yRef - y0)*divisor/factor + 0 */
    p_linear->XOffset           = 0;
    p_linear->YOffset           = y0;
    p_linear->DeltaX            = p_linear->XReference - p_linear->XOffset;
    p_linear->DeltaY            = yRef - y0;
    p_linear->SlopeShift        = _Linear_GetMaxSlopeShift_Signed(factor, divisor, p_linear->DeltaX);
    p_linear->Slope             = (factor << p_linear->SlopeShift) / divisor;
    p_linear->InvSlopeShift     = _Linear_GetMaxSlopeShift_Signed(divisor, factor, p_linear->DeltaY);
    p_linear->InvSlope          = (divisor << p_linear->InvSlopeShift) / factor;
#elif defined(CONFIG_LINEAR_DIVIDE_NUMERICAL)
    p_linear->SlopeFactor = factor;
    p_linear->SlopeDivisor = divisor;
    p_linear->YOffset = y0;
    p_linear->YReference = yRef;
#endif
}

/*
    Map [x0:xRef] to [y0:yRef]. Interpolate from (x0, y0) to (xRef, yRef).
    Derive slope
*/
void Linear_Init_Map(Linear_T * p_linear, int32_t x0, int32_t xRef, int32_t y0, int32_t yRef)
{
#ifdef CONFIG_LINEAR_DIVIDE_SHIFT
    p_linear->XOffset           = x0;
    p_linear->YOffset           = y0;
    p_linear->DeltaX            = xRef - x0;
    p_linear->DeltaY            = yRef - y0;
    p_linear->YReference        = yRef;
    p_linear->XReference        = xRef;
    p_linear->SlopeShift        = _Linear_GetMaxSlopeShift_Signed(p_linear->DeltaY, p_linear->DeltaX, p_linear->DeltaX);
    p_linear->Slope             = (p_linear->DeltaY << p_linear->SlopeShift) / p_linear->DeltaX;
    p_linear->InvSlopeShift     = _Linear_GetMaxSlopeShift_Signed(p_linear->DeltaY, p_linear->DeltaX, p_linear->DeltaY);
    p_linear->InvSlope          = (p_linear->DeltaX << p_linear->InvSlopeShift) / p_linear->DeltaY;
#endif
}

/******************************************************************************/
/*!
    Scalar
*/
/******************************************************************************/
/* scalar may be compile time constant, can compiler unroll loop to optimize? */
int32_t Linear_Function_Scalar(const Linear_T * p_linear, int32_t x, uint16_t scalar)
{
    int32_t factor = x * p_linear->Slope;
    int32_t result = 0U;

    /*
        Loop N = Log_[Divisor*=](scalar)
        scalar 1000
            [Divisor*=] == 10 => 4
            [Divisor*=] == 2 => 10
    */
    for(uint16_t iDivisor = 1U; scalar >= iDivisor; iDivisor *= 4U)     /* scalar / iDivisor > 0U */
    {
        if(factor < INT32_MAX / scalar * iDivisor) /* (factor < INT32_MAX / (scalar / iDivisor)) */
        {
            result = Linear_Function(p_linear, x * scalar / iDivisor) * iDivisor;
            break;
        }
    }

    if(result == 0) { result = Linear_Function(p_linear, x) * scalar; }

    return result;
}

int32_t Linear_InvFunction_Scalar(const Linear_T * p_linear, int32_t y, uint16_t scalar)
{
    (void)p_linear;
    (void)y;
    (void)scalar;
    return 0; //todo
}
