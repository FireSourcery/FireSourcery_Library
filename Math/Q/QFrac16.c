/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2023 FireSourcery

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
    @file   Q.c
    @author FireSourcery
    @brief
    @version V0
*/
/******************************************************************************/
#include "QFrac16.h"

/*! Resolution: 1024 steps per revolution */
const qfrac16_t QFRAC16_SINE_90_TABLE[QFRAC16_SINE_90_TABLE_LENGTH] =
{
    0, 201, 402, 603, 804, 1005, 1206, 1406,
    1607, 1808, 2009, 2209, 2410, 2610, 2811, 3011,
    3211, 3411, 3611, 3811, 4011, 4210, 4409, 4608,
    4807, 5006, 5205, 5403, 5601, 5799, 5997, 6195,
    6392, 6589, 6786, 6982, 7179, 7375, 7571, 7766,
    7961, 8156, 8351, 8545, 8739, 8932, 9126, 9319,
    9511, 9703, 9895, 10087, 10278, 10469, 10659, 10849,
    11038, 11227, 11416, 11604, 11792, 11980, 12166, 12353,
    12539, 12724, 12909, 13094, 13278, 13462, 13645, 13827,
    14009, 14191, 14372, 14552, 14732, 14911, 15090, 15268,
    15446, 15623, 15799, 15975, 16150, 16325, 16499, 16672,
    16845, 17017, 17189, 17360, 17530, 17699, 17868, 18036,
    18204, 18371, 18537, 18702, 18867, 19031, 19194, 19357,
    19519, 19680, 19840, 20000, 20159, 20317, 20474, 20631,
    20787, 20942, 21096, 21249, 21402, 21554, 21705, 21855,
    22004, 22153, 22301, 22448, 22594, 22739, 22883, 23027,
    23169, 23311, 23452, 23592, 23731, 23869, 24006, 24143,
    24278, 24413, 24546, 24679, 24811, 24942, 25072, 25201,
    25329, 25456, 25582, 25707, 25831, 25954, 26077, 26198,
    26318, 26437, 26556, 26673, 26789, 26905, 27019, 27132,
    27244, 27355, 27466, 27575, 27683, 27790, 27896, 28001,
    28105, 28208, 28309, 28410, 28510, 28608, 28706, 28802,
    28897, 28992, 29085, 29177, 29268, 29358, 29446, 29534,
    29621, 29706, 29790, 29873, 29955, 30036, 30116, 30195,
    30272, 30349, 30424, 30498, 30571, 30643, 30713, 30783,
    30851, 30918, 30984, 31049, 31113, 31175, 31236, 31297,
    31356, 31413, 31470, 31525, 31580, 31633, 31684, 31735,
    31785, 31833, 31880, 31926, 31970, 32014, 32056, 32097,
    32137, 32176, 32213, 32249, 32284, 32318, 32350, 32382,
    32412, 32441, 32468, 32495, 32520, 32544, 32567, 32588,
    32609, 32628, 32646, 32662, 32678, 32692, 32705, 32717,
    32727, 32736, 32744, 32751, 32757, 32761, 32764, 32766
};

/*
    sin_quadrant_265
    0b xx11 1111 11xx xxxx
    Use 8 most significant digits of 90 degree bound.
    Removes sign / 180 degree bit, 90 degree bit, and 6 lsb.
*/
static inline qfrac16_t sin90(qangle16_t theta)
{
    return QFRAC16_SINE_90_TABLE[(uint8_t)(theta >> QFRAC16_SINE_90_TABLE_LSB)];
}

static inline qfrac16_t cos90(qangle16_t theta)
{
    return QFRAC16_SINE_90_TABLE[(0xFFU - (uint8_t)(theta >> QFRAC16_SINE_90_TABLE_LSB))];
}

/*
    [0, 90)     => [0x0000, 0x3FFF] => [0, 0xFF] == [0, 1)
    [90, 180)   => [0x4000, 0x7FFF] => [0xFF, 0] == (1, 0]
    [180, 270)  => [0x8000, 0xBFFF] => [0, 0xFF] == [0, -1)
    [270, 360)  => [0xC000, 0xFFFF] => [0xFF, 0] == (-1, 0]
*/
qfrac16_t qfrac16_sin(qangle16_t theta)
{
    qfrac16_t sine;
    switch(qangle16_quadrant(theta))
    {
        case QANGLE16_QUADRANT_I:   sine = sin90(theta);                            break;
        case QANGLE16_QUADRANT_II:  sine = sin90(QANGLE16_180 - 1 - theta);         break;
        case QANGLE16_QUADRANT_III: sine = 0 - sin90(theta);                        break;
        case QANGLE16_QUADRANT_IV:  sine = 0 - sin90(QANGLE16_180 - 1 - theta);     break;
        default: sine = 0; break;
    }
    return sine;
}

qfrac16_t qfrac16_cos(qangle16_t theta)
{
    qfrac16_t cosine;
    switch(qangle16_quadrant(theta))
    {
        case QANGLE16_QUADRANT_I:   cosine = sin90(QANGLE16_180 - 1 - theta);       break;
        case QANGLE16_QUADRANT_II:  cosine = 0 - sin90(theta);                      break;
        case QANGLE16_QUADRANT_III: cosine = 0 - sin90(QANGLE16_180 - 1 - theta);   break;
        case QANGLE16_QUADRANT_IV:  cosine = sin90(theta);                          break;
        default: cosine = 0; break;
    }
    return cosine;
}

/* compiler optimize into single switch? */
void qfrac16_vector(qfrac16_t * p_cos, qfrac16_t * p_sin, qangle16_t theta)
{
    *p_sin = qfrac16_sin(theta);
    *p_cos = qfrac16_cos(theta);
    return; /* (*p_cos, *p_sin) */
}

uint16_t qfrac16_vector_magnitude(qfrac16_t x, qfrac16_t y)
{
    return q_sqrt((int32_t)x * x + (int32_t)y * y);
}

/*!
    Vector Circle Limit
    @return sqrt(x^2 + y^2) if limited
*/
uint16_t qfrac16_vector_limit(qfrac16_t * p_x, qfrac16_t * p_y, qfrac16_t magnitudeMax)
{
    uint32_t magnitudeMaxSquared = (int32_t)magnitudeMax * magnitudeMax;
    uint32_t vectorMagnitudeSquared = ((int32_t)(*p_x) * (*p_x)) + ((int32_t)(*p_y) * (*p_y));
    uint16_t vectorMagnitude = 0U;
    int32_t ratio; /* Q1.15 where 32767 ~= 1 */

    if(vectorMagnitudeSquared > magnitudeMaxSquared)
    {
        vectorMagnitude = q_sqrt(vectorMagnitudeSquared);
        ratio = qfrac16_div(magnitudeMax, vectorMagnitude); /* no saturation needed, vectorMagnitude > magnitudeMax, max return 32768 ~= 1 */
        *p_x = (qfrac16_t)qfrac16_mul(*p_x, ratio); /* no saturation needed, ratio < 1 */
        *p_y = (qfrac16_t)qfrac16_mul(*p_y, ratio);
    }

    return vectorMagnitude;
}

/*
    Adapted from libfixmath https://github.com/PetteriAimonen/libfixmath/blob/master/libfixmath/qfrac16_trig.c
*/
qangle16_t qfrac16_atan2(qfrac16_t y, qfrac16_t x)
{
    int32_t mask = (y >> QFRAC16_N_BITS);
    int32_t yAbs = (y + mask) ^ mask;
    int32_t r, r_3, angle;

    if(x >= 0)
    {
        r = qfrac16_div((x - yAbs), (x + yAbs));
        r_3 = qfrac16_mul(qfrac16_mul(r, r), r);
        angle = qfrac16_mul(0x07FF, r_3) - qfrac16_mul(0x27FF, r) + QFRAC16_1_DIV_4;
    }
    else
    {
        r = qfrac16_div((x + yAbs), (yAbs - x));
        r_3 = qfrac16_mul(qfrac16_mul(r, r), r);
        angle = qfrac16_mul(0x07FF, r_3) - qfrac16_mul(0x27FF, r) + QFRAC16_3_DIV_4;
    }

    if(y < 0) { angle = 0 - angle; }

    return angle; /* angle loops, no need to saturate */
}