#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2026 FireSourcery

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
    @file   angle16.h
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>


/******************************************************************************/
/*!
    angle16
*/
/******************************************************************************/
#define PI_FLOAT (3.14159265358979323846F)
#define ANGLE16_PER_REVOLUTION (65536UL)

typedef int16_t angle16_t;      /*!< [-pi, pi) signed or [0, 2pi) unsigned, angle wraps. base as signed for int32_t cast arithmetic */
typedef uint16_t uangle16_t;    /*!< [0, 2pi) unsigned compare */

typedef int32_t angle32_t;  /* extended angle16, Q32 */
typedef int32_t rot32_t;    /* n.angle16 */

#define ROT32_TURN_BITS (16)
#define ROT32_ANGLE_BITS (16)

static const angle16_t ANGLE16_0 = 0U;         /*! 0 */
static const angle16_t ANGLE16_30 = 0x1555U;   /*! 5461 */
static const angle16_t ANGLE16_60 = 0x2AAAU;   /*! 10922 */
static const angle16_t ANGLE16_90 = 0x4000U;   /*! 16384 */
static const angle16_t ANGLE16_120 = 0x5555U;  /*! 21845 */
static const angle16_t ANGLE16_150 = 0x6AAAU;  /*! 27306 */
static const angle16_t ANGLE16_180 = 0x8000U;  /*! 32768, -32768, 180 == -180 */
static const angle16_t ANGLE16_PI = 0x8000U;
static const angle16_t ANGLE16_210 = 0x9555U;  /*! 38229 */
static const angle16_t ANGLE16_240 = 0xAAAAU;  /*! 43690, -21845 */
static const angle16_t ANGLE16_270 = 0xC000U;  /*! 49152, -16384, 270 == -90 */
static const angle16_t ANGLE16_300 = 0xD555U;  /*! 54613 */
static const angle16_t ANGLE16_330 = 0xEAAAU;  /*! 60074 */

/*
    Conversion factor
    Reciprocal of FRACT16_PI in Q30, both map the half turn 32768 ⇔ π:
        ANGLE16_PER_RADIAN = 32768 / π                  rad [Q15] → angle16:  rad · ANGLE16_PER_RADIAN >> 15
        FRACT16_PI         = 32768 · π                  angle16 → rad [Q15]:  angle16 · FRACT16_PI >> 15
        ANGLE16_PER_RADIAN · FRACT16_PI = 2^30          π cancels
    Truncation: 10430 is −3.6e-5, 102943 is −6.9e-6, 683565275 is −8.4e-10.
    With an int64 intermediate, ANGLE32_PER_RADIAN · x / 65536 is the closest form.
*/
static const angle16_t ANGLE16_PER_RADIAN = 10430UL; /* 65536 / 2π */
static const angle32_t ANGLE32_PER_RADIAN = 683565275UL; /* 2^32 / 2π */


#define ANGLE16_OF_RADIANS(radians) ((angle16_t)((radians) * ANGLE16_PER_REVOLUTION / (2.0F * PI_FLOAT)))
#define ANGLE16_OF_TURNS(turns) ((angle16_t)((turns) * 65536.0F))

/* Caller cast result */
/* angle / FRACT16_SCALE * PI_FLOAT */
#define ANGLE16_TO_RAD(angle) (((float)(angle) * (2.0F * PI_FLOAT / ANGLE16_PER_REVOLUTION)))

/*
    radian si scaled
    [0:1.0f/2pi] ~0.16 of rev
*/
/* from scaled storage */
static inline angle16_t angle16_of_radians(int32_t rad, int32_t scaling) { return (angle16_t)((int64_t)rad * ANGLE32_PER_RADIAN / ANGLE16_PER_REVOLUTION / scaling); }

/*

*/
#define ANGLE16_QUADRANT_MASK (0xC000U)

typedef enum angle16_quadrant
{
    ANGLE16_QUADRANT_I      = 0U,       /* 0_90 */
    ANGLE16_QUADRANT_II     = 0x4000U,  /* 90_180 */
    ANGLE16_QUADRANT_III    = 0x8000U,  /* 180_270 */
    ANGLE16_QUADRANT_IV     = 0xC000U,  /* 270_360 */
}
angle16_quadrant_t;

/* e.g. Mechanical Angle */
static inline angle16_quadrant_t angle16_quadrant(angle16_t theta) { return (angle16_quadrant_t)((uint16_t)theta & ANGLE16_QUADRANT_MASK); }

static inline bool angle16_cycle_inc(angle16_t theta0, angle16_t theta1) { return ((uint16_t)theta0 > (uint16_t)theta1); }

static inline bool angle16_cycle_dec(angle16_t theta0, angle16_t theta1) { return ((uint16_t)theta0 < (uint16_t)theta1); }

static inline bool angle16_cycle(angle16_t theta0, angle16_t theta1, int sign) { return ((sign > 0) == angle16_cycle_inc(theta0, theta1)); }

/* polling freq must be sufficient */
/* crossing 0 and 180 */
/* angle16_half_cycle */
/* ((theta0 ^ theta1) < 0); */
static inline bool angle16_cycle2(angle16_t theta0, angle16_t theta1) { return (((theta0 ^ theta1) & 0x8000U) != (uint16_t)0U); }
/* angle16_quarter_cycle */
static inline bool angle16_cycle4(angle16_t theta0, angle16_t theta1) { return (((theta0 ^ theta1) & ANGLE16_QUADRANT_MASK) != (uint16_t)0U); }