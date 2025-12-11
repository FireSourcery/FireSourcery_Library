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
    @file   Q.h
    @author FireSourcery
    @brief  fixed point / int math operations

*/
/******************************************************************************/
#ifndef FIXED_MATH_H
#define FIXED_MATH_H

#include <stdint.h>

extern uint16_t fixed_sqrt(uint32_t x);
extern uint8_t fixed_log2(uint32_t x);
extern uint8_t fixed_log2_ceiling(uint32_t x);
extern uint8_t fixed_log2_round(uint32_t x);
extern uint32_t fixed_pow2_round(uint32_t x);

extern uint8_t fixed_bit_width(uint32_t x);
extern uint8_t fixed_bit_width_signed(int32_t x);

// extern uint8_t fixed_sign_bits_unsigned(uint32_t x);
extern uint8_t fixed_lshift_max_unsigned(uint32_t x);
extern uint8_t fixed_lshift_max_signed(int32_t x);

#endif

