/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery

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
    @file   Timer.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
/******************************************************************************/
#include "Timer.h"


// bool Timer_Poll(Timer_T * p_timer)
// {
//     bool isElapsed;

//     if (p_timer->Mode == TIMER_MODE_STOPPED) { isElapsed = false; }
//     else
//     {
//         isElapsed = Timer_IsElapsed(p_timer);
//         if (isElapsed == true)
//         {
//             switch (p_timer->Mode)
//             {
//                 case TIMER_MODE_PERIODIC: Timer_Restart(p_timer); break;
//                 case TIMER_MODE_ONE_SHOT: p_timer->Mode = TIMER_MODE_STOPPED; break;
//                 case TIMER_MODE_MULTI_SHOT:
//                     if (p_timer->Counter > 0U) { p_timer->Counter--; Timer_Restart(p_timer); }
//                     else { p_timer->Mode = TIMER_MODE_STOPPED; }
//                     break;
//                 default: break;
//             }
//         }
//     }

//     return isElapsed;
// }