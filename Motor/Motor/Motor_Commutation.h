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
    @file   Motor_Commutation.h
    @author FireSourcery

    @brief
*/
/******************************************************************************/
#ifndef MOTOR_COMMUTATION_H
#define MOTOR_COMMUTATION_H

#include "Motor_FOC.h"
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
#include "Motor_SixStep.h"
#endif
#include "Motor.h"

/******************************************************************************/
/*
    Simplify CommutationMode Check
    This function should optimize away select if only 1 mode is enabled

*/
/******************************************************************************/
static inline const void * _Motor_CommutationModeFn(const Motor_State_T * p_motor, const void * focFunction, const void * sixStepFunction)
{
    const void * fn;
    switch (p_motor->Config.CommutationMode)
    {
#if defined(CONFIG_MOTOR_FOC_ENABLE)
        case MOTOR_COMMUTATION_MODE_FOC: fn = focFunction; break;
#endif
#if defined(CONFIG_MOTOR_SIX_STEP_ENABLE)
        case MOTOR_COMMUTATION_MODE_SIX_STEP: fn = sixStepFunction; break;
#endif
        // default: assert(false); break;
    }
    return fn;
}

// c23
// #define Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction) ((typeof(focFunction) *)(_Motor_CommutationModeFn(p_motor, focFunction, sixStepFunction)))

#define Motor_CommutationModeFn(p_motor, focSet, sixStepSet) \
    _Generic((focSet), \
        Motor_Set_T:    (Motor_Set_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)), \
        Motor_Get_T:    (Motor_Get_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet)),  \
        Motor_Proc_T:   (Motor_Proc_T)(_Motor_CommutationModeFn(p_motor, focSet, sixStepSet))   \
    )

#define Motor_CommutationModeFn_Call(p_motor, focSet, sixStepSet, ...) ((Motor_CommutationModeFn(p_motor, focSet, sixStepSet))(p_motor __VA_OPT__(,) __VA_ARGS__))


/*     alternatively seprate 3 sets of functions as abstraction layer */
// static inline void Motor_Commutation_SetDirection(Motor_State_T * p_motor, int direction) { Motor_FOC_SetDirection(p_motor, direction); }
// static inline void Motor_Commutation_SetDirection(Motor_State_T * p_motor, int direction) { Motor_SixStep_SetDirection(p_motor, direction); }

// static inline void Motor_Commutation_SetDirection(Motor_State_T * p_motor, int direction) {
//     ((typeof(Motor_FOC_SetDirection) *)(_Motor_CommutationModeFn(p_motor, Motor_FOC_SetDirection, Motor_SetDirection)))(p_motor, direction);
// }



#endif // MOTOR_COMMUTATION_H