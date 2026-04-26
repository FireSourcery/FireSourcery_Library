#pragma once

/******************************************************************************/
/*!
    @section LICENSE

    Copyright (C) 2025 FireSourcery
*/
/******************************************************************************/
/******************************************************************************/
/*!
    @file   Motor_SinCos.h
    @author FireSourcery
    @brief  Motor-level integration for SinCos resolver: calibration state machine,
            generic Cmd/Var dispatch.
*/
/******************************************************************************/
#include "../../Motor.h"

extern void Motor_SinCos_Calibrate(const Motor_T * p_motor);
extern void Motor_SinCos_Cmd(const Motor_T * p_motor, int varId, int varValue);
