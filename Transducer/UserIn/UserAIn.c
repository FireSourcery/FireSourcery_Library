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
    @file   UserAIn.c
    @author FireSourcery
    @brief  [Brief description of the file]
*/
/******************************************************************************/
#include "UserAIn.h"


/******************************************************************************/
/*
    Private Helper Functions
*/
/******************************************************************************/
static inline bool UseDIn(const UserAIn_T * p_context)
{
    // return (p_context->UseEdgePin == false)  && ;
    return (p_context->P_EDGE_PIN != NULL);
}

static inline uint16_t FilterValue(const UserAIn_T * p_context, uint16_t value)
{
    return (value + (p_context->P_STATE->Value_Percent16 << p_context->FILTER_SHIFT)) >> (p_context->FILTER_SHIFT + 1U);
}

static inline void _UserAIn_CaptureValue(const UserAIn_T * p_context, uint16_t value_adcu)
{
    p_context->P_STATE->RawValue_Adcu = value_adcu;
    p_context->P_STATE->ValuePrev_Percent16 = p_context->P_STATE->Value_Percent16;
    p_context->P_STATE->Value_Percent16 = FilterValue(p_context, Linear_ADC_Percent16(&p_context->P_STATE->Units, value_adcu));
}


/******************************************************************************/
/*
    Public Functions
*/
/******************************************************************************/
void UserAIn_InitFrom(const UserAIn_T * p_context, const UserAIn_Config_T * p_config)
{
    if (p_context->P_NVM_CONFIG != NULL) { p_context->P_STATE->Config = *p_config; }

    if (p_context->P_EDGE_PIN != NULL) { UserDIn_Init(p_context->P_EDGE_PIN); }

    /* Initialize linear conversion */
    Linear_Q16_Init(&p_context->P_STATE->Units, p_context->P_STATE->Config.AdcZero, p_context->P_STATE->Config.AdcMax);

    /* Initialize state */
    uint16_t initialPercent = Linear_ADC_Percent16(&p_context->P_STATE->Units, p_context->P_STATE->Config.AdcZero);

    p_context->P_STATE->RawValue_Adcu = p_context->P_STATE->Config.AdcZero;
    p_context->P_STATE->Value_Percent16 = initialPercent;
    p_context->P_STATE->ValuePrev_Percent16 = initialPercent;
}


void UserAIn_Init(const UserAIn_T * p_context)
{
    UserAIn_InitFrom(p_context, p_context->P_NVM_CONFIG);
}

/******************************************************************************/
/*
    Main Polling Function
*/
/******************************************************************************/
void UserAIn_CaptureValue(const UserAIn_T * p_context, uint16_t value_adcu)
{
    if (p_context->P_EDGE_PIN != NULL) { UserDIn_PollEdge(p_context->P_EDGE_PIN); }

    /* Analog capture stops when pin gate selects disable. filerted value persists... */
    /* Note: When edge pin blocks, analog value persists (not cleared to 0) */
    /* This preserves the last valid reading for when pin is re-enabled */
    /* Gating is handled in the getter functions instead */
    if (_UserAIn_IsEdgePinPassthrough(p_context->P_EDGE_PIN) == true) { _UserAIn_CaptureValue(p_context, value_adcu); }
}

bool UserAIn_PollEdge(const UserAIn_T * p_context, uint16_t value_adcu)
{
    UserAIn_CaptureValue(p_context, value_adcu);
    return (p_context->P_EDGE_PIN != NULL) ? UserDIn_PollRisingEdge(p_context->P_EDGE_PIN) : _UserAIn_IsEdge(p_context);
}

bool UserAIn_PollRisingEdge(const UserAIn_T * p_context, uint16_t value_adcu)
{
    UserAIn_CaptureValue(p_context, value_adcu);
    return (p_context->P_EDGE_PIN != NULL) ? UserDIn_PollRisingEdge(p_context->P_EDGE_PIN) : _UserAIn_IsRisingEdge(p_context);
}

/* get value needs to check pin on getter */
bool UserAIn_PollFallingEdge(const UserAIn_T * p_context, uint16_t value_adcu)
{
    UserAIn_CaptureValue(p_context, value_adcu);
    return (p_context->P_EDGE_PIN != NULL) ? UserDIn_PollFallingEdge(p_context->P_EDGE_PIN) : _UserAIn_IsFallingEdge(p_context);
}



/* capture pin to value */
// static inline bool UserAIn_PollFallingEdge(const UserAIn_T * p_context)
// {`
//     bool isValueFallingEdge = ((p_context->P_STATE->ValuePrev_Percent16 > 0U) && (p_context->P_STATE->Value_Percent16 <= 0U));
//     bool isDInFallingEdge = false;

//     if (UseDIn(p_context) == true)
//     {
//         /* Once 1 part detects falling edge, prevent the other from repeat detect */ /* alternatively handle during access */
//         if (UserDIn_PollFallingEdge(p_context->P_EDGE_PIN) == true)
//         {
//             p_context->P_STATE->ValuePrev_Percent16 = p_context->P_STATE->Value_Percent16;
//             p_context->P_STATE->Value_Percent16 = 0;
//         }
//     }

//     return isValueFallingEdge || isDInFallingEdge;
// }
