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
    @file   Analog.c
    @author FireSourcery
    @brief  Analog module conventional function definitions
    @version V0
*/
/******************************************************************************/
#include "Analog.h"

/******************************************************************************/
/*!
    Protected
*/
/******************************************************************************/
/*
    Activate and wait for return
*/
static inline void WriteAdcChannel(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
    HAL_Analog_Activate(p_analog->CONST.P_HAL_ANALOG, p_conversion->PIN);
#ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
    p_analog->ActiveChannelCount = 1U;
#endif
}

#ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
static inline void WriteAdcFifo(Analog_T * p_analog)
{
    Analog_Conversion_T * p_conversion;

    uint8_t remainingChannelCount = Ring_GetFullCount(&p_analog->ConversionQueue); //check options
    p_analog->ActiveChannelCount = (remainingChannelCount < GLOBAL_ANALOG.ADC_FIFO_LENGTH) ? remainingChannelCount : GLOBAL_ANALOG.ADC_FIFO_LENGTH;

    HAL_Analog_WriteFifoCount(p_analog->CONST.P_HAL_ANALOG, p_analog->ActiveChannelCount);

    for(uint8_t iConversion = 0U; iConversion < p_analog->ActiveChannelCount - 1U; iConversion++)
    {
        Ring_PeekIndex(&p_analog->ConversionQueue, &p_conversion, iConversion);
        HAL_Analog_WriteFifoPin(p_analog->CONST.P_HAL_ANALOG, p_conversion->PIN);
    }
    Ring_PeekIndex(&p_analog->ConversionQueue, &p_conversion, p_analog->ActiveChannelCount);
    HAL_Analog_ActivateFifo(p_analog->CONST.P_HAL_ANALOG, p_conversion->PIN);
}
#endif

void WriteAdcOptions(Analog_T * p_analog, const Analog_Options_T * p_options)
{
    // xor with previous
    if(p_options->FLAGS.HwTriggerConversion == 1U)      { HAL_Analog_EnableHwTrigger(p_analog->CONST.P_HAL_ANALOG); }
    else                                                { HAL_Analog_DisableHwTrigger(p_analog->CONST.P_HAL_ANALOG); }
#ifdef CONFIG_ANALOG_HW_CONTINOUS_CONVERSION_ENABLE
    if(p_options->FLAGS.ContinuousConversion == 1U)     { HAL_Analog_EnableContinuousConversion(p_analog->CONST.P_HAL_ANALOG): }
    else                                                { HAL_Analog_DisableContinuousConversion(p_analog->CONST.P_HAL_ANALOG); }
#endif
    if(p_options->ON_OPTIONS != 0U) { p_options->ON_OPTIONS(p_options->P_CALLBACK_CONTEXT); }
}

/*
    Dequeue all conversion containing only options, until next channel, starts wait for ISRs
*/
void _Analog_ProcQueue(Analog_T * p_analog)
{
    bool isComplete = true;
    Analog_QueueItem_T * p_next;

    while(Ring_PeekFront(&p_analog->ConversionQueue, &p_next) == true)
    {
        if(p_next->TYPE == ANALOG_QUEUE_TYPE_OPTIONS) /* Write and Deactivate. Ensure not to write empty conversion that calls ISR */
        {
            WriteAdcOptions(p_analog, (Analog_Options_T *)p_next);
            Ring_RemoveFront(&p_analog->ConversionQueue, 1U);
        }
        else if(p_next->TYPE == ANALOG_QUEUE_TYPE_CHANNEL)  /* Activate and wait for ISR */
        {
#ifdef CONFIG_ANALOG_HW_FIFO_ENABLE
            WriteAdcFifo(p_analog);
#else
            WriteAdcChannel(p_analog, (Analog_Conversion_T *)p_next);
#endif
            isComplete = false;
            break;
        }
        else { break; } /* error */
    }

    if(isComplete == true) { HAL_Analog_Deactivate(p_analog->CONST.P_HAL_ANALOG); }
}



/******************************************************************************/
/*!
    Public
*/
/******************************************************************************/
void Analog_Init(Analog_T * p_analog)
{
    HAL_Analog_Init(p_analog->CONST.P_HAL_ANALOG);
    HAL_Analog_Deactivate(p_analog->CONST.P_HAL_ANALOG);
    Ring_Init(&p_analog->ConversionQueue);
}

// bool Analog_MarkConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
// {
//     bool isSuccess;
//     p_analog->P_ADCS[p_conversion->ADC_ID].ConversionFlags[p_conversion->PIN] = true;
//     return isSuccess;
// }

/*
    Cannot overwrite
*/
bool Analog_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
    bool isSuccess;
    _Analog_EnterCritical(p_analog);
    if(Ring_IsEmpty(&p_analog->ConversionQueue) == true) { WriteAdcChannel(p_analog, p_conversion); }
    isSuccess = Ring_Enqueue(&p_analog->ConversionQueue, &p_conversion);
    _Analog_ExitCritical(p_analog);
    return isSuccess;
}

bool Analog_EnqueueOptions(Analog_T * p_analog, const Analog_Options_T * p_options)
{
    bool isSuccess;
    _Analog_EnterCritical(p_analog);
    if(Ring_IsEmpty(&p_analog->ConversionQueue) == true)
    {
        WriteAdcOptions(p_analog, p_options);
        HAL_Analog_Deactivate(p_analog->CONST.P_HAL_ANALOG);
        isSuccess = true;
    }
    else
    {
        isSuccess = Ring_Enqueue(&p_analog->ConversionQueue, &p_options);
    }
    _Analog_ExitCritical(p_analog);
    return isSuccess;
}

/*!
    @brief Public function to activate ADC.
    Overwrite active conversion
*/
void Analog_ActivateConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
    _Analog_EnterCritical(p_analog);
    //    p_analog->p_ActiveConversion = p_conversion;
    Ring_RemoveFront(&p_analog->ConversionQueue, 1U); /* Removes only if available */
    WriteAdcChannel(p_analog, p_conversion);
    _Analog_ExitCritical(p_analog);
}

void Analog_ActivateOptions(Analog_T * p_analog, const Analog_Options_T * p_options)
{
    _Analog_EnterCritical(p_analog);
    WriteAdcOptions(p_analog, p_options);
    _Analog_ExitCritical(p_analog);
}

/******************************************************************************/
/*!
    Group
*/
/******************************************************************************/
/* Common with AnalogN */
void _Analog_Group_ResumeQueue(Analog_T * p_analog)
{
    if(_Analog_ReadIsActive(p_analog) == false) { _Analog_ProcQueue(p_analog); }
}

/*
    Enqueue without starting conversion of first item until queue resumes
    Eliminate repeat empty check, and uneven wait period between 1st and 2nd complete
*/
void Analog_Group_PauseQueue(Analog_T * p_analog)
{
    _Analog_EnterCritical(p_analog);
}

void Analog_Group_ResumeQueue(Analog_T * p_analog)
{
    _Analog_Group_ResumeQueue(p_analog);
    _Analog_ExitCritical(p_analog);
}

bool Analog_Group_EnqueueConversion(Analog_T * p_analog, const Analog_Conversion_T * p_conversion)
{
    return Ring_Enqueue(&p_analog->ConversionQueue, &p_conversion);
}

bool Analog_Group_EnqueueOptions(Analog_T * p_analog, const Analog_Options_T * p_options)
{
    return Ring_Enqueue(&p_analog->ConversionQueue, &p_options);
}



/******************************************************************************/
/*!
    Channel Fags
*/
/******************************************************************************/
// void Analog_SetChannelConversion(Analog_T * p_analog, analog_channel_t channelId)
// {
//     p_analog->ChannelFlags = (p_analog->ChannelFlags | (1U << channelId));
// }

// void Analog_ProcChannelConversion(Analog_T * p_analog, analog_channel_t channelId)
// {
    // for(channel in channels)
//     p_analog->ChannelFlags
// }
