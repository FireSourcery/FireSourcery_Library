#pragma once

/*
    Host test double for the ADC HAL. Shaped like the S32K1 path:
    SC1[slot] holds the pin, R[slot] the result, and the trigger converts the slot range the
    sequencer was pointed at. RangeStart / RangeCount stand in for the PDB channel enable mask.
*/
#include <stdint.h>
#include <stdbool.h>

#define HAL_ADC_SLOT_COUNT 8U

typedef struct HAL_ADC
{
    uint32_t SC1[HAL_ADC_SLOT_COUNT];   /* slot -> pin */
    uint32_t R[HAL_ADC_SLOT_COUNT];     /* slot -> result */

    uint8_t RangeStart;                 /* the sequencer's slot range, as HAL_ADC_ActivateSequence sets it */
    uint8_t RangeCount;
    uint32_t RangeWrites;               /* how many times the range was rewritten */

    bool IsActive;
    bool IsComplete;
}
HAL_ADC_T;
