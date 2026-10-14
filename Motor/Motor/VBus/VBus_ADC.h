#include "VBus.h"
#include "../Math/foc_math.h"

#include "../Phase_Input/Phase_Board.h"
#include "Transducer/Monitor/Voltage/VMonitor.h"

#include "Math/Fixed/fract16.h"
#include "Math/Linear/linear_math.h"

#include <stdint.h>
#include <stdbool.h>


/*
    Optionally as Adc callback, per control cycle
*/
#include "../Phase_Input/Phase_Analog.h" /* for Phase_Analog_VPuOf() */
static inline void VBus_ADC_Capture(VBus_T * p_vbus, adc_result_t adcu) { VBus_Capture(p_vbus, Phase_Analog_VPuOf(adcu)); }
/* optionally separate filter for monitor vs vout scaling  */

#include "../Phase_Input/Phase_Analog.h" /*  */
/*  */
static inline uint32_t VBus_ADC_BoardId_Get(VDivider_ConfigId_T var_id)
{
    switch (var_id)
    {
        case VDIVIDER_BOARD_R1: return PHASE_ANALOG_BOARD.V_PHASE_R1;
        case VDIVIDER_BOARD_R2: return PHASE_ANALOG_BOARD.V_PHASE_R2;
        default: return 0U;
    }
}