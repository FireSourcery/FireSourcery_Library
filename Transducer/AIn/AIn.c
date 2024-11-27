// #include <stdint.h>
// #include <stdbool.h>

// typedef struct MotAnalogUser_AIn
// {
//     Debounce_T EdgePin;
//     bool UseEdgePin;        /* Use digital pin to determine AIn Threshold. */
//     Linear_T Units;
//     uint16_t Value_Scalar16;
//     uint16_t ValuePrev_Scalar16;
// }
// MotAnalogUser_AIn_T;
// /*
//     AIn Sub Module
// */
// static inline void _MotAnalogUser_AIn_CaptureValue(MotAnalogUser_AIn_T * p_aIn, uint16_t value_Adcu)
// {
//     if(p_aIn->UseEdgePin == true) { Debounce_CaptureState(&p_aIn->EdgePin); }
//     if((p_aIn->UseEdgePin == false) || (Debounce_GetState(&p_aIn->EdgePin) == true))
//     {
//         p_aIn->Value_Scalar16 = ((uint32_t)Linear_ADC_Percent16(&p_aIn->Units, value_Adcu) + p_aIn->ValuePrev_Scalar16) / 2U;
//         p_aIn->ValuePrev_Scalar16 = p_aIn->Value_Scalar16;
//     }
// }

// /* Edge acts as threshold - Both on for on, one off for off */
// static inline bool _MotAnalogUser_AIn_GetIsOn(const MotAnalogUser_AIn_T * p_aIn)
// {
//     return (p_aIn->UseEdgePin == true) ? (Debounce_GetState(&p_aIn->EdgePin) && (p_aIn->Value_Scalar16 > 0U)) : (p_aIn->Value_Scalar16 > 0U);
// }

// static inline bool _MotAnalogUser_AIn_PollFallingEdge(MotAnalogUser_AIn_T * p_aIn)
// {
//     bool isValueFallingEdge = ((p_aIn->Value_Scalar16 == 0U) && (p_aIn->ValuePrev_Scalar16 > 0U));
//     bool isFallingEdge;

//     if(p_aIn->UseEdgePin == true)
//     {
//         /* Once 1 part detects falling edge, disable the other from repeat detect */
//         if(Debounce_PollFallingEdge(&p_aIn->EdgePin) == true) { p_aIn->Value_Scalar16 = 0U; p_aIn->ValuePrev_Scalar16 = 0U; isFallingEdge = true; }
//         else if(isValueFallingEdge == true) { p_aIn->EdgePin.DebouncedState = false; p_aIn->EdgePin.DebouncedState = false; isFallingEdge = true; }
//         else { isFallingEdge = false; }
//     }
//     else
//     {
//         isFallingEdge = isValueFallingEdge;
//     }

//     return isFallingEdge;
//     // return (p_aIn->UseEdgePin == true) ? (Debounce_PollFallingEdge(&p_aIn->EdgePin)) : ((p_aIn->Value_U16 == 0U) && (p_aIn->ValuePrev_U16 > 0U));
// }

// static inline uint16_t _MotAnalogUser_AIn_GetValue(const MotAnalogUser_AIn_T * p_aIn)
// {
//     return (_MotAnalogUser_AIn_GetIsOn(p_aIn) == true) ? p_aIn->Value_Scalar16 : 0U; /* Check IsOn again, If !IsOn Value_U16 remains prev captured value*/
// }