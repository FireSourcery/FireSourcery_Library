#include "../Xcvr/Xcvr.h"
#include "Serial.h"


extern const Xcvr_Interface_T SERIAL_XCVR;


// typedef struct Serial_Xcvr
// {
//     Serial_T * p_Xcvr; /* Xcvr data struct */
//     Xcvr_Interface_T * p_Class;
// }
// Serial_Xcvr_T;

// static inline void Serial_Interface_(Xcvr_T * p_xcvr )
// {
//     p_xcvr->p_Xcvr.value
// }

// #define SERIAL_XCVR_INIT(p_Serial) { .p_Xcvr = p_Serial, .p_Class = &SERIAL_XCVR, }