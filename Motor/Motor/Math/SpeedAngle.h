#include "Math/Fixed/fract16.h"

// #include "Type/Mux.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct SpeedAngle
{
    angle16_t MechanicalAngle;
    angle16_t ElectricalAngle;  /* Angle Feedback. Shared E-Cycle edge detect, User output */
    angle16_t AngularSpeed_DegPerCycle;
    int32_t Speed_Fract16;      /* fract16 [-32767:32767]*2 Speed Feedback Variable. -/+ => virtual CW/CCW */
}
SpeedAngle_T;

typedef struct
{
    SpeedAngle_T SpeedAngle;
    // Ramp_T SpeedRamp;  /* Speed Ramp */
    // PID_T SpeedPid;  /* Speed PID */
}
SpeedAngle_Feedback_T;


// static inline void SpeedAngle(SpeedAngle_T *    )

/* SpeedAngle OuterLoop */
// static inline void SpeedAngle_ProcFeedback(SpeedAngle_T *  , PID_T *  )