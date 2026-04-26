#pragma once

/******************************************************************************/
/*!
    @file   Motor_State_Sketch.h
    @brief  ARCHITECTURE SKETCH — not built, not included.

            Thread-grouped decomposition of Motor_State_T into pure container
            of sub-blocks. Each sub-block is owned by exactly one writer thread
            (SWMR discipline). Outer Motor_T orchestrates cross-thread reads.

            Primary cleavage:
                20 kHz FOC loop  (PWM ISR)
                1  kHz Speed loop (ms thread)

            Secondary blocks fall out by writer:
                - Supervisor / StateMachine   (event-driven, 1 ms tick)
                - Limits arbitration          (multi-writer LimitArray reducers)
                - Sensor                      (sensor-rate ISR; already typed)
                - OpenLoop / Calibration      (sub-state owners, run inside FOC tick)
                - HeatMonitor                 (1 ms tick; already typed)
                - Config                      (init/NVM only)
*/
/******************************************************************************/

/*============================================================================*\
                            THREAD TOPOLOGY
\*============================================================================*/
/*
    Writer  →  Sub-block                            ←  Readers
    --------------------------------------------------------------------
    PWM ISR    Motor_Foc_State_T                       (telemetry: const)
    PWM ISR    Motor_PhaseInput_State_T                FOC ISR
    PWM ISR    Motor_OpenLoop_State_T  *sub-state*     FOC ISR
    PWM ISR    Motor_Calibration_State_T *sub-state*   FOC ISR
    PWM ISR    Motor_ControlTimer_T  (counter only)    Supervisor

    1 kHz      Motor_Speed_State_T                     FOC ISR (TorqueRamp out)
    1 kHz      Motor_VRamp_State_T                     FOC ISR
    1 kHz      Motor_UserReq_State_T                   Supervisor (latched)

    1 ms /     Motor_Supervisor_State_T                FOC ISR (Direction,
    event        (StateMachine, Direction, FeedbackMode,                 FeedbackMode read-only)
                  FaultFlags, CalibrationStateIndex,
                  cached directional limits)

    edge       Motor_Limits_State_T                    Supervisor (recomputes
                 (LimitArray reducers; multi-writer                cached directional)
                  via LimitArray_Augments_T)

    sensor ISR Motor_Sensor_State_T (RotorSensor_State_T) FOC ISR (angle, speed)

    1 ms       Motor_HeatMonitor_State_T (HeatMonitor_State_T)  Supervisor

    init/NVM   Motor_Config_T                          all (read-only at runtime)
    --------------------------------------------------------------------

    SWMR boundaries (single-writer-multiple-reader):
      • TorqueRamp output: 1 kHz writer → 20 kHz reader (atomic int16, no lock)
      • SpeedUpdateFlag : 20 kHz writer (capture sync) → 1 kHz reader
      • FaultFlags      : any thread sets bits; supervisor reads/clears
      • LimitArray_Active: edge writers → supervisor reads on change

    Cross-thread reads use volatile or memory-order acquire/release
    where the field is wider than a register-atomic write.
*/

#include "Phase/Phase.h"
#include "Phase_Input/Phase_Input.h"
#include "Sensor/RotorSensor.h"
#include "VBus/VBus.h"
#include "Math/FOC.h"
#include "Math/Ramp/Ramp.h"
#include "Math/PID/PID.h"
#include "Math/Angle/Angle.h"
#include "Math/Accumulator/Accumulator.h"
#include "Transducer/Monitor/Heat/HeatMonitor.h"
#include "Framework/StateMachine/_StateMachine.h"
#include "Type/Array/LimitArray/LimitArray.h"
#include "Motor_Limits.h"
#include "Motor_Config.h"

#include <stdint.h>
#include <stdbool.h>


/*============================================================================*\
                  20 kHz BLOCK — written by PWM ISR (FOC tick)
\*============================================================================*/
/*
    Owned by the PWM ISR. The only writer. Readers outside the ISR (telemetry,
    supervisor, var/protocol) take const Motor_Foc_State_T * and accept that
    multi-byte fields may tear unless guarded by SpeedUpdateFlag handshake.

    Hot path. Cache-line aligned candidate. Keep small; keep contiguous.
*/
typedef struct Motor_Foc_State
{
    volatile Phase_Input_T   PhaseInput;        /* Phase V/I capture buffers */
    FOC_T                    Foc;               /* d/q transforms, vd/vq, id/iq */
    PID_T                    PidIq;
    PID_T                    PidId;
    /* PID_T                 PidIPhase; */
}
Motor_Foc_State_T;

/*
    Free-running 20 kHz counter. Single 32-bit word — register-atomic on M4.
    Read by supervisor for elapsed-time queries; written only by FOC ISR.
*/
typedef struct Motor_ControlTimer
{
    volatile uint32_t Base;                     /* 20 kHz tick counter */
}
Motor_ControlTimer_T;

/*
    Sub-state of FOC ISR. Active only during open-loop start-up; otherwise dormant.
    Lifetime: enter on RAMP/STARTUP state, exit on transition to closed loop.
    Same writer as Motor_Foc_State_T (PWM ISR).
*/
typedef struct Motor_OpenLoop_State
{
    Ramp_T                   SpeedRamp;
    Ramp_T                   IRamp;
    /* Ramp_T                TorqueRamp; */
    Angle_T                  Angle;
    Angle_SpeedFractRef_T    SpeedRef;
}
Motor_OpenLoop_State_T;

/*
    Sub-state of FOC ISR. Active only during calibration sequence.
    Same writer as Motor_Foc_State_T (PWM ISR). FilterA/B/C are
    per-phase accumulators used by I-zero / encoder calibration.
*/
typedef struct Motor_Calibration_State
{
    Accumulator_T            FilterA;
    Accumulator_T            FilterB;
    Accumulator_T            FilterC;
    /* Step + bias state for ParamId moves here when re-enabled. */
}
Motor_Calibration_State_T;


/*============================================================================*\
                1 kHz BLOCK — written by ms-tick / speed thread
\*============================================================================*/
/*
    Outer loop. Speed measurement → PI → TorqueRamp output.
    The TorqueRamp output is the SWMR boundary: 1 kHz writer, 20 kHz reader.
    Reader (FOC ISR) takes a snapshot of Ramp output; do not let FOC reach
    into PidSpeed internals.
*/
typedef struct Motor_Speed_State
{
    Ramp_T                   SpeedRamp;         /* Setpoint shaping */
    PID_T                    PidSpeed;          /* Speed PI */
    Ramp_T                   TorqueRamp;        /* OUTPUT to FOC: torque/I setpoint */
    /* PID_T                 PidPosition; */

    volatile bool            SpeedUpdateFlag;   /* Capture-sync handshake (20 kHz → 1 kHz) */
}
Motor_Speed_State_T;

/*
    Voltage-mode ramp. 1 kHz writer; 20 kHz reader (FOC voltage path).
    Separate from torque path so V-mode and I-mode coexist without coupling.
*/
typedef struct Motor_VRamp_State
{
    Ramp_T                   VRamp;
}
Motor_VRamp_State_T;

/*
    User input buffers. Written by host/protocol thread (any-rate).
    Read by 1 kHz speed thread, which latches into ramps under StateMachine guard.
    Two int16s — register-atomic on M4. No lock needed; supervisor decides when
    to apply.
*/
typedef struct Motor_UserReq_State
{
    int16_t                  UserSpeedReq;
    int16_t                  UserTorqueReq;
}
Motor_UserReq_State_T;


/*============================================================================*\
            SUPERVISOR BLOCK — written by 1 ms tick / event handlers
\*============================================================================*/
/*
    State machine, mode bits, fault flags. Edge-rate writes; both the FOC ISR
    and the speed loop *read* these (read-mostly, register-atomic flags).

    Cached directional limits live here because the supervisor recomputes
    them on direction/limit changes. The 20 kHz FOC reads them as int16 each
    cycle — register-atomic, SWMR.
*/
typedef struct Motor_Supervisor_State
{
    StateMachine_Active_T    StateMachine;

    Motor_Direction_T        Direction;         /* read by FOC, written by SM */
    Motor_FeedbackMode_T     FeedbackMode;      /* read by FOC, written by SM */
    Motor_FaultFlags_T       FaultFlags;        /* multi-writer (set), supervisor reads/clears */
    uint8_t                  CalibrationStateIndex;

    /* Cached directional limits — derived; recomputed on SM events */
    int16_t                  SpeedLimitCcw_Fract16;
    int16_t                  SpeedLimitCw_Fract16;
    int16_t                  ILimitCcw_Fract16;
    int16_t                  ILimitCw_Fract16;
}
Motor_Supervisor_State_T;


/*============================================================================*\
                LIMITS BLOCK — multi-writer reducers (edge-driven)
\*============================================================================*/
/*
    LimitArray-style aggregators. Many independent writers (winding heat,
    stall, field-weaken, host commands) post values; the reducer computes
    the active min. Supervisor reads the reduced value and recomputes
    cached directional limits in Motor_Supervisor_State_T.

    Per-source limits, not the directional cache.
*/
typedef struct Motor_Limits_State
{
    /* Source-side limit fract16 caps */
    uint16_t                 SpeedLimitForward_Fract16;
    uint16_t                 SpeedLimitReverse_Fract16;
    uint16_t                 ILimitMotoring_Fract16;
    uint16_t                 ILimitGenerating_Fract16;

    /* LimitArray reducers — per-source contributor tables */
    limit_t                  ILimits[MOTOR_I_LIMIT_COUNT];
    LimitArray_Augments_T    ILimitsActive;
    limit_t                  IGenLimits[MOTOR_I_GEN_LIMIT_COUNT];
    LimitArray_Augments_T    IGenLimitsActive;
    limit_t                  SpeedLimits[MOTOR_SPEED_LIMIT_COUNT];
    LimitArray_Augments_T    SpeedLimitsActive;
}
Motor_Limits_State_T;


/*============================================================================*\
                          PURE CONTAINER
\*============================================================================*/
/*
    Composition only. No methods on Motor_State_T except Motor_State_Init,
    which delegates to per-block init. Every other operation is either:
        - sub-block API:      Motor_<Block>_*(Motor_<Block>_State_T *, ...)
        - orchestrator API:   Motor_*(Motor_T *, ...)   ← takes outer context

    Layout grouping is also thread-affinity grouping — cache and any future
    MPU region placement can follow the writer.
*/
typedef struct Motor_State
{
    /* 20 kHz domain */
    Motor_Foc_State_T            Foc;
    Motor_ControlTimer_T         ControlTimer;
    Motor_OpenLoop_State_T       OpenLoop;        /* sub-state, dormant outside startup */
    Motor_Calibration_State_T    Calibration;     /* sub-state, dormant outside calibration */

    /* 1 kHz domain */
    Motor_Speed_State_T          Speed;
    Motor_VRamp_State_T          VMode;
    Motor_UserReq_State_T        UserReq;

    /* Supervisor / event domain */
    Motor_Supervisor_State_T     Supervisor;

    /* Edge-driven limit aggregation */
    Motor_Limits_State_T         Limits;

    /* Sensor ISR domain — already typed */
    const RotorSensor_T *        p_ActiveSensor;
    RotorSensor_State_T          SensorState;

    /* 1 ms domain — already typed */
    HeatMonitor_State_T          HeatMonitor;

    /* External peer (controller-owned) */
    VBus_T *                     p_VBus;          /* back-pointer; lifetime > Motor */

    /* Init/NVM domain */
    Motor_Config_T               Config;

#if defined(MOTOR_SIX_STEP_ENABLE)
    /* Six-step is its own writer-group; sketch later. */
    /* Motor_SixStep_State_T     SixStep; */
#endif

#if defined(MOTOR_DEBUG_ENABLE) && !defined(NDEBUG)
    volatile uint32_t            DebugCounter;
#endif
}
Motor_State_T;


/*============================================================================*\
                EXAMPLE SUB-BLOCK APIs (illustrative)
\*============================================================================*/
/*
    Sub-block APIs take a pointer to their block only. Cross-block calls
    are illegal at this layer — they get promoted to Motor_T orchestrator API.
*/

/* --- 20 kHz domain --- */
void       Motor_Foc_ProcLoop      (Motor_Foc_State_T * p_foc,
                                    const Motor_Supervisor_State_T * p_sv,   /* dir, mode read */
                                    const Motor_Speed_State_T      * p_spd,  /* TorqueRamp out */
                                    const RotorSensor_State_T      * p_sen,
                                    const VBus_T                   * p_vbus);

void       Motor_OpenLoop_Step     (Motor_OpenLoop_State_T * p_ol);
void       Motor_Calibration_Step  (Motor_Calibration_State_T * p_cal,
                                    const Phase_Input_T * p_phase);

uint32_t   Motor_ControlTimer_Get  (const Motor_ControlTimer_T * p_t);
void       Motor_ControlTimer_Tick (Motor_ControlTimer_T * p_t);


/* --- 1 kHz domain --- */
void       Motor_Speed_ProcLoop    (Motor_Speed_State_T * p_spd,
                                    const Motor_Supervisor_State_T * p_sv,
                                    const RotorSensor_State_T      * p_sen);

int16_t    Motor_Speed_GetTorqueOut(const Motor_Speed_State_T * p_spd);   /* SWMR read */
void       Motor_Speed_SetUpdateFlag(Motor_Speed_State_T * p_spd);        /* from FOC ISR */

void       Motor_VRamp_Step        (Motor_VRamp_State_T * p_v);
void       Motor_UserReq_SetSpeed  (Motor_UserReq_State_T * p_u, int16_t req);
void       Motor_UserReq_SetTorque (Motor_UserReq_State_T * p_u, int16_t req);


/* --- Supervisor --- */
bool       Motor_Supervisor_TryEnable    (Motor_Supervisor_State_T * p_sv);
void       Motor_Supervisor_OnFault      (Motor_Supervisor_State_T * p_sv,
                                          Motor_FaultFlags_T flags);
void       Motor_Supervisor_RecacheLimits(Motor_Supervisor_State_T * p_sv,
                                          const Motor_Limits_State_T * p_lim);

Motor_Direction_T   Motor_Supervisor_GetDirection (const Motor_Supervisor_State_T * p_sv);
Motor_FeedbackMode_T Motor_Supervisor_GetFeedback (const Motor_Supervisor_State_T * p_sv);


/* --- Limits --- */
void       Motor_Limits_PostI      (Motor_Limits_State_T * p_lim, uint8_t source, limit_t cap);
void       Motor_Limits_PostSpeed  (Motor_Limits_State_T * p_lim, uint8_t source, limit_t cap);
limit_t    Motor_Limits_ReduceI    (const Motor_Limits_State_T * p_lim);
limit_t    Motor_Limits_ReduceSpeed(const Motor_Limits_State_T * p_lim);


/* --- Container init only --- */
void       Motor_State_Init        (Motor_State_T * p);


/*============================================================================*\
            ORCHESTRATOR API — takes Motor_T (outer context)
\*============================================================================*/
/*
    These cross sub-blocks. They live ONLY at the outer Motor_T level.
    Pattern: read inputs from peer blocks, hand to the writer block.
    No sub-block API ever calls another sub-block API directly.
*/
struct Motor;
typedef struct Motor Motor_T;

void       Motor_TryEnableControl  (Motor_T * p_motor);   /* Supervisor + Limits + Foc */
void       Motor_OnFault           (Motor_T * p_motor);   /* Foc disable + Output + SM event */
void       Motor_RunFocLoop        (Motor_T * p_motor);   /* Foc + Sensor + Speed + Supervisor */
void       Motor_RunSpeedLoop      (Motor_T * p_motor);   /* Speed + Sensor + Supervisor */
void       Motor_LimitsChanged     (Motor_T * p_motor);   /* Limits → Supervisor recache */


/*============================================================================*\
                        CALL-SITE TRANSFORMATION
\*============================================================================*/
/*
    Before:   ufract16_t Motor_VLimitsCcw(const Motor_State_T * p_motor);
              ↑ reaches into Supervisor.SpeedLimitCcw_Fract16
              ↑ reaches into VBus speed-derate (via p_motor->p_VBus)

    After (option A — sub-block local, no derate):
              ufract16_t Motor_Supervisor_VLimitsCcw(const Motor_Supervisor_State_T *);

    After (option B — orchestrator, includes derate):
              ufract16_t Motor_VLimitsCcw(const Motor_T * p_motor);
              { return apply_derate(Motor_Supervisor_VLimitsCcw(&p_motor->P_STATE->Supervisor),
                                    VBus_SpeedDerateScale(p_motor->P_STATE->p_VBus)); }

    The derate composes Supervisor + VBus → orchestrator level. Pure cached
    limit lookup is sub-block local.
*/


/*============================================================================*\
                MIGRATION ORDER (least to most coupled)
\*============================================================================*/
/*
    1.  Motor_Calibration_State_T   — sub-state of FOC, easily isolated.
                                      FilterA/B/C move out of Motor_State_T verbatim.
    2.  Motor_OpenLoop_State_T      — same; sub-state of FOC, isolated lifetime.
    3.  Motor_Limits_State_T        — already conceptually separate.
                                      LimitArray_Augments_T already lives in its own type.
    4.  Motor_UserReq_State_T       — two int16s; lowest-risk extraction.
    5.  Motor_Speed_State_T         — 1 kHz block. Cleaves cleanly at TorqueRamp output.
    6.  Motor_Supervisor_State_T    — pulls StateMachine, mode bits, fault flags,
                                      cached directional limits. Touches everything,
                                      so do it after the sub-blocks above are out.
    7.  Motor_Foc_State_T           — last. By this point everything else is gone
                                      and what remains *is* the FOC block.

    Verification at each step: no Motor_*(Motor_State_T *) function reaches
    fields outside its newly-named block. If one does, it gets renamed
    Motor_<Block>_* (move down) or Motor_*(Motor_T *) (promote up).
*/
