#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef int16_t fract16_t;  typedef int32_t accum32_t;  typedef uint32_t state_t;
typedef enum { MOTOR_STATE_ID_STOP, MOTOR_STATE_ID_RUN } Motor_StateId_T;
typedef enum { MOTOR_DIRECTION_CW, MOTOR_DIRECTION_CCW } Motor_Direction_T;
typedef union { struct { uint8_t a : 1; }; uint8_t Value; } Motor_FaultFlags_T;
typedef union { struct { uint8_t b : 1; }; uint8_t Value; } Motor_FeedbackMode_T;
typedef struct Motor_Config { int PidSpeedKp, PidSpeedKi; } Motor_Config_T;
typedef struct Motor_Context { int Direction; Motor_Config_T Config; } Motor_Context_T;
typedef struct Motor { Motor_Context_T * P_MOTOR; } Motor_T;
typedef enum { UNITS_RPM, UNITS_AMPS, UNITS_VOLTS, UNITS_NONE, UNITS_PID } MotVarUnits_T;

/* --- state context --- */
static inline accum32_t          Motor_User_GetSpeed_Fract16(const Motor_Context_T * p) { return p->Direction * 100; }
static inline fract16_t          Motor_GetIPhase_Fract16(const Motor_Context_T * p)     { return (fract16_t)p->Direction; }
static inline Motor_StateId_T    Motor_GetStateId(const Motor_Context_T * p)            { return (Motor_StateId_T)(p->Direction != 0); }
static inline state_t            Motor_GetPathId(const Motor_Context_T * p)             { return (state_t)p->Direction; }
static inline Motor_FaultFlags_T Motor_GetFaultFlags(const Motor_Context_T * p)         { return (Motor_FaultFlags_T){ .Value = (uint8_t)p->Direction }; }
/* --- dev context (same group!) --- */
static inline fract16_t          Motor_GetIBus_Fract16(const Motor_T * p)               { return (fract16_t)p->P_MOTOR->Direction; }
/* --- setpoints: uniform void(state, int) --- */
static inline void Motor_SetSpeedCmd(Motor_Context_T * p, int v)   { p->Direction = v; }
static inline void Motor_SetTorqueCmd(Motor_Context_T * p, int v)  { p->Direction = v; }
static inline void Motor_SetICmd(Motor_Context_T * p, int v)       { p->Direction = v; }
/* --- board: NO context --- */
static inline fract16_t Phase_VRated_Fract16(void) { return 100; }
static inline fract16_t Phase_IRatedPeak_Fract16(void) { return 200; }
static inline uint16_t  Phase_VMaxVolts(void) { return 300; }
/* --- config: keyed, already uniform --- */
static inline int  _Motor_Var_ConfigPid_Get(const Motor_Config_T * p, int id) { return id ? p->PidSpeedKi : p->PidSpeedKp; }
static inline void _Motor_Var_ConfigPid_Set(Motor_Config_T * p, int id, int v) { if (id) p->PidSpeedKi = v; else p->PidSpeedKp = v; }
