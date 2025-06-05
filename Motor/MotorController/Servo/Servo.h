

#include "Motor/Motor/Motor_Config.h"
#include "Motor/Motor/Motor_User.h"
#include "Motor/Motor/Motor_StateMachine.h"
#include "../MotMotors/MotMotors.h"
#include "Motor/Motor/MotorTimeRef.h"
// #include "Motor/Motor/Motor_Include.h"

#include "Transducer/Blinky/Blinky.h"
#include "Transducer/Thermistor/Thermistor.h"
#include "Transducer/VMonitor/VMonitor.h"

#include "Peripheral/Analog/Analog.h"
#include "Peripheral/NvMemory/Flash/Flash.h"
#include "Peripheral/NvMemory/EEPROM/EEPROM.h"
#include "Peripheral/Serial/Serial.h"
#if defined(CONFIG_MOTOR_CONTROLLER_CAN_BUS_ENABLE)
#include "Peripheral/CanBus/CanBus.h"
#endif

#include "Utility/Timer/Timer.h"
#include "Utility/StateMachine/StateMachine.h"
#include "Utility/Protocol/Protocol.h"
#if defined(CONFIG_MOTOR_CONTROLLER_SHELL_ENABLE)
#include "Utility/Shell/Shell.h"
#endif
#include "Utility/BootRef/BootRef.h"
#include "Type/Array/struct_array.h"

#include "Transducer/Voltage/Linear_Voltage.h"
#include "Math/Linear/Linear.h"


#include <stdint.h>
#include <string.h>


typedef enum MotorController_ServoMode
{
    MOTOR_CONTROLLER_SERVO_MODE_ENTER,
    MOTOR_CONTROLLER_SERVO_MODE_EXIT,
    // POSITION_PUSE_TIME
}
MotorController_ServoMode_T;

/******************************************************************************/
/*
    Servo
*/
/******************************************************************************/
#ifdef CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE
static inline void MotorController_User_InputServoMode(MotorController_T * p_mc, MotorController_ServoMode_T servoMode) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, servoMode); }
static inline bool MotorController_User_IsServoState(MotorController_T * p_mc) { return (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO); }

static inline void MotorController_User_EnterServoMode(MotorController_T * p_mc) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_INPUT_VALUE_NULL); }
static inline void MotorController_User_ExitServoMode(MotorController_T * p_mc) { _StateMachine_ProcAsyncInput(&p_mc->StateMachine, MCSM_INPUT_SERVO, STATE_INPUT_VALUE_NULL); }

// static inline void MotorController_User_StartControlMode(MotorController_T * p_mc, uint8_t feedbackMode)
// {
//     if (StateMachine_GetActiveStateId(&p_mc->StateMachine) == MCSM_STATE_ID_SERVO)
//     {
//         MotorController_StartControlModeAll(p_mc, feedbackMode);
//     }
// }
#endif

#if defined(CONFIG_MOTOR_CONTROLLER_SERVO_ENABLE) && defined(CONFIG_MOTOR_CONTROLLER_SERVO_EXTERN_ENABLE)
extern void MotorController_ServoExtern_Start(MotorController_T * p_mc);
extern void MotorController_ServoExtern_Proc(MotorController_T * p_mc);
extern void MotorController_ServoExtern_SetCmd(MotorController_T * p_mc, int32_t cmd);
#endif
