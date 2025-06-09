/******************************************************************************/
/*!
    @file   MotorController_Var.c
    @author FireSourcery
    @brief  Refactored to use new MotVarId scheme with proper TypeType dispatching
*/
/******************************************************************************/
#include "MotorController_Var.h"

/******************************************************************************/
/*
    Forward Declarations - Only for complex functions
*/
/******************************************************************************/
static int32_t _MotVarId_GetService(const MotorController_T * p_context, MotVarId_T id);
static MotVarId_Status_T _MotVarId_SetService(const MotorController_T * p_context, MotVarId_T id, int32_t value);
static MotVarId_Status_T _MotVarId_ExecuteCommand(const MotorController_T * p_context, MotVarId_T id, int32_t value);
static MotVarId_Status_T _MotVarId_ExecuteMotorContextCommand(const MotorController_T * p_context, uint8_t nameBase, int32_t value);
static MotVarId_Status_T _MotVarId_ExecuteBlockingCommand(const MotorController_T * p_context, uint8_t nameBase, int32_t value);

/* Shorthand */
static inline Motor_T * MotorContext(const MotorController_T * p_context, MotVarId_T id) { MotMotors_ContextAt(&p_context->MOTORS, id.Instance); }
static inline Motor_State_T * MotorState(const MotorController_T * p_context, MotVarId_T id) { return MotMotors_StateAt(&p_context->MOTORS, id.Instance); }


/******************************************************************************/
/*
    Service TypeType Dispatcher
*/
/******************************************************************************/
static int32_t _MotVarId_GetService(const MotorController_T * p_context, MotVarId_T id)
{
    switch ((MotVarId_Type_Service_T)id.NameType)
    {
        // case MOT_VAR_ID_TYPE_GENERAL_VAR_OUT: return _MotVarId_GetGeneralVar(p_context, id.NameBase);
        // case MOT_VAR_ID_TYPE_GENERAL_CONFIG: return _MotVarId_GetGeneralConfig(p_context, id.NameBase);
        // case MOT_VAR_ID_TYPE_MONITOR_VAR_OUT: return _MotVarId_GetMonitorVar(p_context, id.NameBase, id.Instance);
        // case MOT_VAR_ID_TYPE_V_MONITOR_SOURCE_CONFIG:
        //     return VMonitor_GetConfig(&p_context->V_SOURCE.MONITOR, id.NameBase);
        // case MOT_VAR_ID_TYPE_V_MONITOR_AUX_CONFIG:
        //     return (id.Instance == 0) ? VMonitor_GetConfig(&p_context->V_ACCESSORIES.MONITOR, id.NameBase) :
        //            (id.Instance == 1) ? VMonitor_GetConfig(&p_context->V_ANALOG.MONITOR, id.NameBase) : 0;
        // case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:
        //     return HeatMonitor_GroupContext_GetConfig(&p_context->HEAT_MOSFETS, id.Instance, id.NameBase);
        // case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG:
        //     return HeatMonitor_GetConfig(&p_context->HEAT_PCB.MONITOR, id.NameBase);
        // case MOT_VAR_ID_TYPE_PROTOCOL_CONFIG:
        //     return (id.Instance < p_context->PROTOCOL_COUNT) ?
        //            Protocol_Config_Get(&p_context->P_PROTOCOLS[id.Instance], id.NameBase) : 0;
        // case MOT_VAR_ID_TYPE_BOOT_REF_CONFIG: return _MotVarId_GetBootRefConfig(p_context, id.NameBase);
        // case MOT_VAR_ID_TYPE_DEBUG: return _MotVarId_GetDebug(p_context, id.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _MotVarId_SetService(const MotorController_T * p_context, MotVarId_T id, int32_t value)
{
    switch ((MotVarId_Type_Service_T)id.NameType)
    {
        case MOT_VAR_ID_TYPE_GENERAL_VAR_OUT: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_GENERAL_CONFIG: return _MotVarId_SetGeneralConfig(p_context, id.NameBase, value);
        // case MOT_VAR_ID_TYPE_ANALOG_USER_VAR_OUT: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_ANALOG_USER_CONFIG:
        //     MotAnalogUser_Config_Set(&p_context->ANALOG_USER, id.NameBase, id.Instance, value);
        //     return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_MONITOR_VAR_OUT: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_V_MONITOR_SOURCE_CONFIG:
        //     VMonitor_SetConfig(&p_context->V_SOURCE.MONITOR, id.NameBase, value);
        //     return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_V_MONITOR_AUX_CONFIG:
        //     if (id.Instance == 0) {
        //         VMonitor_SetConfig(&p_context->V_ACCESSORIES.MONITOR, id.NameBase, value);
        //     } else if (id.Instance == 1) {
        //         VMonitor_SetConfig(&p_context->V_ANALOG.MONITOR, id.NameBase, value);
        //     } else {
        //         return MOT_VAR_STATUS_ERROR;
        //     }
        //     return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_HEAT_MONITOR_MOSFETS_CONFIG:
        //     return HeatMonitor_GroupContext_SetConfig(&p_context->HEAT_MOSFETS, id.Instance, id.NameBase, value) ?
        //            MOT_VAR_STATUS_OK : MOT_VAR_STATUS_ERROR;
        // case MOT_VAR_ID_TYPE_HEAT_MONITOR_PCB_CONFIG:
        //     HeatMonitor_SetConfig(&p_context->HEAT_PCB.MONITOR, id.NameBase, value);
        //     return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_PROTOCOL_CONFIG:
        //     if (id.Instance < p_context->PROTOCOL_COUNT) {
        //         Protocol_Config_Set(&p_context->P_PROTOCOLS[id.Instance], id.NameBase, value);
        //         return MOT_VAR_STATUS_OK;
        //     }
        //     return MOT_VAR_STATUS_ERROR;
        // case MOT_VAR_ID_TYPE_BOOT_REF_CONFIG: return _MotVarId_SetBootRefConfig(p_context, id.NameBase, value);
        // case MOT_VAR_ID_TYPE_BOARD_REF: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_DEBUG: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        default: return MOT_VAR_STATUS_ERROR;
    }
}

/******************************************************************************/
/*
    Motor Var TypeType Dispatcher
*/
/******************************************************************************/
static int32_t _MotVarId_GetMotorVar(const MotorController_T * p_context, MotVarId_T id)
{
    Motor_State_T * p_motor = MotorState(p_context, id);
    if (p_motor == NULL) return 0;

    switch ((MotVarId_Type_MotorVar_T)id.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_METRICS: return Motor_VarOutput_Get(p_motor, id.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_FOC: return Motor_VarOutput_Foc_Get(p_motor, id.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_IO: return Motor_VarIO_Get(p_motor, id.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_VAR_CMD: return 0; /* Commands are write-only */
        case MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING: return Motor_VarConfig_Pid_Get(p_motor, id.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _MotVarId_SetMotorVar(const MotorController_T * p_context, MotVarId_T id, int32_t value)
{
    Motor_State_T * p_motor = MotorState(p_context, id);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR;

    switch ((MotVarId_Type_MotorVar_T)id.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_METRICS: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_FOC: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_OUT_SENSOR: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_IO: Motor_VarIO_Set(MotorContext(p_context, id), id.NameBase, value); return MOT_VAR_STATUS_OK;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_CMD: Motor_VarInput_Set(MotorContext(p_context, id), id.NameBase, value); return MOT_VAR_STATUS_OK;
        case MOT_VAR_ID_TYPE_MOTOR_VAR_PID_TUNNING: Motor_VarConfig_Pid_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        default: return MOT_VAR_STATUS_ERROR;
    }
}
/******************************************************************************/
/*
    Motor Config TypeType Dispatcher
*/
/******************************************************************************/
static int32_t _MotVarId_GetMotorConfig(const MotorController_T * p_context, MotVarId_T id)
{
    Motor_State_T * p_motor = MotorState(p_context, id);
    if (p_motor == NULL) return 0;

    switch ((MotVarId_Type_MotorConfig_T)id.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION: return Motor_VarConfig_Calibration_Get(p_motor, id.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_ACTUATION: return Motor_VarConfig_Actuation_Get(p_motor, id.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_PID: return Motor_VarConfig_Pid_Get(p_motor, id.NameBase);
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION_ALIAS: return Motor_VarConfig_CalibrationAlias_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG_THERMISTOR: return Motor_VarConfig_Thermistor_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG_ROUTINE: return Motor_VarConfig_Routine_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG_BOARD_REF: return Motor_BoardRef_Get(p_context, id.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _MotVarId_SetMotorConfig(const MotorController_T * p_context, MotVarId_T id, int32_t value)
{
    Motor_State_T * p_motor = MotorState(p_context, id);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR;

    switch ((MotVarId_Type_MotorConfig_T)id.NameType)
    {
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION: Motor_VarConfig_Calibration_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_ACTUATION: Motor_VarConfig_Actuation_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_PID: Motor_VarConfig_Pid_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG_CALIBRATION_ALIAS: Motor_VarConfig_CalibrationAlias_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG_THERMISTOR: Motor_VarConfig_Thermistor_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG_ROUTINE: Motor_VarConfig_Routine_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        case MOT_VAR_ID_TYPE_MOTOR_CONFIG_BOARD_REF: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        default: return MOT_VAR_STATUS_ERROR;
    }
}

/******************************************************************************/
/*
    Motor Sensor TypeType Dispatcher
*/
/******************************************************************************/
static int32_t _MotVarId_GetMotorSensor(const MotorController_T * p_context, MotVarId_T id)
{
    Motor_State_T * p_motor = MotorState(p_context, id);
    if (p_motor == NULL) return 0;

    switch ((MotVarId_Type_MotorSensor_T)id.NameType)
    {
        // case MOT_VAR_ID_TYPE_MOTOR_SENSOR_STATE: return Motor_Sensor_State_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_SENSOR_CONFIG: return Motor_Sensor_Config_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_HALL_STATE: return Motor_Hall_State_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG: return Motor_Hall_Config_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE: return Motor_Encoder_State_Get(p_motor, id.NameBase);
        // case MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG: return Motor_Encoder_Config_Get(p_motor, id.NameBase);
        default: return 0;
    }
}

static MotVarId_Status_T _MotVarId_SetMotorSensor(const MotorController_T * p_context, MotVarId_T id, int32_t value)
{
    Motor_State_T * p_motor = MotorState(p_context, id);
    if (p_motor == NULL) return MOT_VAR_STATUS_ERROR;

    switch ((MotVarId_Type_MotorSensor_T)id.NameType)
    {
        // case MOT_VAR_ID_TYPE_MOTOR_SENSOR_STATE: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_MOTOR_SENSOR_CONFIG: Motor_Sensor_Config_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_MOTOR_HALL_STATE: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_MOTOR_HALL_CONFIG: Motor_Hall_Config_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        // case MOT_VAR_ID_TYPE_MOTOR_ENCODER_STATE: return MOT_VAR_STATUS_ERROR_READ_ONLY;
        // case MOT_VAR_ID_TYPE_MOTOR_ENCODER_CONFIG: Motor_Encoder_Config_Set(p_motor, id.NameBase, value); return MOT_VAR_STATUS_OK;
        // default: return MOT_VAR_STATUS_ERROR;
    }
}

/******************************************************************************/
/*
    Command TypeType Dispatcher
*/
/******************************************************************************/
static MotVarId_Status_T _MotVarId_ExecuteCommand(const MotorController_T * p_context, MotVarId_T id, int32_t value)
{
    switch ((MotVarId_Type_Command_T)id.NameType)
    {
        case MOT_VAR_ID_TYPE_COMMAND_MOTOR_CONTEXT: return _MotVarId_ExecuteMotorContextCommand(p_context, id.NameBase, value);
        // case MOT_VAR_ID_TYPE_COMMAND_MOT_DRIVE: return MotDrive_ExecuteCommand(p_context, id.NameBase, value);
        // case MOT_VAR_ID_TYPE_COMMAND_SYSTEM: return System_ExecuteCommand(p_context, id.NameBase, value);
        case MOT_VAR_ID_TYPE_COMMAND_BLOCKING: return _MotVarId_ExecuteBlockingCommand(p_context, id.NameBase, value);
        default: return MOT_VAR_STATUS_ERROR;
    }
}

/******************************************************************************/
/*
    Implementation Functions - Service Level
*/
/******************************************************************************/
static int32_t _MotVarId_GetGeneralVar(const MotorController_T * p_context, uint8_t nameBase)
{
    // switch ((MotorController_VarOutput_T)nameBase)
    // {
    //     case MOT_VAR_ZERO: return 0;
    //     case MOT_VAR_MILLIS: return SysTime_GetMillis();
    //     case MOT_VAR_MC_STATE: return MotorController_User_GetStateId(p_context);
    //     case MOT_VAR_MC_STATUS_FLAGS: return MotorController_User_GetStatusFlags(p_context).Value;
    //     case MOT_VAR_MC_FAULT_FLAGS: return MotorController_User_GetFaultFlags(p_context).Value;
    //     // case MOT_DRIVE_DIRECTION: return MotorController_User_GetDirection(p_context);
    //     default: return 0;
    // }
}

static int32_t _MotVarId_GetMonitorVar(const MotorController_T * p_context, uint8_t nameBase, uint8_t instance)
{
    // switch ((MotMonitor_VarOutput_T)nameBase)
    // {
    //     // case MOT_MONITOR_V_SOURCE: return VMonitor_GetValue(&p_context->V_SOURCE.MONITOR);
    //     // case MOT_MONITOR_V_ANALOG: return VMonitor_GetValue(&p_context->V_ANALOG.MONITOR);
    //     // case MOT_MONITOR_V_ACCS: return VMonitor_GetValue(&p_context->V_ACCESSORIES.MONITOR);
    //     // case MOT_MONITOR_HEAT_PCB: return HeatMonitor_GetValue(&p_context->HEAT_PCB.MONITOR);
    //     // case MOT_MONITOR_HEAT_MOSFETS: return HeatMonitor_GroupContext_GetValue(&p_context->HEAT_MOSFETS, instance);
    //     case MOT_MONITOR_HEAT_MOSFETS_1: return HeatMonitor_GroupContext_GetValue(&p_context->HEAT_MOSFETS, 0);
    //     case MOT_MONITOR_HEAT_MOSFETS_2: return HeatMonitor_GroupContext_GetValue(&p_context->HEAT_MOSFETS, 1);
    //     case MOT_MONITOR_HEAT_MOSFETS_3: return HeatMonitor_GroupContext_GetValue(&p_context->HEAT_MOSFETS, 2);
    //     default: return 0;
    // }
}

static int32_t _MotVarId_GetDebug(const MotorController_T * p_context, uint8_t nameBase)
{
    // Motor_State_T * p_motor = MotorState(p_context, 0);
    Motor_State_T * p_motor = NULL;
    if (p_motor == NULL) return 0;

    switch ((MotorController_VarOutput_Debug_T)nameBase)
    {
        case MOT_VAR_DEBUG0: return p_motor->DebugTime[4];
        case MOT_VAR_DEBUG1:
        case MOT_VAR_DEBUG2:
        case MOT_VAR_DEBUG3:
        case MOT_VAR_DEBUG4:
        case MOT_VAR_DEBUG5:
        case MOT_VAR_DEBUG6:
        case MOT_VAR_DEBUG7:
        default: return 0;
    }
}

static int32_t _MotVarId_GetGeneralConfig(const MotorController_T * p_context, uint8_t nameBase)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    switch ((MotorController_ConfigId_T)nameBase)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS: return p_mc->Config.VSupplyRef;
        case MOT_VAR_I_LIMIT_LOW_V: return p_mc->Config.VLowILimit_Fract16;
        case MOT_VAR_USER_INIT_MODE: return p_mc->Config.InitMode;
        case MOT_VAR_USER_INPUT_MODE: return p_mc->Config.InputMode;
        case MOT_VAR_OPT_DIN_FUNCTION: return p_mc->Config.OptDinMode;
        case MOT_VAR_OPT_SPEED_LIMIT: return p_mc->Config.OptSpeedLimit_Fract16;
        case MOT_VAR_OPT_I_LIMIT: return p_mc->Config.OptILimit_Fract16;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE: return p_mc->Config.BuzzerFlags;
        // case MOT_DRIVE_CONFIG_THROTTLE_MODE: return p_mc->Config.ThrottleMode;
        // case MOT_DRIVE_CONFIG_BRAKE_MODE: return p_mc->Config.BrakeMode;
        // case MOT_DRIVE_CONFIG_ZERO_MODE: return p_mc->Config.ZeroMode;
        default: return 0;
    }
}

static MotVarId_Status_T _MotVarId_SetGeneralConfig(const MotorController_T * p_context, uint8_t nameBase, int32_t value)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    switch ((MotorController_ConfigId_T)nameBase)
    {
        case MOT_VAR_V_SOURCE_REF_VOLTS: MotorController_User_SetVSupplyRef(p_context, value); break;
        case MOT_VAR_I_LIMIT_LOW_V: p_mc->Config.VLowILimit_Fract16 = value; break;
        case MOT_VAR_USER_INIT_MODE: p_mc->Config.InitMode = (MotorController_MainMode_T)value; break;
        case MOT_VAR_USER_INPUT_MODE: MotorController_User_SetInputMode(p_context, (MotorController_InputMode_T)value);  break;
        // case MOT_VAR_OPT_DIN_FUNCTION: MotorController_User_SetOptDinMode(p_mc, (MotorController_OptDinMode_T)value);  break;
        case MOT_VAR_OPT_SPEED_LIMIT: p_mc->Config.OptSpeedLimit_Fract16 = value; break;
        case MOT_VAR_OPT_I_LIMIT: p_mc->Config.OptILimit_Fract16 = value; break;
        // case MOT_VAR_BUZZER_FLAGS_ENABLE: p_mc->Config.BuzzerFlags = value; break;
        // case MOT_DRIVE_CONFIG_THROTTLE_MODE: p_mc->Config.ThrottleMode = (MotDrive_ThrottleMode_T)value; break;
        // case MOT_DRIVE_CONFIG_BRAKE_MODE: p_mc->Config.BrakeMode = (MotDrive_BrakeMode_T)value; break;
        // case MOT_DRIVE_CONFIG_ZERO_MODE: p_mc->Config.ZeroMode = (MotDrive_ZeroMode_T)value; break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

static int32_t _MotVarId_GetBootRefConfig(const MotorController_T * p_context, uint8_t nameBase)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    switch ((MotorController_Config_BootRef_T)nameBase)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT: return p_mc->BootRef.FastBoot;
        case MOT_VAR_BOOT_REF_BEEP: return p_mc->BootRef.Beep;
        case MOT_VAR_BOOT_REF_BLINK: return p_mc->BootRef.Blink;
        default: return 0;
    }
}

static MotVarId_Status_T _MotVarId_SetBootRefConfig(const MotorController_T * p_context, uint8_t nameBase, int32_t value)
{
    MotorController_State_T * p_mc = p_context->P_ACTIVE;

    switch ((MotorController_Config_BootRef_T)nameBase)
    {
        case MOT_VAR_BOOT_REF_FAST_BOOT: MotorController_User_SetFastBoot(p_mc, value); break;
        case MOT_VAR_BOOT_REF_BEEP: MotorController_User_SetBeep(p_mc, value); break;
        case MOT_VAR_BOOT_REF_BLINK: MotorController_User_SetBlink(p_mc, value); break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

/******************************************************************************/
/*
    Command Implementation Functions
*/
/******************************************************************************/
static MotVarId_Status_T _MotVarId_ExecuteMotorContextCommand(const MotorController_T * p_context, uint8_t nameBase, int32_t value)
{
    switch ((MotorController_Command_MotorContext_T)nameBase)
    {
        case MOTOR_CONTROLLER_USER_CMD: MotorController_User_SetCmdValue(p_context, value); break;
        // case MOTOR_CONTROLLER_USER_FEEDBACK_MODE: MotorController_User_SetFeedbackMode(p_context, value); break;
        // case MOT_DRIVE_VAR_THROTTLE: MotorController_User_SetCmdThrottle(p_context, value); break;
        // case MOT_DRIVE_VAR_BRAKE: MotorController_User_SetCmdBrake(p_context, value); break;
        // case MOT_DRIVE_CMD_DIRECTION: MotorController_User_SetDirection(p_context, (MotDrive_Direction_T)value); break;
        case MOTOR_CONTROLLER_OPT_SPEED_LIMIT_ON_OFF: MotorController_User_SetOptSpeedLimitOnOff(p_context, value); break;
        case MOTOR_CONTROLLER_OPT_I_LIMIT_ON_OFF: MotorController_User_SetOptILimitOnOff(p_context, value); break;
        case MOTOR_CONTROLLER_RELAY_TOGGLE: /* Implement relay toggle */ break;
        case MOTOR_CONTROLLER_METER_TOGGLE: /* Implement meter toggle */ break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}

static MotVarId_Status_T _MotVarId_ExecuteBlockingCommand(const MotorController_T * p_context, uint8_t nameBase, int32_t value)
{
    switch ((MotorController_Command_Blocking_T)nameBase)
    {
        // case MOTOR_CONTROLLER_CMD_CALIBRATE_ADC: MotorController_User_CalibrateADC(p_context); break;
        // case MOTOR_CONTROLLER_CMD_NVM_SAVE_CONFIG: MotorController_User_NvmSaveConfig(p_context); break;
        // case MOTOR_CONTROLLER_CMD_NVM_RESTORE_CONFIG: MotorController_User_NvmRestoreConfig(p_context); break;
        case MOTOR_CONTROLLER_CMD_CALIBRATE_ADC:        MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_CALIBRATE_ADC);         break;
        case MOTOR_CONTROLLER_CMD_NVM_SAVE_CONFIG:      MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_NVM_SAVE_CONFIG);       break;
        case MOTOR_CONTROLLER_CMD_NVM_RESTORE_CONFIG:   MotorController_User_InputLock(p_context, MOTOR_CONTROLLER_LOCK_NVM_RESTORE_CONFIG);    break;
        default: return MOT_VAR_STATUS_ERROR;
    }

    return MOT_VAR_STATUS_OK;
}



/******************************************************************************/
/*
    Main Dispatcher Functions - Public Interface
*/
/******************************************************************************/
int32_t MotorController_Var_Get(const MotorController_T * p_context, MotVarId_T varId)
{
    switch ((MotVarId_TypeType_T)varId.NameTypeType)
    {
        // case MOT_VAR_ID_TYPE_SERVICE: return _MotVarId_GetService(p_context, varId);
        // case MOT_VAR_ID_TYPE_MOTOR_VAR: return _MotVarId_GetMotorVar(p_context, varId);
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG: return _MotVarId_GetMotorConfig(p_context, varId);
        // case MOT_VAR_ID_TYPE_MOTOR_SENSOR: return _MotVarId_GetMotorSensor(p_context, varId);
        case MOT_VAR_ID_TYPE_COMMAND: return 0; /* Commands are write-only */
        default: return 0;
    }
}

MotVarId_Status_T MotorController_Var_Set(const MotorController_T * p_context, MotVarId_T varId, int32_t varValue)
{
    switch ((MotVarId_TypeType_T)varId.NameTypeType)
    {
        // case MOT_VAR_ID_TYPE_SERVICE: return _MotVarId_SetService(p_context, varId, varValue);
        // case MOT_VAR_ID_TYPE_MOTOR_VAR: return _MotVarId_SetMotorVar(p_context, varId, varValue);
        // case MOT_VAR_ID_TYPE_MOTOR_CONFIG:
        //     return MotorController_User_IsConfigState(p_context->P_ACTIVE) ?
        //         _MotVarId_SetMotorConfig(p_context, varId, varValue) : MOT_VAR_STATUS_ERROR_RUNNING;
        // case MOT_VAR_ID_TYPE_MOTOR_SENSOR: return _MotVarId_SetMotorSensor(p_context, varId, varValue);
        // case MOT_VAR_ID_TYPE_COMMAND: return _MotVarId_ExecuteCommand(p_context, varId, varValue);
        default: return MOT_VAR_STATUS_ERROR;
    }
}
