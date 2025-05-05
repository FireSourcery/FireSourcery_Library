
#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
void Motor_ResetUnitsSinCos(Motor_T * p_motor)
{
    if (p_motor->Config.PolePairs != p_motor->SinCos.Config.ElectricalRotationsRatio)
    {
        SinCos_SetAngleRatio(&p_motor->SinCos, p_motor->Config.PolePairs);
    }
}
#endif

#if defined(CONFIG_MOTOR_SENSORS_SIN_COS_ENABLE)
static inline void Motor_Calibrate_StartSinCos(Motor_T * p_motor)
{
    Timer_StartPeriod(&p_motor->ControlTimer, p_motor->Config.AlignTime_ControlCycles);
}

static inline bool Motor_Calibrate_SinCos(Motor_T * p_motor)
{
    bool isComplete = false;

    if (Timer_Periodic_Poll(&p_motor->ControlTimer) == true)
    {
        switch (p_motor->CalibrationStateIndex)
        {
            case 0U:
                Phase_WriteDuty_Fract16(&p_motor->Phase, p_motor->Config.AlignPower_Fract16, 0U, 0U);
                p_motor->CalibrationStateIndex = 2U;
                /* wait 1s */
                break;

                // case 1U:
                //     //can repeat adc and filter results, or skip state use check in sensor routine
                //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
                //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
                //     p_motor->CalibrationStateIndex = 2U;
                //     /* wait 50us, 1s */
                //     break;

            case 2U:
                SinCos_CalibrateA(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, p_motor->Config.AlignPower_Fract16, 0U);
                p_motor->CalibrationStateIndex = 4U;
                /* wait 1s */
                break;

                // case 3U:
                //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_SIN);
                //     AnalogN_EnqueueConversion(p_motor->CONST.P_ANALOG, &p_motor->CONST.ANALOG_CONVERSIONS.CONVERSION_COS);
                //     p_motor->CalibrationStateIndex = 4U;
                //     break;

            case 4U:
                SinCos_CalibrateB(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                //                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, 0U, p_motor->Config.AlignPower_Fract16);
                p_motor->CalibrationStateIndex = 5U;
                // isComplete = true;
                Phase_WriteDuty_Fract16(&p_motor->Phase, p_motor->Config.AlignPower_Fract16, 0U, 0U);
                break;

            case 5U:
                p_motor->SinCos.DebugAPostMech = SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                p_motor->SinCos.DebugAPostElec = SinCos_GetElectricalAngle(&p_motor->SinCos);
                Phase_WriteDuty_Fract16(&p_motor->Phase, 0U, p_motor->Config.AlignPower_Fract16, 0U);
                p_motor->CalibrationStateIndex = 6U;
                break;

            case 6U:
                p_motor->SinCos.DebugBPostMech = SinCos_CaptureAngle(&p_motor->SinCos, p_motor->AnalogResults.Sin_Adcu, p_motor->AnalogResults.Cos_Adcu);
                p_motor->SinCos.DebugBPostElec = SinCos_GetElectricalAngle(&p_motor->SinCos);
                p_motor->CalibrationStateIndex = 0U;
                isComplete = true;
                break;
            default: break;
        }
    }

    return isComplete;
}
#endif