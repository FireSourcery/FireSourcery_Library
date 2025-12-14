
/* Buzzer Config  options enabled for use */
typedef union MotBuzzer_OptionFlags
{
    struct
    {
        // uint16_t IsEnabled              : 1U; /* Primary Enable */
        // uint16_t OnInit              : 1U;
        // uint16_t OnDirectionChange   : 1U;
        // uint16_t OnReverse           : 2U; /* 0: Off, 1: Short Beep, 2: Continuous */
        // uint16_t OnInitThrottle      : 1U;
        // uint16_t ThrottleOnBrakeCmd;
        // uint16_t ThrottleOnBrakeRelease;
        // uint16_t ThrottleOnNeutralRelease;
    };
    uint16_t Value;
}
MotBuzzer_OptionFlags_T;

/******************************************************************************/
/*
    move to buzzer
*/
/******************************************************************************/
// static inline void MotorController_BeepN(const MotorController_T * p_context, uint32_t onTime, uint32_t offTime, uint8_t n) { Blinky_BlinkN(&p_context->BUZZER, onTime, offTime, n); }
// static inline void MotorController_BeepStart(const MotorController_T * p_context, uint32_t onTime, uint32_t offTime) { Blinky_StartPeriodic(&p_context->BUZZER, onTime, offTime); }
// static inline void MotorController_BeepStop(const MotorController_T * p_context) { Blinky_Stop(&p_context->BUZZER); }
// static inline void MotorController_DisableBuzzer(const MotorController_T * p_context) { Blinky_Disable(&p_context->BUZZER); }
