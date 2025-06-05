
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