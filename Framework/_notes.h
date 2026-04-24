    typedef const struct RingT
    {
        Ring_Type_T TYPE;
        Ring_T * P_STATE; // Ring_State_T * P_STATE;
    }
    RingT_T;

    _RingT_Head(Ring_Type_T type, const Ring_State_T * p_state);
    RingT_Head(RingT_T * p_outer);


    typedef const struct TimerT
    {
        Timer_Base_T * P_BASE;
        Timer_State_T * P_STATE;
    }
    TimerT_T;

    _TimerT_Elapsed(const Timer_Base_T * p_base, const Timer_State_T * p_state);
    TimerT_Elapsed(TimerT_T outer);


Value - Semantics Gradient
A type's storage and passing form should follow its weight. Small descriptors travel by value; heavy descriptors travel by reference; instances always travel by reference.
Wrapper passing form depends on wrapper weight. Two-pointer wrappers may be passed by value; larger wrappers by pointer.

Storage form follows descriptor weight, not module convention.
- A descriptor that is small and bounded embeds by value; its wrapper is passed by pointer.
- A descriptor that is heavy or growing is referenced by pointer; its wrapper, being just two pointers, is passed by value.
- The bare primitive always takes the descriptor and state as separate parameters — the wrapper is a convenience for call-site ergonomics, not a structural requirement.
- The wrapper's pass-by-X form is determined by its total size against the ABI register threshold (16 bytes on ARM AAPCS), not by what feels symmetric with peer modules.

