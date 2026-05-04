// static inline void FeedbackSetpoint_ProcControlUser(Ramp_T * p_Ramp, PID_T * p_pid, uint16_t Feedback) { return PID_ProcPI(p_pid, Feedback, Ramp_ProcNext(p_Ramp)); }
// static inline void FeedbackSetpoint_ProcControlSource(Ramp_T * p_Ramp, PID_T * p_pid, uint16_t Feedback, uint16_t Target) { return PID_ProcPI(p_pid, Feedback, Ramp_ProcNextOnInputOf(p_Ramp, Target)); }
