
#define PARAM_STR(PARAM_STR)  #PARAM_STR

#define PRINT_PARAM_STR(PARAM_STR)  				\
	Terminal_SendString(p_terminal, #PARAM_STR); 	\
	Terminal_SendString(p_terminal, ":	");


#define PRINT_PARAM_VAR_MOTOR(PARAM_STR) \
		PRINT_PARAM_STR(PARAM_STR) \
		Terminal_SendNum(p_terminal, p_motor->P##arameters.PARAM_STR); 	\
		Terminal_SendString(p_terminal, "\r\n");

#define PRINT_PARAM_VAR_MC(PARAM_STR) \
		PRINT_PARAM_STR(PARAM_STR) \
		Terminal_SendNum(p_terminal, p_mc->P##arameters.PARAM_STR); 	\
		Terminal_SendString(p_terminal, "\r\n");

static Cmd_Status_T Cmd_params(MotorController_T * p_mc, int argc, char ** argv)
{
	Terminal_T * p_terminal = &p_mc->Shell.Terminal;
	Motor_T * p_motor = MotorController_User_GetPtrMotor(p_mc, 0U);

	if(argc == 1U)
	{


		//		Terminal_SendString(p_terminal, "SpeedFeedbackRef_Rpm: ");
		//		Terminal_SendNum(p_terminal, p_motor->Parameters.SpeedFeedbackRef_Rpm);
		//		Terminal_SendString(p_terminal, "\r\n");

		PRINT_PARAM_VAR_MOTOR(PolePairs)
			PRINT_PARAM_VAR_MOTOR(SpeedFeedbackRef_Rpm)
			// PRINT_PARAM_VAR_MOTOR(SpeedVMatchRef_Rpm)
			// PRINT_PARAM_VAR_MOTOR(IRefMax_Amp)
			// PRINT_PARAM_VAR_MOTOR(IaRefMax_Adcu)
			// PRINT_PARAM_VAR_MOTOR(IbRefMax_Adcu)
			// PRINT_PARAM_VAR_MOTOR(IcRefMax_Adcu)
			PRINT_PARAM_VAR_MOTOR(IaRefZero_Adcu)
			PRINT_PARAM_VAR_MOTOR(IbRefZero_Adcu)
			PRINT_PARAM_VAR_MOTOR(IcRefZero_Adcu)
			PRINT_PARAM_VAR_MOTOR(AlignVoltage_Frac16)
			PRINT_PARAM_VAR_MOTOR(AlignTime_ControlCycles)
			//
			//		PRINT_PARAM(p_motor->Encoder.Params, CountsPerRevolution)
			//		PRINT_PARAM(p_motor->Encoder.Params, DistancePerCount)
			//		PRINT_PARAM(p_motor->Encoder.Params, IsQuadratureCaptureEnabled)
			//		PRINT_PARAM(p_motor->Encoder.Params, IsALeadBPositive)
			//		PRINT_PARAM(p_motor->Encoder.Params, ExtendedTimerDeltaTStop)
			//		PRINT_PARAM(p_motor->Encoder.Params, MotorPolePairs)
			//		PRINT_PARAM(p_mc, Params.MotorPolePairs)
			Terminal_SendString(p_terminal, "\r\n");
	}

	return CMD_STATUS_SUCCESS;
}