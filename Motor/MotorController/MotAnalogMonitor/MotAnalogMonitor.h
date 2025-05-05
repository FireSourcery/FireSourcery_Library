

// // /*
// //     Activate ADC outside module
// // */
// // typedef const struct MotAnalogMonitor_Const
// // {
// //     const MotAnalogMonitor_Config_T * const P_CONFIG;
// // }
// // MotAnalogMonitor_Const_T;

// const Analog_Conversion_T CONVERSION_VSOURCE;
// const Analog_Conversion_T CONVERSION_VSENSE;
// const Analog_Conversion_T CONVERSION_VACCS;
// const Analog_Conversion_T CONVERSION_HEAT_PCB;
// /* COUNT is defined by macro. It is also needed to determine global channel index  */
// const Analog_Conversion_T HEAT_MOSFETS_CONVERSIONS[MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT];

// typedef struct MotAnalogMonitor
// {
//     MotAnalogMonitor_Const_T CONST;
//     MotAnalogMonitor_Config_T Config;
//     VMonitor_T VMonitorSource;  /* Controller Supply */
//     VMonitor_T VMonitorSense;   /* ~5V */
//     VMonitor_T VMonitorAccs;    /* ~12V */
// }
// MotAnalogMonitor_T;

// static inline MotAnalogMonitor_Status_T _Mot_ProcHeatMonitor(MotorController_T * p_mc)
// {
//     bool isFault = false;
//     bool isWarning = false;

//     Thermistor_PollMonitor(&p_mc->ThermistorPcb, MotorController_Analog_GetHeatPcb(p_mc));
//     if (Thermistor_IsFault(&p_mc->ThermistorPcb) == true) { p_mc->FaultFlags.PcbOverheat = 1U; isFault = true; }

//     for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
//     {
//         Thermistor_PollMonitor(&p_mc->MosfetsThermistors[iMosfets], MotorController_Analog_GetHeatMosfets(p_mc, iMosfets));
//         if (Thermistor_IsFault(&p_mc->MosfetsThermistors[iMosfets]) == true) { p_mc->FaultFlags.MosfetsOverheat = 1U; isFault = true; }
//     }

//     if (isFault == true)
//     {
//         MotorController_StateMachine_EnterFault(p_mc); /* Shutdown repeat set ok */
//     }
//     else
//     {
//         /* Warning behaviors edge triggered */
//         isWarning |= Thermistor_IsWarning(&p_mc->ThermistorPcb);
//         for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
//         {
//             isWarning |= Thermistor_IsWarning(&p_mc->MosfetsThermistors[iMosfets]);
//         }

//         if (isWarning == true)
//         {
//             /*
//                 Thermistor Adcu is roughly linear in Warning region
//                 Assume Heat Mosfets as highest heat
//                 constantly compares lowest active limit, alternatively, check and restore prev on clear limit
//                 Increasing Limit only, reset on warning clear.
//             */
//             MotorController_SetILimitAll_Scalar(p_mc, MOT_I_LIMIT_HEAT_MC, Thermistor_GetHeatLimit_Percent16(&p_mc->MosfetsThermistors[0U]) / 2);

//             // Thermistor_PollWarningRisingEdge(&p_mc->ThermistorMosfets ); // use highest or
//             if (p_mc->StateFlags.HeatWarning == false)
//             {
//                 // Blinky_BlinkN(&p_mc->Buzzer, 250U, 250U, 1U); //todo disable during init, alternatively disable monitors on init
//                 p_mc->StateFlags.HeatWarning = true;
//             }
//         }
//         else
//         {
//             // Thermistor_PollWarningFallingEdge(&p_mc->ThermistorMosfets );
//             if (p_mc->StateFlags.HeatWarning == true)
//             {
//                 MotorController_ClearILimitAll(p_mc, MOT_I_LIMIT_HEAT_MC); //clear mosfet
//                 p_mc->StateFlags.HeatWarning = false;
//             }
//         }
//     }

//     Analog_MarkConversion(&p_mc->CONST.CONVERSION_HEAT_PCB);
//     for (uint8_t iMosfets = 0U; iMosfets < MOTOR_CONTROLLER_HEAT_MOSFETS_COUNT; iMosfets++)
//     {
//         Analog_MarkConversion(&p_mc->CONST.HEAT_MOSFETS_CONVERSIONS[iMosfets]);
//     }

//     for (uint8_t iMotor = 0U; iMotor < p_mc->CONST.MOTORS.LENGTH; iMotor++) { Motor_Heat_Thread(&p_mc->CONST.MOTORS.P_ARRAY[iMotor]); }
// }