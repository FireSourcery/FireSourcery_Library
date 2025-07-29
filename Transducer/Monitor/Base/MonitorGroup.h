
// #include "Monitor.h"
// #include "Peripheral/Analog/Analog.h"
// #include "Peripheral/Analog/Linear_ADC.h"

// #include <stdint.h>
// #include <stdbool.h>

// typedef const struct  Monitor_Context
// {
//     Monitor_T * P_STATE;

//     /* adc context */
//     Analog_Conversion_T ANALOG_CONVERSION;
//     Linear_T * P_LINEAR; /* Optional for local unit conversion */

//     /* Common across instances in a GroupContext */
//     Linear_T * P_LIMIT_SCALAR;
//     const Monitor_Config_T * P_NVM_CONFIG;
// }
// Monitor_Context_T;

// typedef const struct  MonitorGroup
// {
//     Monitor_T * P_MONITORS;

//     /* adc context */
//     Analog_Conversion_T ANALOG_CONVERSION;
//     Linear_T * P_LINEAR; /* Optional for local unit conversion */

//     /* Common across instances in a GroupContext */
//     Linear_T * P_LIMIT_SCALAR;
//     const Monitor_Config_T * P_NVM_CONFIG;
// }
// MonitorGroup_T;

// static inline uint8_t Monitors_FindGreatest(const Monitor_T * p_group, )
// {
//     uint8_t index = 0U;
//     int32_t max = 0;
//     int32_t compare;

//     for (uint8_t i = 0U; i < p_group->COUNT; i++)
//     {
//         compare = Monitor_GetLastInputComparable(p_group[]);
//         if (compare > max) { max = compare; index = i; }
//     }

//     return index;
// }

// static inline uint8_t _Monitor_Group_PollEach_Index(const Monitor_GroupContext_T * p_group)
// {
//     uint8_t index = 0U;
//     int32_t max = 0; /* Monitor_GetLastInputComparable returns value for > direction compare. sensor values > 0 */
//     int32_t compare;

//     for (uint8_t i = 0U; i < p_group->COUNT; i++)
//     {
//         Monitor_Poll(&p_group->P_CONTEXTS[i]);
//         // todo handle normalizing each capture value, if coeffecients are different
//         compare = Monitor_GetLastInputComparable(p_group->P_CONTEXTS[i].P_STATE);
//         if (compare > max) { max = compare; index = i; }

//         /* optionally mark on same loop */
//     }

//     return index;
// }

// // static inline Monitor_Context_T * _Monitor_Group_PollEach_Context(const Monitor_GroupContext_T * p_group)
// // {
// //     Monitor_Context_T * p_element = NULL;
// //     int32_t max = 0; /* Monitor_GetLastInputComparable returns value for > direction compare. sensor values > 0 */
// //     int32_t compare;

// //     for (uint8_t i = 0U; i < p_group->COUNT; i++)
// //     {
// //         Monitor_Poll(&p_group->P_CONTEXTS[i]);
// //         compare = Monitor_GetLastInputComparable(p_group->P_CONTEXTS[i].P_STATE);
// //         if (compare > max)
// //         {
// //             max = compare;
// //             p_element = &p_group->P_CONTEXTS[i];
// //         }
// //     }

// //     return p_element;
// // }

// /* This function polls all sensors and returns the most severe status */
// // static inline Monitor_Status_T _Monitor_Group_PollEach_Status(const Monitor_GroupContext_T * p_group)
// // {
// //     Monitor_Status_T worstStatus = HEAT_MONITOR_STATUS_NORMAL;
// //     Monitor_Status_T status;
// //     // uint8_t faultCount = 0;
// //     // uint8_t warningCount = 0;

// //     for (uint8_t i = 0U; i < p_group->COUNT; i++)
// //     {
// //         status = Monitor_Poll(&p_group->P_CONTEXTS[i]);
// //         /* Track worst status */
// //         if (status > worstStatus) { worstStatus = status; }

// //         /* Track group statistics */
// //         // if (status == HEAT_MONITOR_STATUS_FAULT_OVERHEAT) { faultCount++; }
// //         // else if (status == HEAT_MONITOR_STATUS_WARNING_HIGH) { warningCount++; }
// //     }

// //     return worstStatus;
// // }

// /* Poll group using round-robin strategy */
// // static inline Monitor_Status_T Monitor_Group_PollRoundRobin(Monitor_GroupContext_T * p_group)
// // {
// //     Monitor_Status_T worstStatus = HEAT_MONITOR_STATUS_NORMAL;

// //     /* Poll next sensor in sequence */
// //     uint8_t sensorIndex = p_group->ActiveSensorIndex;
// //     Monitor_Context_T * p_context = &p_group->P_CONTEXTS[sensorIndex];

// //     Monitor_Status_T status = Monitor_Context_PollSensor(p_context);

// //     /* Update group tracking */
// //     if (status > worstStatus) worstStatus = status;

// //     /* Advance to next sensor */
// //     p_group->ActiveSensorIndex = (sensorIndex + 1) % p_group->COUNT;
// //     p_group->LastProcessedIndex = sensorIndex;
// //     p_group->GroupPollCounter++;

// //     return worstStatus;
// // }
