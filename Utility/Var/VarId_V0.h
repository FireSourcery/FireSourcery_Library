// typedef union VarId
// {
//     struct
//     {
//         uint16_t NameBase       : 4U; /* Name - corresponds 1:1 with enum value */
//         uint16_t NameType       : 4U; /* Name's Type - corresponds 1:1 with enum type */
//         uint16_t NamePartition  : 2U; /* Type's Type/Prefix/Partition */
//         uint16_t Instance       : 3U; /* TypeInstance1 - Upto 8 Instances Per Type */
//         uint16_t Alt            : 3U; /* Alternative unit/format */
//     };
//     /* Correspond to host side */
//     struct
//     {
//         uint16_t NamePart       : 10U; /* name can be determined by NameBase + nameId_Type if prefix maps to nameId_Type 1:1 */
//         uint16_t InstancePart   : 3U;
//         uint16_t ResvPart       : 3U;
//     };
//     uint16_t Value;
// }
// VarId_T;
