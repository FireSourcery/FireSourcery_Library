#ifndef MOT_MEM_MAP_BOOT_H
#define MOT_MEM_MAP_BOOT_H

// use 2 bits do differentiate unwritten memory patterns
typedef enum
{
	MOT_MEM_BOOT_IS_VALID = 0b01,
//	BOOT_IS_VALID = 0b10,
}
MotMemMap_Boot_IsValid_T;

typedef union
{
	struct
	{
		volatile uint32_t IsValid	: 2U;
		volatile uint32_t FastBoot 	: 1U;
		volatile uint32_t Beep 		: 1U;
		volatile uint32_t Blink  	: 1U;
	};
	volatile uint32_t Register;
}
MotMemMap_Boot_T;

#define MOT_MEM_BOOT	(( MotMemMap_Boot_T *)CONFIG_MOT_MEM_BOOT_ADDRESS)


#endif
