#ifndef MEM_MAP_BOOT_H
#define MEM_MAP_BOOT_H

// use 2 bits do differentiate unwritten memory patterns
typedef enum
{
	MEM_MAP_BOOT_IS_VALID_01 = 0b01,
	MEM_MAP_BOOT_IS_VALID_10 = 0b10,
	MEM_MAP_BOOT_IS_INVALID_00 = 0b00,
	MEM_MAP_BOOT_IS_INVALID_11 = 0b11,
}
MemMapBoot_IsValid_T;

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
MemMapBoot_T;

#define MEM_MAP_BOOT ((MemMap_Boot_T *)CONFIG_MEM_MAP_BOOT_ADDRESS)

#endif
