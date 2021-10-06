#include <stdint.h>

//#ifdef CONFIG_ENDIAN_LITTLE
#define ENDIAN_ADDR16_HIGH(var) ( &(((uint8_t *)&var)[1U]) )
#define ENDIAN_ADDR16_LOW(var) 	( &(((uint8_t *)&var)[0U]) )

#define ENDIAN_ADDR32_HIGH_HIGH(var)( &(((uint8_t *)&var)[3U]) )
#define ENDIAN_ADDR32_HIGH_LOW(var) ( &(((uint8_t *)&var)[2U]) )
#define ENDIAN_ADDR32_LOW_HIGH(var) ( &(((uint8_t *)&var)[1U]) )
#define ENDIAN_ADDR32_LOW_LOW(var) 	( &(((uint8_t *)&var)[0U]) )

typedef union
{
	uint16_t UInt16;
	struct
	{
		uint8_t Low;
		uint8_t High;
	};
} Endian_Var16_T;

typedef union
{
	uint32_t UInt32;
	struct
	{
		Endian_Var16_T Low;
		Endian_Var16_T High;
	};
//	struct
//	{
//		uint8_t LowLow;
//		uint8_t LowHigh;
//		uint8_t HighLow;
//		uint8_t HighHigh;
//	};
} Endian_Var32_T;

//#elif CONFIG_ENDIAN_BIG
//#define ENDIAN_ADDR32_HIGH(var) ( &(((uint8_t *)&var)[0U]) )
//#define ENDIAN_ADDR32_LOW(var) 	( &(((uint8_t *)&var)[1U]) )
//
//#define ENDIAN_ADDR32_HIGH_HIGH(var)( &(((uint8_t *)&var)[0U]) )
//#define ENDIAN_ADDR32_HIGH_LOW(var) ( &(((uint8_t *)&var)[1U]) )
//#define ENDIAN_ADDR32_LOW_HIGH(var) ( &(((uint8_t *)&var)[2U]) )
//#define ENDIAN_ADDR32_LOW_LOW(var) 	( &(((uint8_t *)&var)[3U]) )

//typedef union
//{
//	uint32_t UInt32;
//	struct
//	{
//		uint16_t UInt16_High;
//		uint16_t UInt16_Low;
//	};
//	struct
//	{
//		uint8_t UInt8_HighHigh;
//		uint8_t UInt8_HighLow;
//		uint8_t UInt8_LowHigh;
//		uint8_t UInt8_LowLow;
//	};
//} Endian_Var32_T;
//#endif


static inline uint16_t Endian_Swap16(uint16_t val)
{
#if	defined(CONFIG_ENDIAN_SWAP_ARM)
	__REV16(val)
#else
	return (val << 8U) | (val >> 8U);
#endif
}

static inline uint32_t Endian_Swap32(uint32_t val)
{
#if	defined(CONFIG_ENDIAN_SWAP_ARM)
	   return __REV(val);
#else
	uint32_t value;
	value = ((value << 8U) & 0xFF00FF00U) | ((value >> 8U) & 0x00FF00FFU);
	return (value << 16U) | (value >> 16U);
#endif
}

//static inline int16_t Endian_SwapS16(int16_t val)
//{
//	return (val << 8U) | ((val >> 8U) & 0x00FFU);
//}
//
//static inline int32_t Endian_SwapS32(int32_t val)
//{
//	uint32_t value;
//	value = ((value << 8U) & 0xFF00FF00U) | ((value >> 8U) & 0x00FF00FFU);
//	return (value << 16U) | ((value >> 16U) & 0x0000FFFFU);
//}

static inline uint32_t Memory_Read8(const uint8_t * p_address){return *((uint8_t*) p_address);}
static inline uint32_t Memory_Read16(const uint8_t * p_address){return *((uint16_t*) p_address);}
static inline uint32_t Memory_Read32(const uint8_t * p_address){return *((uint32_t*) p_address);}


static inline uint32_t Memory_Read32HighHigh(const uint32_t * p_address){return *ENDIAN_ADDR32_HIGH_HIGH(*p_address);}


