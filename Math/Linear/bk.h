// /*
// 	Right shift must retain sign bit,
// 	user factor, divisor, input must be less than 16 bits
// */
// static inline int32_t MaxLeftShiftDivide_Signed(int32_t factor, int32_t divisor, uint8_t leftShift)
// {
// 	int32_t result = 0;
// 	int32_t shiftedValue = (1U << leftShift);

// 	if (shiftedValue % divisor == 0U) /* when divisor is a whole number factor of shifted. */
// 	{
// 		result = factor * (shiftedValue / divisor);
// 	}
// 	else
// 	{
// 		for (uint8_t maxShift = leftShift; maxShift > 0U; maxShift--)
// 		{
// 			if(factor > 0)
// 			{
// 				if (factor <= (INT32_MAX >> maxShift))
// 				{
// 					result = (factor << maxShift) / divisor; //max shift before divide

// 					if (result <= (INT32_MAX >> (leftShift - maxShift)))
// 					{
// 						result = result << (leftShift - maxShift); //remaining shift
// 					}
// 					else /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
// 					{
// 						result = 0;
// 					}
// 					break;
// 				}
// 			}
// 			else
// 			{
// 				if((factor << maxShift) < 0) //still negative after shifting
// 				{
// 					result = (factor << maxShift) / divisor;

// 					if ((result << (leftShift - maxShift)) < 0)
// 					{
// 						result = result << (leftShift - maxShift);
// 					}
// 					else  /* error, will overflow 32 bit even using ((factor << 0)/divisor) << leftShift */
// 					{
// 						result = 0;
// 					}
// 					break;
// 				}
// 			}
// 		}
// 	}

// 	return result;
// }

// /*
// 	Max left shit, while retaining
// */
// static inline uint8_t CalcMaxLeftShift_Signed(int32_t slope, int32_t xMaxRef)
// {
// 	uint8_t shift;

// 	//todo log2

// 	for(uint8_t iShift = 0U; iShift < 32U; iShift++)
// 	{
// 		if((slope << shift) > INT32_MAX / xMaxRef)
// 		{
// 			shift = iShift - 1U;
// 			break;
// 		}
// 	}

// 	return shift;
// }
