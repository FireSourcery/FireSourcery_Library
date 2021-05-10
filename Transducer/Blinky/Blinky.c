
#include "Blinky.h"

void Blinky_Toggle(Blinky_T * blinky)
{
	if (blinky->IsOn)
	{
		blinky->IsOn = 0;
		HAL_Pin_WriteOff(blinky->p_HAL_Pin);
	}
	else
	{
		blinky->IsOn = 1;
		HAL_Pin_WriteOn(blinky->p_HAL_Pin);
	}
}

/*!
 * @brief toggle blink switch
 *
 * @param[in] blinky- original blink value
 */
void Blinky_Init(Blinky_T * blinky )
{

}

/*!
 * @brief intialize blink switch
 *
 * @param[in] (*on)- physical switch status
 * @param[in] (*off)- physical switch status
 */
