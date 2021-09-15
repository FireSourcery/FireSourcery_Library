
#include "Blinky.h"

#include "HAL_Pin.h"

/*!
 * @brief toggle blink switch
 *
 * @param[in] blinky- original blink value
 */
void Blinky_Toggle(Blinky_T * p_blinky)
{
	if (p_blinky->IsOn)
	{
		p_blinky->IsOn = false;
		HAL_Pin_WriteState(p_blinky->p_HAL_Pin, true);
	}
	else
	{
		p_blinky->IsOn = true;
		HAL_Pin_WriteState(p_blinky->p_HAL_Pin, false);
	}
}


void Blinky_Pattern1(Blinky_T * p_blinky)
{

}


/*!
 * @brief intialize blink switch
 *

 */
void Blinky_Init
(
	Blinky_T * p_blinky,
	HAL_Pin_T * p_hal_pin
//		uint32_t procFreq
)
{
	p_blinky->p_HAL_Pin = p_hal_pin;

//	p_blinky->ProcFreq = procFreq;


	//use millis timer for 1000Hz and below
//	Thread_InitThreadPeriodic_Period(&p_blinky->ThreadTimer, 	p_consts->P_MILLIS_TIMER, 1000U, 1000U, 	0U, 0U);
}


