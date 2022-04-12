#include "SinCos.h"

//#include <stdbool.h>
//#include <stdint.h>
#include <string.h>

void SinCos_Init(SinCos_T * p_sincos)
{
	if (p_sincos->CONFIG.P_PARAMS != 0U)
	{
		memcpy(&p_sincos->Params, p_sincos->CONFIG.P_PARAMS, sizeof(SinCos_Params_T));
		Linear_ADC_Init(&p_sincos->UnitSin, p_sincos->Params.SinZero_ADCU, p_sincos->Params.SinMax_ADCU, p_sincos->Params.SinMax_MilliV);
		Linear_ADC_Init(&p_sincos->UnitCos, p_sincos->Params.CosZero_ADCU, p_sincos->Params.CosMax_ADCU, p_sincos->Params.CosMax_MilliV);
	}
	else
	{
		Linear_ADC_Init(&p_sincos->UnitSin, 2048U, 4095U, 4500U);
		Linear_ADC_Init(&p_sincos->UnitCos, 2048U, 4095U, 4500U);
	}
}

//void SinCos_SetParams(SinCos_T * p_sincos, const SinCos_Params_T * p_param)
//{
//	memcpy(&p_sincos->Params, p_param, sizeof(SinCos_Params_T));
//}

void SinCos_CalibrateSin(SinCos_T * p_sincos, uint16_t zero_ADCU, uint16_t max_ADCU, uint16_t milliV_ADCU)
{
	p_sincos->Params.SinZero_ADCU = zero_ADCU;
	p_sincos->Params.SinMax_ADCU = max_ADCU;
	p_sincos->Params.SinMax_MilliV = milliV_ADCU;
	Linear_ADC_Init(&p_sincos->UnitSin, p_sincos->Params.SinZero_ADCU, p_sincos->Params.SinMax_ADCU, p_sincos->Params.SinMax_MilliV);
}

void SinCos_CalibrateCos(SinCos_T * p_sincos, uint16_t zero_ADCU, uint16_t max_ADCU, uint16_t milliV_ADCU)
{
	p_sincos->Params.CosZero_ADCU = zero_ADCU;
	p_sincos->Params.CosMax_ADCU = max_ADCU;
	p_sincos->Params.CosMax_MilliV = milliV_ADCU;
	Linear_ADC_Init(&p_sincos->UnitCos, p_sincos->Params.CosZero_ADCU, p_sincos->Params.CosMax_ADCU, p_sincos->Params.CosMax_MilliV);
}




