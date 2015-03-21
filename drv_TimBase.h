/***************************************************************************************************
 *
 * @file	drv_TimBase.h
 * @author	Duclos Timothe
 * @date	21/11/2014
 * @brief	Driver gestion TimBase
 *
 *********************************************************************/

#ifndef DRV_TIMBASE_H
#define DRV_TIMBASE_H

/***************************************************************************************************
 * Includes
 */
#include "BSP_carte.h"

/***************************************************************************************************
 * Exported macros
 */
#define __IsPeriphTimxOK(pERIPH)	\
		(pERIPH) == (uint32_t) TIM1  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM2  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM3  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM4  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM5  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM6  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM7  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM8  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM9  ?  TRUE : \
		(pERIPH) == (uint32_t) TIM10 ?  TRUE : \
		(pERIPH) == (uint32_t) TIM11 ?  TRUE : \
		(pERIPH) == (uint32_t) TIM12 ?  TRUE : \
		(pERIPH) == (uint32_t) TIM13 ?  TRUE : \
		(pERIPH) == (uint32_t) TIM14 ?  TRUE : FALSE

#define __TimBase_GetITChanel(cHAN)	\
		cHAN == (uint32_t) TIM_Channel_1 ? TIM_IT_CC1 : \
		cHAN == (uint32_t) TIM_Channel_2 ? TIM_IT_CC2 : \
		cHAN == (uint32_t) TIM_Channel_3 ? TIM_IT_CC3 : TIM_IT_CC4

#define __TimeBase_GetPeriphIRQn(pERIPH) \
		pERIPH == (uint32_t) TIM2  ? TIM2_IRQn  : \
		pERIPH == (uint32_t) TIM3  ? TIM3_IRQn  : \
		pERIPH == (uint32_t) TIM4  ? TIM4_IRQn  : \
		pERIPH == (uint32_t) TIM5  ? TIM5_IRQn  : \
		pERIPH == (uint32_t) TIM7  ? TIM7_IRQn  : \
		pERIPH == (uint32_t) TIM8  ? TIM8_CC_IRQn : 0

#define __TimeBase_GetTimPeriphEnum(pERIPH) \
		pERIPH == (uint32_t) TIM1  ? Periph_Tim1  : \
		pERIPH == (uint32_t) TIM2  ? Periph_Tim2  : \
		pERIPH == (uint32_t) TIM3  ? Periph_Tim3  : \
		pERIPH == (uint32_t) TIM4  ? Periph_Tim4  : \
		pERIPH == (uint32_t) TIM5  ? Periph_Tim5  : \
		pERIPH == (uint32_t) TIM6  ? Periph_Tim6  : \
		pERIPH == (uint32_t) TIM7  ? Periph_Tim7  : \
		pERIPH == (uint32_t) TIM8  ? Periph_Tim8  : \
		pERIPH == (uint32_t) TIM9  ? Periph_Tim9  : \
		pERIPH == (uint32_t) TIM10 ? Periph_Tim10 : \
		pERIPH == (uint32_t) TIM11 ? Periph_Tim11 : \
		pERIPH == (uint32_t) TIM12 ? Periph_Tim12 : \
		pERIPH == (uint32_t) TIM13 ? Periph_Tim13 : Periph_Tim14

#define __TimeBase_GetTimChannelEnum(cHANNEL) \
		cHANNEL == (uint32_t) TIM_Channel_1 ? Periph_TimChannel1 : \
		cHANNEL == (uint32_t) TIM_Channel_2 ? Periph_TimChannel2 : \
		cHANNEL == (uint32_t) TIM_Channel_3 ? Periph_TimChannel3 : Periph_TimChannel4

/***************************************************************************************************
 * Exported types
 */
typedef enum {

	Periph_Tim1 = 0,
	Periph_Tim2,
	Periph_Tim3,
	Periph_Tim4,
	Periph_Tim5,
	Periph_Tim6,
	Periph_Tim7,
	Periph_Tim8,
	Periph_Tim9,
	Periph_Tim10,
	Periph_Tim11,
	Periph_Tim12,
	Periph_Tim13,
	Periph_Tim14,

	nb_TimPeriph,

	err_NotATimPeriph

}ListePeriphTim_e;

typedef enum {

	Periph_TimChannel1 = 0,
	Periph_TimChannel2,
	Periph_TimChannel3,
	Periph_TimChannel4,

	nb_TimChannels,

	err_NotATimChannel

}ListeTimChannels_e;

typedef enum {
	PeriphTim_Unused = 0,
	PeriphTim_PWM,
	PeriphTim_IC
}Etat_PeriphTim_e;

typedef struct {
	Mapping_GPIO_e 	IDPin;
	uint32_t 		TIM_Periode;
	uint32_t 		TIM_Prescaler;
	pFunctionVide 	IRQHandle;
}TimeBase_init_s;

/***************************************************************************************************
 * Exported Function
 */
void TimeBase_init		(TimeBase_init_s TimBase_init);
void TimeBase_IT_init	(uint32_t Periph, uint32_t Channel,	uint8_t  Priority);


#endif /** DRV_TIMBASE_H */
