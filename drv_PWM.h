/***************************************************************************************************
 *
 * @file	drv_PWM.h
 * @author	Duclos Timothe
 * @date	16/09/2014
 * @brief	Driver générant un signal PWM sur une broche pre-configuree
 *
 ***************************************************************************************************/

#ifndef DRV_PWM_H
#define DRV_PWM_H

/***************************************************************************************************
 * Includes
 */
#include "drv_TimBase.h"
#include "util_Console.h"
#include "util_AffDebug.h"

/***************************************************************************************************
 * Exported types
 */
typedef enum {

	Err_PeriodAskedForIsUnreachable = 0,
	Err_PWM_Unavailable,

}PWM_Err_Status_e;

typedef struct {
	Mapping_GPIO_e	ID_Pin;
	uint32_t		Periode_us;
	uint32_t		Ratio_pr1000;
	pFunctionVide	IRQHandle;
}PWM_init_s;
/***************************************************************************************************
 * Exported Function
 */
uint8_t PWM_Init	(PWM_init_s PWM_init);
void 	PWM_Desinit	(Mapping_GPIO_e	IdPinPwm);
uint8_t PWM_Activer	(Mapping_GPIO_e IdPinPwm);
uint8_t PWM_Desactiver	(Mapping_GPIO_e IdPinPwm);
void	PWM_GetConfiguration(Mapping_GPIO_e IdPinPwm, uint32_t* Periode_us, uint16_t*	Ratio_pr100);
void 	PWM_Value_toString(toString_Possibilities_e Field, Mapping_GPIO_e IDMapping, uint8_t* pString);


#endif /** DRV_PWM_H */
