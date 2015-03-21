/***************************************************************************************************
 *
 * @author	Duclos Timothe
 * @date	16/01/2015
 * @file	drv_ADC.h
 * @brief	Driver ADC
 *
 ***************************************************************************************************/

#ifndef DRV_ADC_H
#define DRV_ADC_H

/***************************************************************************************************
 * Includes
 */
#include "bsp_carte.h"

/***************************************************************************************************
 * Exported defines
 */

/***************************************************************************************************
 * Exported types
 */
/** @brief Enumeration des status ADC */
typedef enum {

	ADC_OK = 0,

	ADC_err_NotAnADC,
	ADC_err_PeriphAlreadyInited,
	ADC_err_PeriphNotInited,
	ADC_AlreadyRunning,

}ADC_Status_e;

/** @brief Structure d'un ADC */
typedef struct {
	Mapping_GPIO_e	ID_Pin;
	uint32_t* 		StoreValue_mV;
	uint8_t	 		ID_Mesure;
	pFunction		IRQFunction;
}ADC_s;

/***************************************************************************************************
 * Exported variables
 */

/***************************************************************************************************
 * Exported Macros
 */

/***************************************************************************************************
 * Exported Fonction
 */
ADC_Status_e 	ADC_PeriphConfigure	(ADC_s* ADCStruct);
ADC_Status_e 	ADC_StartConv		(ADC_s 	ADCStruct);
void 			ADC_Value_toString	(toString_Possibilities_e Field, uint8_t IDMes, uint8_t* pString);

#endif //DRV_ADC_H
