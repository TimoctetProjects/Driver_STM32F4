/***************************************************************************************************
 *
 * @file	drv_InputCapture.h
 * @author	Duclos Timothe
 * @date	21/11/2014
 * @brief	Driver InputCapture
 *
 ***************************************************************************************************/

#ifndef DRV_INPUTCAPTURE_H
#define DRV_INPUTCAPTURE_H

/***************************************************************************************************
 * Includes
 */
#include "drv_TimBase.h"

/***************************************************************************************************
 * Exported types
 */
typedef struct {
	Mapping_GPIO_e	ID_Pin;
	uint32_t		Periode_us;
	pFunctionVide	IRQHandle;
}InputCapture_init_s;

/***************************************************************************************************
 * Exported function
 */
uint8_t 	InputCapture_init		(InputCapture_init_s InputCapture_init);
uint32_t 	InputCapture_GetValue	(Mapping_GPIO_e IdPinIC);
void 		InputCapture_Activer	(Mapping_GPIO_e IdPinIC);
void 		InputCapture_Desactiver	(Mapping_GPIO_e IdPinIC);


#endif /** DRV_INPUTCAPTURE_H */
