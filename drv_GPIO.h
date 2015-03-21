/*********************************************************************
 * *******************************************************************
 *
 * @file	drv_GPIO.h
 *
 * @author	Duclos Timothe
 *
 * @date	10/10/2014
 *
 * @brief	Driver GPIO
 *
 *********************************************************************/

#ifndef DRV_GPIO_H
#define DRV_GPIO_H

/********************************************************************
 * Includes
 */
#include "BSP/bsp_carte.h"
#include "util_Console.h"

/********************************************************************
 * Exported functions
 */
void 	GPIO_Set(Mapping_GPIO_e Pin, Etat_e State);
Etat_e 	GPIO_Get(Mapping_GPIO_e Pin);

void 	GPIO_Value_toString(toString_Possibilities_e Field, Mapping_GPIO_e IDMapping, uint8_t* pString);

#endif // DRV_GPIO_H
