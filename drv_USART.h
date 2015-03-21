/*********************************************************************
 *
 * @file	drv_USART.h
 * @author	Duclos Timothe
 * @date	15/10/2014
 * @brief	Gestion de l'USART
 *
 *********************************************************************/

#ifndef DRV_USART_H_
#define DRV_USART_H_

/********************************************************************
 * Includes
 */
#include "bsp/bsp_carte.h"
#include "util_FIFO.h"

/********************************************************************
 * Exported types
 */
typedef enum {

	UsartOK = 0,

	UsartErr_NonInitialized,
	UsartErr_RxTxDifferentPeriph,
	UsartErr_RxOrTxPinInvalid,
	UsartErr_SendErrorTimeout,
	UsartErr_TryingToInitNonUsartPeriph,
	UsartErr_AlreadyConfigured,

	nb_UsartErrs

}UsartState_e;

typedef enum {

	Periph_USART1 = 0,
	Periph_USART2,
	Periph_USART3,
	Periph_USART6,

	nb_Periph_Usart,

	err_USARTPERIPH_NOTHANDLED,

}Liste_Usart_Periph_e;

typedef enum {

	Usart_Pin_BothRxTx,
	Usart_Pin_Tx,
	Usart_Pin_Rx,

}Liste_Pin_Usart_e;

/********************************************************************
 * Exported Function Prototypes
 */
UsartState_e 	Usart_InitPeriph(Mapping_GPIO_e ID_PinTX, Mapping_GPIO_e ID_PinRX, uint32_t Baudrate);
int 			USART_Write(Mapping_GPIO_e ID_PinTX, uint8_t* pBuffer, uint16_t Taille);
uint8_t 		USART_SetInterruptState(Liste_Usart_Periph_e UsartPeriph, Liste_Pin_Usart_e	Tx_or_Rx, FunctionalState State);
uint16_t 		USART_Read(uint32_t Periph, uint8_t* pBuffer, uint16_t NbRead);


#endif /** DRV_USART_H_ */
