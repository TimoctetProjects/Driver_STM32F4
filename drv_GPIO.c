/*********************************************************************
 * *******************************************************************
 *
 * @file	drv_GPIO.c
 *
 * @author	Duclos Timothe
 *
 * @date	10/10/2014
 *
 * @brief	Driver GPIO
 *
 *********************************************************************/


/********************************************************************
 * Includes
 */
#include "drv_GPIO.h"

/********************************************************************
 * Private defines
 */
#define ETAT_ACTIF_STRING		"Etat Actif"
#define ETAT_INACTIF_STRING		"Etat Inactif"

/********************************************************************
 * Private Macros
 */
#define	__SetEtatActifString(str) \
	__strncat (str, ETAT_ACTIF_STRING,  __strlen(ETAT_ACTIF_STRING))

#define	__SetEtatInactifString(str)	\
	__strncat (str, ETAT_INACTIF_STRING, __strlen(ETAT_INACTIF_STRING))

/********************************************************************
 * Private variables
 */

/********************************************************************
 * Private Fonctions
 */



/********************************************************************
 * Exported Fonctions
 */
/**------------------------------------------------------------------
 * @brief	Lecture de l'etat d'une GPIO
 */
Etat_e
GPIO_Get(
		Mapping_GPIO_e Pin	/**<[in] ID de la pin GPIO */
) {
	//---------------------------------------------------------------
	//---------------- Declaration des variables
	uint8_t Etat_GPIO;
	Etat_e 	rtrn;

	GPIO_TypeDef* 	Peripheral;
	uint16_t 		GPIO_Pin;

	//---------------- Lecture des valeurs de config
	Peripheral	= Mapping_GPIO[Pin].GpioPeripheral;
	GPIO_Pin 	= Mapping_GPIO[Pin].GpioPin;

	//---------------------------------------------------------------
	//---------------- Getter l'etat
	switch(Mapping_GPIO[Pin].GpioMode) {

		case GPIO_Mode_IN:	Etat_GPIO = GPIO_ReadInputDataBit(Peripheral, GPIO_Pin);
							break;

		case GPIO_Mode_OUT:	Etat_GPIO = GPIO_ReadOutputDataBit(Peripheral, GPIO_Pin);
							break;

		default:			break;
	}

	switch(Mapping_GPIO[Pin].Inverse) {

		case TRUE:
					switch(Etat_GPIO) {
						case Bit_SET:	rtrn = ETAT_INACTIF;	break;
						case Bit_RESET: rtrn = ETAT_ACTIF;		break;
						default:								break;
					}	break;

		case FALSE:
					switch(Etat_GPIO) {
						case Bit_SET:	rtrn = ETAT_ACTIF;		break;
						case Bit_RESET: rtrn = ETAT_INACTIF;	break;
						default:								break;
					}	break;
	}

	return rtrn;
}

/**------------------------------------------------------------------
 * @brief	Ecriture de l'etat d'une GPIO
 */
void
GPIO_Set(
		Mapping_GPIO_e Pin,	/**<[in] ID de la pin GPIO */
		Etat_e State		/**<[in] Etat de la pin */
) {
	//---------------------------------------------------------------
	//---------------- Declaration des variables
	GPIO_TypeDef* Peripheral;
	uint16_t GPIO_Pin;

	//---------------- Lecture des valeurs de config
	Peripheral	= Mapping_GPIO[Pin].GpioPeripheral;
	GPIO_Pin 	= Mapping_GPIO[Pin].GpioPin;

	//---------------------------------------------------------------
	//---------------- Setter l'etat
	switch(Mapping_GPIO[Pin].Inverse) {

		//--------------------------------------------------------
		case TRUE:
			switch(State) {

				case ETAT_ACTIF:
					if(GPIO_Get(Pin) != ETAT_ACTIF) {
						GPIO_ResetBits(Peripheral, GPIO_Pin);
					} break;

				case ETAT_INACTIF:
					if(GPIO_Get(Pin) != ETAT_INACTIF) {
						GPIO_SetBits(Peripheral, GPIO_Pin);
					} break;

				default:
					break;

			} break;


		//--------------------------------------------------------
		case FALSE:

			switch(State) {

				case ETAT_ACTIF:
					if(GPIO_Get(Pin) != ETAT_ACTIF) {
						GPIO_SetBits(Peripheral, GPIO_Pin);
					} break;

				case ETAT_INACTIF:
					if(GPIO_Get(Pin) != ETAT_INACTIF) {
						GPIO_ResetBits(Peripheral, GPIO_Pin);
					} break;

				default:
					break;

			} break;
	}

}


#ifdef UTIL_CONSOLE_H
/**--------------------------------------------------------------------
 *
 * @brief	Fonction permettant de convertir en string l'etat d'une GPIO
 *
 */
void
GPIO_Value_toString(
		toString_Possibilities_e	Field,
		Mapping_GPIO_e 				IDMapping,
		uint8_t*					pString
) {

	Etat_e	EtatGpio = 0;

	switch(Field) {

		//----------------------------------------------------------
		case toString_Getpin:	toString_GetPeriphral	(Mapping_GPIO[IDMapping].GpioPeripheral,pString);
					toString_GetPin		(Mapping_GPIO[IDMapping].GpioPin, 	pString);
					break;

		//----------------------------------------------------------
		case toString_GetValue:	EtatGpio = GPIO_Get(IDMapping);
					switch(EtatGpio) {
						case ETAT_ACTIF:	__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_verte);
									__SetEtatActifString(pString);
									__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_blanc);
									break;

						case ETAT_INACTIF:	__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_rouge);
									__SetEtatInactifString(pString);
									__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_blanc);
									break;
						default:							break;
					} break;

		//----------------------------------------------------------
		default:
					break;
	}
}
#endif
