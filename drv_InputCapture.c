/***************************************************************************************************
 *
 * @file	drv_InputCapture.c
 * @author	Duclos Timothe
 * @date	21/11/2014
 * @brief	Driver InputCapture
 *
 ***************************************************************************************************/

/***************************************************************************************************
 * Includes
 */
#include "drv_InputCapture.h"

/***************************************************************************************************
 * Exported Fonctions Definition
 */
typedef enum {
	IC_OK = 0,
}IC_Status_e;

/**-------------------------------------------------------------------------------------------------
 * @brief	Init input capture
 */
uint8_t
InputCapture_init(
	InputCapture_init_s InputCapture_init	/**<[in] Structure d'init Input capture */
) {

	Mapping_GPIO_e	IdPinIC = InputCapture_init.ID_Pin;
	uint32_t 	Periode_us 	= InputCapture_init.Periode_us;

	uint32_t TIM_Prescaler 	= 1;	/** Prescaler du TimeBase */
	uint32_t TIM_Periode 	= 1;	/** Periode de comptage du TimeBase */

	TIM_ICInitTypeDef 	TIM_ICInitStruct = {
			.TIM_Channel 		= Mapping_GPIO[IdPinIC].Parametre,
			.TIM_ICPolarity 	= TIM_ICPolarity_Rising,
			.TIM_ICSelection	= TIM_ICSelection_DirectTI,
			.TIM_ICPrescaler	= TIM_ICPSC_DIV1,
			.TIM_ICFilter		= 0x0
	};

	__CheckParameters(__IsPeriphTimxOK(Mapping_GPIO[IdPinIC].Periph));

	TIM_Prescaler 	= (((SystemCoreClock / 1000000) / 2) - 1);
	TIM_Periode 	= (Periode_us * 2) - 1;

	TimeBase_init	( (TimeBase_init_s) {	IdPinIC,
											TIM_Periode,
											TIM_Prescaler,
											InputCapture_init.IRQHandle } );

	TIM_ICInit(	(TIM_TypeDef*) 	Mapping_GPIO[IdPinIC].Periph,
								&TIM_ICInitStruct				);

	if( Mapping_GPIO[IdPinIC].Etat_Interruption != Interrupt_OFF ) {

		TimeBase_IT_init(	Mapping_GPIO[IdPinIC].Periph,
							Mapping_GPIO[IdPinIC].Parametre,
							12 );
	}

	TIM_Cmd( (TIM_TypeDef*) Mapping_GPIO[IdPinIC].Periph, ENABLE );

	return IC_OK;
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Lecture de la valeur capturee
 */
uint32_t
InputCapture_GetValue(
	Mapping_GPIO_e IdPinIC
) {
	return 0;
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Desactivation de l'input capture
 */
void
InputCapture_Desactiver(
	Mapping_GPIO_e IdPinIC
) {
	TIM_Cmd( (TIM_TypeDef*) Mapping_GPIO[IdPinIC].Periph, DISABLE );

	if(Mapping_GPIO[IdPinIC].Etat_Interruption)
		TIM_ITConfig(	(TIM_TypeDef* )Mapping_GPIO[IdPinIC].Periph,
						__TimBase_GetITChanel(Mapping_GPIO[IdPinIC].Parametre),
						DISABLE);
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Activation de l'input capture
 */
void
InputCapture_Activer(
	Mapping_GPIO_e IdPinIC
) {
	TIM_Cmd( (TIM_TypeDef*) Mapping_GPIO[IdPinIC].Periph, ENABLE );

	if(Mapping_GPIO[IdPinIC].Etat_Interruption)
		TIM_ITConfig(	(TIM_TypeDef* )Mapping_GPIO[IdPinIC].Periph,
						__TimBase_GetITChanel(Mapping_GPIO[IdPinIC].Parametre),
						ENABLE);
}
