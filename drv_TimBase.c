/***************************************************************************************************
 *
 * @author	Duclos Timothe
 * @date	16/09/2014
 * @file	drv_PWM.c
 * @brief	Driver gerant les different TimBase du systeme
 *
 ***************************************************************************************************/

/***************************************************************************************************
 * Includes
 */
#include "drv_TimBase.h"

/***************************************************************************************************
 * Private Types
 */
typedef struct {
	uint32_t	Periode;
	Bool_e		IsConfigured;
}Timbase_s;

/***************************************************************************************************
 * Private Variables
 */
Timbase_s Etat_TimBase[nb_TimPeriph];

/***************************************************************************************************
 * Private Function prototypes
 */
static inline void 		TimeBase_RccInit(uint32_t Periph);
static pFunctionVide IRQFunctions[nb_TimPeriph][nb_TimChannels];

/***************************************************************************************************
 * Exported Fonctions Definition
 */
/**-------------------------------------------------------------------------------------------------
 * @brief	Initialisation du TimeBase
 */
void
TimeBase_init(
		TimeBase_init_s TimBase_init	/**<[in] Structure d'init TimeBase */
) {

	uint32_t	Periph 			= Mapping_GPIO[TimBase_init.IDPin].Periph;
	uint32_t 	TIM_Periode 	= TimBase_init.TIM_Periode;
	uint32_t 	TIM_Prescaler	= TimBase_init.TIM_Prescaler;

	IRQFunctions[__TimeBase_GetTimPeriphEnum(Periph)]
	            [__TimeBase_GetTimChannelEnum(Mapping_GPIO[TimBase_init.IDPin].Parametre)]
	             = TimBase_init.IRQHandle;

	//----------------------------------------------------------------------------
	//------------------------ Déclaration et Initialisation ---------------------
	//----------------------------------------------------------------------------
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	//--------- Initialisation des structures aux valeurs par défaut
	TIM_TimeBaseStructInit	(&TIM_TimeBaseInitStruct);

	//--------- Initialisation de l'horloge
	TimeBase_RccInit(Periph);

	//---------- Remplissage de la structure d'init TimBase
	TIM_TimeBaseInitStruct.TIM_Prescaler		= TIM_Prescaler;
	TIM_TimeBaseInitStruct.TIM_Period			= TIM_Periode;
	TIM_TimeBaseInitStruct.TIM_CounterMode		= TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision	= TIM_CKD_DIV1;

	//--------- Initialisation de la base de temps
	TIM_TimeBaseInit( (TIM_TypeDef*) Periph, &TIM_TimeBaseInitStruct );
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Initialisation d'interruption
 */
void
TimeBase_IT_init(
		uint32_t Periph,
		uint32_t Channel,
		uint8_t  Priority
) {
	NVIC_InitTypeDef 	NVIC_InitStructure 	= {
		.NVIC_IRQChannelPreemptionPriority 	= Priority,
		.NVIC_IRQChannelSubPriority 		= 0x01,
		.NVIC_IRQChannelCmd 				= ENABLE
	};

	NVIC_InitStructure.NVIC_IRQChannel = __TimeBase_GetPeriphIRQn(Periph);

	NVIC_Init( &NVIC_InitStructure );
	TIM_ITConfig((TIM_TypeDef* )Periph, __TimBase_GetITChanel(Channel), ENABLE);
}

/***************************************************************************************************
 * Interruptions Tmer
 */
void
TIM5_IRQHandler(
	void
) {
	if(TIM_GetITStatus(TIM5, TIM_IT_CC3) == SET) {

		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
		IRQFunctions[Periph_Tim5][Periph_TimChannel3]();
	}
}

void
TIM2_IRQHandler(
	void
) {
	 if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		IRQFunctions[Periph_Tim2][Periph_TimChannel1]();
	}

	else if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		IRQFunctions[Periph_Tim2][Periph_TimChannel2]();
	}

	else if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		IRQFunctions[Periph_Tim2][Periph_TimChannel3]();
	}

	else if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		IRQFunctions[Periph_Tim2][Periph_TimChannel4]();
	}
}

void
TIM3_IRQHandler(
	void
) {
	 if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET) {

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		IRQFunctions[Periph_Tim3][Periph_TimChannel1]();
	}

	else if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) {

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		IRQFunctions[Periph_Tim3][Periph_TimChannel2]();
	}

	else if(TIM_GetITStatus(TIM3, TIM_IT_CC3) == SET) {

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		IRQFunctions[Periph_Tim3][Periph_TimChannel3]();
	}

	else if(TIM_GetITStatus(TIM3, TIM_IT_CC4) == SET) {

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		IRQFunctions[Periph_Tim3][Periph_TimChannel4]();
	}
}

void
TIM4_IRQHandler(
	void
) {
	 if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET) {

		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		IRQFunctions[Periph_Tim4][Periph_TimChannel1]();
	}

	else if(TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET) {

		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		IRQFunctions[Periph_Tim4][Periph_TimChannel2]();
	}

	else if(TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET) {

		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		IRQFunctions[Periph_Tim4][Periph_TimChannel3]();
	}

	else if(TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET) {

		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
		IRQFunctions[Periph_Tim4][Periph_TimChannel4]();
	}
}

/***************************************************************************************************
 * Private Fonctions Definition
 */
/**-------------------------------------------------------------------------------------------------
 * @brief	Init Clock APB1 ou APB2 en fonction du TIM
 */
static inline void
TimeBase_RccInit(
		uint32_t Periph
) {
	switch(Periph) {

		case (uint32_t) TIM1: 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE); 	break;
		case (uint32_t) TIM2: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE); 	break;
		case (uint32_t) TIM3: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE); 	break;
		case (uint32_t) TIM4: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE); 	break;
		case (uint32_t) TIM5: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE); 	break;
		case (uint32_t) TIM6: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,  ENABLE); 	break;
		case (uint32_t) TIM7: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,  ENABLE); 	break;
		case (uint32_t) TIM8: 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,  ENABLE); 	break;
		case (uint32_t) TIM9: 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,  ENABLE); 	break;
		case (uint32_t) TIM10: 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE); 	break;
		case (uint32_t) TIM11: 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE); 	break;
		case (uint32_t) TIM12: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); 	break;
		case (uint32_t) TIM13: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); 	break;
		case (uint32_t) TIM14: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); 	break;
	}
}

