/***************************************************************************************************
 *
 * @file	drv_PWM.c
 * @author	Duclos Timothe
 * @date	16/09/2014
 * @brief	Driver générant un signal PWM sur une broche pré-configurée
 *
 ***************************************************************************************************/

/***************************************************************************************************
 * Includes
 */
#include "drv_PWM.h"


/***************************************************************************************************
 * Private defines
 */
#define SIZEOF_TIMER_16BITS	0xFFFF			/** Taille du Timer 16 bits */
#define SIZEOF_TIMER_32BITS	0xFFFFFFFF		/** Taille du Timer 32 bits */


/***************************************************************************************************
 * Private macros
 */
#define __GetTimerSize(pERIPH) \
		(pERIPH) == (uint32_t) TIM2 ? SIZEOF_TIMER_32BITS : \
		(pERIPH) == (uint32_t) TIM5 ? SIZEOF_TIMER_32BITS : SIZEOF_TIMER_16BITS

#define __GetTimOCxInit(cHAN) \
		cHAN == TIM_Channel_1 ? (pFunction)TIM_OC1Init : \
		cHAN == TIM_Channel_2 ? (pFunction)TIM_OC2Init : \
		cHAN == TIM_Channel_3 ? (pFunction)TIM_OC3Init : (pFunction)TIM_OC4Init

#define __GetTimOCxPreloadInit(cHAN) \
		cHAN == TIM_Channel_1 ? (pFunction)TIM_OC1PreloadConfig : \
		cHAN == TIM_Channel_2 ? (pFunction)TIM_OC2PreloadConfig : \
		cHAN == TIM_Channel_3 ? (pFunction)TIM_OC3PreloadConfig : (pFunction)TIM_OC4PreloadConfig

#define	__GetPeriod_uS(Periph, Channel) \
	( PWM[ PWM_Save_GetTimPeriph(Periph) ][ PWM_Save_GetTimChannel(Channel) ].SavePeriode_us   )

#define	__GetRatio_pr100(Periph, Channel) \
	( PWM[ PWM_Save_GetTimPeriph(Periph) ][ PWM_Save_GetTimChannel(Channel) ].SaveRatio_pr100   )

#define	__GetRunningState(Periph, Channel) \
	( PWM[ PWM_Save_GetTimPeriph(Periph) ][ PWM_Save_GetTimChannel(Channel) ].RunningState   )

#define __GetConfState(periph, channel)	\
	( PWM[ PWM_Save_GetTimPeriph(periph) ][ PWM_Save_GetTimChannel(channel) ].ConfState )

#define __SetConfState(periph, channel, val) \
	( PWM[ PWM_Save_GetTimPeriph(periph) ][ PWM_Save_GetTimChannel(channel) ].ConfState = val )

#define	__SaveRatio_pr1000(Ratio, Periph, Channel) \
	( PWM[ PWM_Save_GetTimPeriph(Periph) ][ PWM_Save_GetTimChannel(Channel) ].SaveRatio_pr100   = Ratio 	)

#define	__SetRunningState(bool, Periph, Channel) \
	( PWM[ PWM_Save_GetTimPeriph(Periph) ][ PWM_Save_GetTimChannel(Channel) ].RunningState 	    = bool	)

#define	__SavePeriod_uS(Periode_us, Periph, Channel) \
	( PWM[ PWM_Save_GetTimPeriph(Periph) ][ PWM_Save_GetTimChannel(Channel) ].SavePeriode_us    = Periode_us)


#define __SetPWMOffString(str) \
	__strncat(  str, 	 OFF_STRING, strlen(OFF_STRING))

#define __SetRatioString(str, Ratio) \
	__strncat  ( str, Ratio, NB_CHAR_POURCENTAGE )

#define __PeriodeToFrequencyString(str, periode) \
	__snprintf ( str, 3, "%d", 		(int)(1000000 / periode)	)

#define __RatioToString(str, ratio) \
	__snprintf ( str, NB_CHAR_POURCENTAGE, 	" %d pr1000", 	ratio		)


/***************************************************************************************************
 * Private Types
 */
typedef enum {

	PWM_Stopped = 0,
	PWM_Running,
	PWM_OK,

}Liste_Etat_PWM_e;

typedef enum {

	PWM_Unconfigured = 0,
	PWM_Configured,

	err_PWM_AlreadyConfigured,
	err_PWM_CantBeConfigured,

}Liste_EtatConf_PWM_e;


/** Structure d'une PWM */
typedef struct {

	uint32_t 				SavePeriode_us;
	uint16_t 				SaveRatio_pr100;

	Liste_Etat_PWM_e 		RunningState;
	Liste_EtatConf_PWM_e	ConfState;

}PWM_s;

/***************************************************************************************************
 * Private variables
 */
static PWM_s	PWM[nb_TimPeriph][nb_TimChannels];

/***************************************************************************************************
 * Private Fonctions Prototypes
 */
inline static void 		 		 PWM_initOutputCompare		(Mapping_GPIO_e	IdPinPwm, uint32_t TIM_Periode, uint32_t Ratio_pr1000);
inline static void 		 		 PWM_initTimeBase_RccInit	(uint32_t Periph);
inline static ListePeriphTim_e 	 PWM_Save_GetTimPeriph		(uint32_t Periph);
inline static ListeTimChannels_e PWM_Save_GetTimChannel		(uint32_t Channel);

/***************************************************************************************************
 * Exported Fonctions Definition
 */
/**-------------------------------------------------------------------------------------------------
 * @brief	Initialisation de la PWM
 *
 */
uint8_t
PWM_Init(
	PWM_init_s PWM_init	/**<[in]Structure d'init d'une PWM */
) {

	uint32_t 	Periode_us 	= PWM_init.Periode_us;
	Mapping_GPIO_e	IdPinPwm= PWM_init.ID_Pin;
	uint32_t	Ratio_pr1000= PWM_init.Ratio_pr1000;

	uint32_t 	TIM_Periode 	= 1;	/** Periode de comptage du TimeBas */
	uint32_t 	TIM_Prescaler 	= 1;	/** Prescaler du TimeBase */
	uint32_t	SizeTimerMax	= 0;

	__CheckParameters(__IsPeriphTimxOK(Mapping_GPIO[IdPinPwm].Periph));

	//--------- Verif que pas deja conf
	if( __GetConfState(Mapping_GPIO[IdPinPwm].Periph, Mapping_GPIO[IdPinPwm].Parametre) ==  PWM_Configured)
		return err_PWM_AlreadyConfigured;

	//------- Calcul de la periode et du prescaler du Timer
	if(Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM8
	|| Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM9
	|| Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM1)
		TIM_Prescaler = (((SystemCoreClock / 1000000) / 2) - 1);
	else
		TIM_Prescaler = (((SystemCoreClock / 1000000) / 4) - 1);

	TIM_Periode = (Periode_us  * 2) - 1;

	//------- Recuperation de la taille max du timer
	SizeTimerMax = __GetTimerSize(Mapping_GPIO[IdPinPwm].Periph);

	//------- Verification que la taille n'est pas dépassée
	while (	TIM_Periode >= SizeTimerMax )	{

		TIM_Prescaler 	<<= 1;
		TIM_Periode 	>>= 1;
	}

	TimeBase_init		( (TimeBase_init_s) {	IdPinPwm,
												TIM_Periode,
												TIM_Prescaler,
												PWM_init.IRQHandle	} );

	PWM_initOutputCompare	(IdPinPwm, 	TIM_Periode, 	Ratio_pr1000	);

	//--------- Configuration interruption
	if( Mapping_GPIO[IdPinPwm].Etat_Interruption == Interrupt_ON ) {

		TimeBase_IT_init(	Mapping_GPIO[IdPinPwm].Periph,
							Mapping_GPIO[IdPinPwm].Parametre,
							12	);
	}

	//--------- Activation du Timer
	TIM_Cmd( (TIM_TypeDef*) Mapping_GPIO[IdPinPwm].Periph, ENABLE );

	if(Mapping_GPIO[IdPinPwm].EtatInit == ETAT_ACTIF) {

		PWM_Activer(IdPinPwm);

	} else {

		__SetRunningState(	PWM_Stopped,
							Mapping_GPIO[IdPinPwm].Periph,
							Mapping_GPIO[IdPinPwm].Parametre );
	}

	//--------- Sauvegarde de la periode
	__SavePeriod_uS( Periode_us,
					 Mapping_GPIO[IdPinPwm].Periph,
					 Mapping_GPIO[IdPinPwm].Parametre );

	__SetConfState( Mapping_GPIO[IdPinPwm].Periph,
					Mapping_GPIO[IdPinPwm].Parametre,
					PWM_Configured			  );

	return PWM_OK;
}

/**-------------------------------------------------------------------------------------------------
 *
 * @brief	Activation du signal PWM
 *
 */
uint8_t
PWM_Activer(
		Mapping_GPIO_e IdPinPwm		/** <[in] ID de la PIN où générer le signal PWM */
) {

	Liste_Etat_PWM_e PWM_State;

	//--------- Verif que periph conf
	if( __GetConfState(Mapping_GPIO[IdPinPwm].Periph, Mapping_GPIO[IdPinPwm].Parametre) ==  PWM_Unconfigured)
		return PWM_Unconfigured;

	PWM_State = __GetRunningState(	 Mapping_GPIO[IdPinPwm].Periph,
				 	 Mapping_GPIO[IdPinPwm].Parametre );

	if(PWM_State == PWM_Running)		return PWM_Running;



	__SetRunningState(	PWM_Running,
						Mapping_GPIO[IdPinPwm].Periph,
						Mapping_GPIO[IdPinPwm].Parametre );

	TIM_CCxCmd	  (	(TIM_TypeDef*)	Mapping_GPIO[IdPinPwm].Periph,
									Mapping_GPIO[IdPinPwm].Parametre,
									TIM_CCx_Enable					);

	if(Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM1
	|| Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM8)
		TIM_CtrlPWMOutputs(	(TIM_TypeDef*) 	Mapping_GPIO[IdPinPwm].Periph,
											ENABLE						);

	if(Mapping_GPIO[IdPinPwm].Etat_Interruption == Interrupt_ON)
		TIM_ITConfig(	(TIM_TypeDef* )	Mapping_GPIO[IdPinPwm].Periph,
										__TimBase_GetITChanel(Mapping_GPIO[IdPinPwm].Parametre),
										ENABLE);


	return PWM_Running;
}


/**-------------------------------------------------------------------------------------------------
 *
 * @brief	Desactivation du signal PWM
 *
 */
uint8_t
PWM_Desactiver(
		Mapping_GPIO_e IdPinPwm		/** <[in] ID de la PIN où le signal PWM était généré */
) {

	Liste_Etat_PWM_e PWM_State;

	//--------- Verif que periph conf
	if( __GetConfState(Mapping_GPIO[IdPinPwm].Periph, Mapping_GPIO[IdPinPwm].Parametre) ==  PWM_Unconfigured)
		return PWM_Unconfigured;

	PWM_State = __GetRunningState(	Mapping_GPIO[IdPinPwm].Periph,
									Mapping_GPIO[IdPinPwm].Parametre );

	if(PWM_State == PWM_Stopped)		return PWM_Stopped;



	__SetRunningState(	 FALSE,
						 Mapping_GPIO[IdPinPwm].Periph,
						 Mapping_GPIO[IdPinPwm].Parametre );

	TIM_CCxCmd	  (	(TIM_TypeDef*) 	Mapping_GPIO[IdPinPwm].Periph,
									Mapping_GPIO[IdPinPwm].Parametre,
									TIM_CCx_Disable				);

	if(Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM1
	|| Mapping_GPIO[IdPinPwm].Periph == (uint32_t)TIM8)
		TIM_CtrlPWMOutputs(	(TIM_TypeDef*) 	Mapping_GPIO[IdPinPwm].Periph,
											DISABLE						);

	/*if(Mapping_GPIO[IdPinPwm].Etat_Interruption == Interrupt_ON)
		TIM_ITConfig(	(TIM_TypeDef* )Mapping_GPIO[IdPinPwm].Periph,
				__TimBase_GetITChanel(Mapping_GPIO[IdPinPwm].Parametre),
				DISABLE);*/

	return PWM_Stopped;
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Desinitialisation de la PWM
 *
 */
void
PWM_Desinit(
			Mapping_GPIO_e	IdPinPwm		/**<[in] ID de la Pin ou générer la PWM*/
) {
	TIM_DeInit( (TIM_TypeDef*) Mapping_GPIO[IdPinPwm].Periph );

	__SavePeriod_uS( 0,
			 Mapping_GPIO[IdPinPwm].Periph,
			 Mapping_GPIO[IdPinPwm].Parametre );

	__SetRunningState( PWM_Stopped,
			   Mapping_GPIO[IdPinPwm].Periph,
			   Mapping_GPIO[IdPinPwm].Parametre );

	__SetConfState( Mapping_GPIO[IdPinPwm].Periph,
			Mapping_GPIO[IdPinPwm].Parametre,
			PWM_Unconfigured			);
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Lecture de la configuration d'une PWM
 *
 * @return	void
 *
 */
void
PWM_GetConfiguration(
			Mapping_GPIO_e	IdPinPwm,	/**<[in] ID de la Pin ou générer la PWM */
			uint32_t* 	Periode_us,	/**<[out] Periode de la PWM en microsecondes */
			uint16_t*	Ratio_pr100	/**<[out] Ratio de la PWM en pr100 */
) {

	Liste_Etat_PWM_e PWM_Running;

	PWM_Running = __GetRunningState( Mapping_GPIO[IdPinPwm].Periph,
					 Mapping_GPIO[IdPinPwm].Parametre );

	if(PWM_Running == PWM_Stopped) {
		*Periode_us  = 0;
		*Ratio_pr100 = 0;
		return;
	}

	*Periode_us = __GetPeriod_uS( 	Mapping_GPIO[IdPinPwm].Periph,
									Mapping_GPIO[IdPinPwm].Parametre );


	*Ratio_pr100 = __GetRatio_pr100( Mapping_GPIO[IdPinPwm].Periph,
									 Mapping_GPIO[IdPinPwm].Parametre );
}


/***************************************************************************************************
 * Private Fonctions Definition
 */
/**-------------------------------------------------------------------------------------------------
 *
 * @brief	Initialisation de OutputCompare
 *
 * @warning La Pin doit être configuré
 *
 */
inline static void
PWM_initOutputCompare(
		Mapping_GPIO_e	IdPinPwm,	/**<[in] ID de la Pin ou générer la PWM*/
		uint32_t 		TIM_Periode,	/**<[in] Periode de comptage */
		uint32_t		Ratio_pr1000	/**<[in] Ratio du signal PWM en % */
) {
	//---------- Déclaration des variables
	uint32_t 			PulseLength_ms;		/** Duree de l'etat haut de la PWM*/

	TIM_OCInitTypeDef 	TIM_OCInit;		/** Structure d'initialisation d'outputcompare */
	pFunction 			pOCxInit;
	pFunction 			pOCxPreloadingConfig;

	TIM_OCStructInit	(&TIM_OCInit);

	//---------- Remplissage de la structure d'init TimOutputCompare
	TIM_OCInit.TIM_OutputState 	= TIM_OutputState_Disable;
	TIM_OCInit.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;

	//---------------- Sauvegarde du ratio
	__SaveRatio_pr1000( Ratio_pr1000,
						Mapping_GPIO[IdPinPwm].Periph,
						Mapping_GPIO[IdPinPwm].Parametre );

	//---------- Calcul de la duree de l'etat haut
	if (Ratio_pr1000 >= 1000)	{
		TIM_OCInit.TIM_Pulse = TIM_Periode;
	} else	{
		PulseLength_ms  = Ratio_pr1000 * TIM_Periode;
		PulseLength_ms /= 1000;
		TIM_OCInit.TIM_Pulse = PulseLength_ms;
	}

	pOCxInit 				= __GetTimOCxInit		(Mapping_GPIO[IdPinPwm].Parametre);
	pOCxPreloadingConfig	= __GetTimOCxPreloadInit(Mapping_GPIO[IdPinPwm].Parametre);

	pOCxInit				( (TIM_TypeDef*) Mapping_GPIO[IdPinPwm].Periph, &TIM_OCInit		);
	pOCxPreloadingConfig	( (TIM_TypeDef*) Mapping_GPIO[IdPinPwm].Periph, TIM_OCPreload_Enable	);
}

/**-------------------------------------------------------------------------------------------------
 *
 * @brief	Init Clock APB1 ou APB2 en fonction du TIM
 *
 */
inline static void
PWM_initTimeBase_RccInit(
		uint32_t Periph
) {
	switch(Periph) {

		case (uint32_t) TIM1: 	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1,  ENABLE); 	break;
		case (uint32_t) TIM2: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE); 	break;
		case (uint32_t) TIM3: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE); 	break;
		case (uint32_t) TIM4: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,  ENABLE); 	break;
		case (uint32_t) TIM5: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE); 	break;
		case (uint32_t) TIM6: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,  ENABLE); 	break;
		case (uint32_t) TIM7: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,  ENABLE); 	break;
		case (uint32_t) TIM8: 	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM8,  ENABLE); 	break;
		case (uint32_t) TIM9: 	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM9,  ENABLE); 	break;
		case (uint32_t) TIM10: 	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE); 	break;
		case (uint32_t) TIM11: 	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE); 	break;
		case (uint32_t) TIM12: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); 	break;
		case (uint32_t) TIM13: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE); 	break;
		case (uint32_t) TIM14: 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE); 	break;
	}
}

/**-------------------------------------------------------------------------------------------------
 *
 * @brief	Lecture du periph TIM
 *
 * @return	Periph
 *
 */
inline static ListePeriphTim_e
PWM_Save_GetTimPeriph(
		uint32_t Periph
) {
	switch(Periph) {

		case (uint32_t) TIM1: 	return Periph_Tim1; 		break;
		case (uint32_t) TIM2: 	return Periph_Tim2; 		break;
		case (uint32_t) TIM3: 	return Periph_Tim3;  		break;
		case (uint32_t) TIM4: 	return Periph_Tim4;  		break;
		case (uint32_t) TIM5: 	return Periph_Tim5;  		break;
		case (uint32_t) TIM6: 	return Periph_Tim6;  		break;
		case (uint32_t) TIM7: 	return Periph_Tim7;  		break;
		case (uint32_t) TIM8: 	return Periph_Tim8;  		break;
		case (uint32_t) TIM9: 	return Periph_Tim9;  		break;
		case (uint32_t) TIM10: 	return Periph_Tim10; 		break;
		case (uint32_t) TIM11: 	return Periph_Tim11; 		break;
		case (uint32_t) TIM12: 	return Periph_Tim12; 		break;
		case (uint32_t) TIM13: 	return Periph_Tim13; 		break;
		case (uint32_t) TIM14: 	return Periph_Tim14; 		break;

		default:		return err_NotATimPeriph;	break;
	}
}

/**-------------------------------------------------------------------------------------------------
 *
 * @brief	Lecture du periph TIM
 *
 * @return	Channel
 *
 */
inline static ListeTimChannels_e
PWM_Save_GetTimChannel(
		uint32_t	Channel
) {
	switch(Channel) {

		case (uint32_t) TIM_Channel_1:	return Periph_TimChannel1;	break;
		case (uint32_t) TIM_Channel_2:	return Periph_TimChannel2;	break;
		case (uint32_t) TIM_Channel_3:	return Periph_TimChannel3;	break;
		case (uint32_t) TIM_Channel_4:	return Periph_TimChannel4;	break;

		default:						return err_NotATimChannel;	break;
	}
}

/**-------------------------------------------------------------------------------------------------
 *
 * @brief	ToString Valeur PWM
 *
 */
inline void
PWM_Value_toString(

		toString_Possibilities_e	Field,
		Mapping_GPIO_e 			IDMapping,
		uint8_t*			pString
) {
	uint32_t	Periode_us;
	uint16_t	Ratio_pr100;
	uint8_t		Ratio[NB_CHAR_POURCENTAGE];
	uint8_t		Freq[NB_CHAR_FREQUENCY];

	memset(Ratio, 0, NB_CHAR_POURCENTAGE);
	memset(Freq, 0, NB_CHAR_FREQUENCY);

	switch(Field) {

		//----------------------------------------------------------
		case toString_Getpin:	toString_GetPeriphral	(Mapping_GPIO[IDMapping].GpioPeripheral,pString);
								toString_GetPin			(Mapping_GPIO[IDMapping].GpioPin, 	pString);
								break;

		//----------------------------------------------------------
		case toString_GetValue:	PWM_GetConfiguration(IDMapping, &Periode_us, &Ratio_pr100);

					if(Periode_us != 0) {

						if(Periode_us < 1000) {
							__PeriodeToFrequencyString (Freq, (Periode_us*1000));
							__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_verte);
							__SetFrequencyString(pString, Freq);
							__SetFrequencyUnit_KHz(pString);
						} else {
							__PeriodeToFrequencyString (Freq, Periode_us);
							__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_verte);
							__SetFrequencyString(pString, Freq);
							__SetFrequencyUnit_Hz(pString);
						}
						__RatioToString	  (Ratio,   Ratio_pr100);
						__SetRatioString  (pString, Ratio);
						__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_blanc);

					} else {
						__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_rouge);
						__SetPWMOffString(pString);
						__VT100STRING_SET_FOREGROUND_COLOR(pString, Couleur_default);
					}break;

		//----------------------------------------------------------
		default:
					break;
	}
}
