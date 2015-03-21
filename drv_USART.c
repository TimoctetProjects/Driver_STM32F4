/*********************************************************************
 *
 * @file	drv_USART.c
 * @author	Duclos Timothe
 * @date	15/10/2014
 * @brief	Gestion de l'USART
 *
 *********************************************************************/

/********************************************************************
 * Includes
 */
#include "drv_USART.h"


/********************************************************************
 * Private defines
 */
#define USART_TIMEOUT				100000	//10ms env.


/********************************************************************
 * Private macros
 */
#define	__IsPeriphUSART(Periph)	\
	(		(Periph) != (uint32_t) USART1 	\
		&& 	(Periph) != (uint32_t) USART2	\
		&& 	(Periph) != (uint32_t) USART3	\
		&& 	(Periph) != (uint32_t) USART6		)


#define __USART_GetInitializedState(periph) \
	( __USART[ USART_GetPeriphEnum(periph) ].Initialisazed )

#define __USART_GetBaudRateState(periph) \
	( __USART[ USART_GetPeriphEnum(periph) ].VitesseCom )

#define __USART_GetInterruptState(periph) \
	( __USART[ USART_GetPeriphEnum(periph) ].InterrupState )

#define __USART_SetInitializedState(periph, state) \
	( __USART[ USART_GetPeriphEnum(periph) ].Initialisazed = state )

#define __USART_SetInterruptState(periph, state) \
	( __USART[ USART_GetPeriphEnum(periph) ].InterrupState = state )

#define __USART_SetBaudRateState(periph, speed)	\
	( __USART[ USART_GetPeriphEnum(periph) ].VitesseCom = speed )


/********************************************************************
 * Private Types
 */
typedef struct {

	Bool_e 				Initialisazed;
	uint32_t			VitesseCom;
	InterruptState_e	InterrupState;
	Mapping_GPIO_e		IDMapp_PinRX;
	Mapping_GPIO_e		IDMapp_PinTX;
	FiFo_s				FIFO;

}Usart_s;

/********************************************************************
 * Private variables
 */
Usart_s	__USART[nb_Periph_Usart] = {

		{FALSE, 0, Interrupt_OFF},
		{FALSE, 0, Interrupt_OFF},
		{FALSE, 0, Interrupt_OFF},
		{FALSE, 0, Interrupt_OFF},
};



/********************************************************************
 * Private Fonction Prototype
 */
inline void USART_RccInit(uint32_t Periph);
inline Liste_Usart_Periph_e USART_GetPeriphEnum(uint32_t Periph);

/********************************************************************
 * Exported Fonction Definition
 */
/**------------------------------------------------------------------
 * @brief	Initialiser l'interface
 * @return	Status de l'initialisation
 */
UsartState_e
Usart_InitPeriph(
		Mapping_GPIO_e	ID_PinTX,	/**<[in] ID de la Pin TX. Mettre PIN_NULL si non utilisee */
		Mapping_GPIO_e	ID_PinRX,	/**<[in] ID de la Pin RX. Mettre PIN_NULL si non utilisee */
		uint32_t 		Baudrate	/**<[in] Valeur de la vitesse de la liaison (en bauds) */
) {
	NVIC_InitTypeDef 	NVIC_InitStructure;
	Mapping_GPIO_e 		ID_Pin = PIN_NULL;

	//----------------------------------------------------------
	//---------- Declaration des structures d'initialisation
	USART_InitTypeDef xUSART_Init = {

			.USART_BaudRate				= Baudrate,
			.USART_WordLength			= USART_WordLength_8b,
			.USART_StopBits				= USART_StopBits_1,
			.USART_Parity				= USART_Parity_No,
			.USART_HardwareFlowControl	= USART_HardwareFlowControl_None,
	};

	//------------ Test sur la validite de la config
	if (__IsPeriphUSART(Mapping_GPIO[ID_PinTX].Periph))
		return UsartErr_TryingToInitNonUsartPeriph;

	//------------ On regarde si pas déjà configuré
	if (__USART_GetInitializedState( Mapping_GPIO[ID_PinTX].Periph ) == TRUE)
		return UsartErr_AlreadyConfigured;


	if(ID_PinTX != PIN_NULL && ID_PinRX != PIN_NULL) {

			xUSART_Init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

			if(Mapping_GPIO[ID_PinTX].Periph != Mapping_GPIO[ID_PinRX].Periph)
				return UsartErr_RxTxDifferentPeriph;

			__USART[ USART_GetPeriphEnum(Mapping_GPIO[ID_PinTX].Periph) ].IDMapp_PinTX = ID_PinTX;
			__USART[ USART_GetPeriphEnum(Mapping_GPIO[ID_PinRX].Periph) ].IDMapp_PinRX = ID_PinRX;
			ID_Pin = ID_PinTX;
		  }

	else if(ID_PinTX != PIN_NULL) { ID_Pin = ID_PinTX; 	xUSART_Init.USART_Mode = USART_Mode_Tx; }
	else if(ID_PinRX != PIN_NULL) { ID_Pin = ID_PinRX; 	xUSART_Init.USART_Mode = USART_Mode_Rx; }

	else
			return UsartErr_RxOrTxPinInvalid;


	//---------- Activation horloges et init
	USART_RccInit		( Mapping_GPIO[ID_Pin].Periph						);
	USART_Init			( (USART_TypeDef*) Mapping_GPIO[ID_Pin].Periph, &xUSART_Init		);

	//---------- Démarrer le périphérique
	USART_Cmd			( (USART_TypeDef*) Mapping_GPIO[ID_Pin].Periph, ENABLE );

	//---------- Activation interruption si besoin est
	if( ID_PinRX != PIN_NULL && Mapping_GPIO[ID_PinRX].Etat_Interruption == Interrupt_ON ) {

		NVIC_InitStructure.NVIC_IRQChannel 						= USART6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 14;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0x01;
		NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
		NVIC_Init( &NVIC_InitStructure );
		USART_ITConfig( USART6, USART_IT_RXNE, ENABLE );
	}


	//---------- Save init
	__USART_SetInitializedState	( Mapping_GPIO[ID_Pin].Periph, TRUE 	  );
	__USART_SetBaudRateState	( Mapping_GPIO[ID_Pin].Periph, Baudrate );

	FiFo_init(&__USART[Periph_USART6].FIFO, 10, sizeof(uint8_t));

	//---------- Init OK
	return UsartOK;
}

/**------------------------------------------------------------------
 *
 * @brief	Ecriture sur l'USART
 *
 * @return	Status de l'ecriture
 *
 */
uint8_t
USART_SetInterruptState(
		Liste_Usart_Periph_e	UsartPeriph,/**<[in] ID de l'USART ou changer l'etat de l'IRQ */
		Liste_Pin_Usart_e		Tx_or_Rx,	/**<[in] Rx Tx ou les deux */
		FunctionalState			State		/**<[in] Etat de l'IRQ */
) {

	if ( __USART[UsartPeriph].Initialisazed == FALSE )		return UsartErr_NonInitialized;

	switch(Tx_or_Rx) {

		case Usart_Pin_BothRxTx:
					break;

		case Usart_Pin_Tx:
					break;

		case Usart_Pin_Rx:	USART_ITConfig( (USART_TypeDef*) Mapping_GPIO[__USART[UsartPeriph].IDMapp_PinRX].Periph,
							USART_IT_RXNE,
							State		);
					break;

		default:
					break;
	}

	return UsartOK;
}

/**------------------------------------------------------------------
 *
 * @brief	Ecriture sur l'USART
 *
 * @return	Status de l'ecriture
 *
 */
int
USART_Write(
		Mapping_GPIO_e	ID_PinTX,	/**<[in] ID de la Pin TX. Mettre PIN_NULL si non utilisee */
		uint8_t*		pBuffer,	/**<[in] Pointeur vers le buffer ou stocker la donnee a lire. */
		uint16_t 		Taille		/**<[in] Nombre d'octet a ecrire. */
) {

	uint16_t NbWrite = 0;
	uint32_t TO;

	while (Taille != 0) {

		//----------- On attend que la donnee d'avant soient envoyée
		TO = USART_TIMEOUT;
		while 	( (USART_GetFlagStatus( (USART_TypeDef*) Mapping_GPIO[ID_PinTX].Periph, USART_FLAG_TXE) != SET) && TO ) TO--;

		//----------- On quitte en cas d'erreur
		if 		(!TO) 		return UsartErr_SendErrorTimeout;

		//----------- Ecriture donnee
		USART_SendData( (USART_TypeDef*) Mapping_GPIO[ID_PinTX].Periph, pBuffer[NbWrite++]);
		Taille--;
	}

	return UsartOK;
}


/**------------------------------------------------------------------
 *
 * @brief	Lecture du buffer de reception.
 *
 * return	Nombre de donnees lues
 *
 */
uint16_t
USART_Read(
	uint32_t Periph,		/** <[in] Periph Usart ou effectuer la lecture */
	uint8_t* pBuffer,		/** <[out] Pointeur vers le Buffer ou ecrire les donnees */
	uint16_t NbRead			/** <[in] Nombre de donnees a lire */
) {

	uint16_t 		cmptRead	= 0;	// Comptage du nombre de lecture
	FiFo_State_e 	Fifo_State;			// Status de la FIFO;


	Fifo_State = FiFo_GetState(&__USART[USART_GetPeriphEnum(Periph)].FIFO);

	// Remplissage du buffer
	while ( (cmptRead != NbRead) && (Fifo_State != FiFoState_Empty) ){

		Fifo_State = FiFo_Pull(&__USART[USART_GetPeriphEnum(Periph)].FIFO, &(pBuffer[cmptRead])) ;
		cmptRead++;
	}

	return cmptRead;
}

/**------------------------------------------------------------------
 *
 * @brief	Interruption USART
 *
 * @return	void
 *
 */
void
USART6_IRQHandler(
	void
) {
	int uartStatusRegister = 0;

	if (USART_GetITStatus(USART6, USART_IT_ORE_RX) == SET) {

		uartStatusRegister = uartStatusRegister | USART_IT_ORE_RX;
		USART_ReceiveData(USART6);
		USART_ClearITPendingBit(USART6, USART_IT_ORE_RX);
	}

	//----------- Verification qu'un donne soit reçu
	if ( USART_GetITStatus(USART6, USART_IT_RXNE) == SET ) {

		FiFo_Push(&__USART[Periph_USART6].FIFO, (uint8_t) USART_ReceiveData(USART6));
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	}
}


/********************************************************************
 * Private Fonction Definition
 */

/**------------------------------------------------------------------
 *
 * @brief	Init des RCC
 *
 * @return	void
 *
 */
inline void
USART_RccInit(
		uint32_t Periph		/**<[in] Periph dont le RCC doit etre initialise */
) {
	switch(Periph) {

		case (uint32_t) USART1:		RCC_APB2PeriphClockCmd	(RCC_APB2Periph_USART1,	ENABLE);	break;
		case (uint32_t) USART2:		RCC_APB2PeriphClockCmd	(RCC_APB1Periph_USART2,	ENABLE);	break;
		case (uint32_t) USART3:		RCC_APB2PeriphClockCmd	(RCC_APB1Periph_USART3,	ENABLE);	break;
		case (uint32_t) USART6:		RCC_APB2PeriphClockCmd	(RCC_APB2Periph_USART6,	ENABLE);	break;
	}
}

/**------------------------------------------------------------------
 *
 * @brief	Init des RCC
 *
 * @return	enum periph
 *
 */
inline Liste_Usart_Periph_e
USART_GetPeriphEnum(
		uint32_t Periph		/**<[in] Periph dont le RCC doit etre initialise */
) {
	switch(Periph) {

		case (uint32_t) USART1:		return Periph_USART1;
		case (uint32_t) USART2:		return Periph_USART2;
		case (uint32_t) USART3:		return Periph_USART3;
		case (uint32_t) USART6:		return Periph_USART6;

		default:					return err_USARTPERIPH_NOTHANDLED;
	}
}
