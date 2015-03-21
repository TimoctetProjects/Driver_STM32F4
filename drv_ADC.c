/***************************************************************************************************
 *
 * @author	Duclos Timothe
 * @date	16/01/2015
 * @file	drv_ADC.c
 * @brief	Driver ADC
 *
 ***************************************************************************************************/


/***************************************************************************************************
 * Includes
 */
#include "drv_ADC.h"

/***************************************************************************************************
 * Private defines
 */
#define NB_PERIPH_ADC	3

/***************************************************************************************************
 * Private macros
 */
#define __IsADCxKO(aDCX)	\
	(uint32_t)aDCX == (uint32_t)ADC1 ? TRUE : 	\
	(uint32_t)aDCX == (uint32_t)ADC2 ? TRUE :  	\
	(uint32_t)aDCX == (uint32_t)ADC3 ? TRUE : FALSE

#define __GetADCPeriphRCC(aDCX)	\
	  (uint32_t)aDCX == (uint32_t)ADC1 ? RCC_APB2Periph_ADC1 : \
	  (uint32_t)aDCX == (uint32_t)ADC2 ? RCC_APB2Periph_ADC2 : RCC_APB2Periph_ADC3

#define __GetPeriphEnum(aDCX) \
	(uint32_t)aDCX == (uint32_t)ADC1 ? 0 : 	\
	(uint32_t)aDCX == (uint32_t)ADC2 ? 1 :  2


/***************************************************************************************************
 * Private Types
 */
/** @brief Structure de gestion d'un ADC */
typedef struct {
	uint32_t* 	ConvValue_mV	[4];
	uint8_t 	ID_CurrentValue;
	Bool_e		Conv_Running;

	ADC_s 		ConvQueu		[4];
	Bool_e		isConvQueuedON	[4];
	Bool_e 		IsConv_Queued;
	pFunction	IRQFunction		[4];

}ADC_HAL_s;

/***************************************************************************************************
 * Private variables
 */

static 	 Bool_e 	IsADCAlreadyInited	[NB_PERIPH_ADC];
static   ADC_HAL_s	ADC_HAL;
static 	 uint8_t	ID_AssignValue;
static 	 uint32_t 	ID_QUEUE;

/***************************************************************************************************
 * Private Functions prototypes
 */


/***************************************************************************************************
 * Exported Functions definition
 */
/**-------------------------------------------------------------------------------------------------
 * @brief	Configuration d'un peripherique ADC
 */
ADC_Status_e
ADC_PeriphConfigure(
	ADC_s* ADCStruct	/**<[in] Structure de l'ADC */
) {
	//------------------------------------------------------
	//--------- Declaration des variables
	ADC_CommonInitTypeDef 	ADC_CommonInitStruct;
	ADC_InitTypeDef 		xADC_Init;
	NVIC_InitTypeDef 		NVIC_InitStructure;
	ADC_TypeDef* 			PeriphADCx;

	PeriphADCx = (ADC_TypeDef*)Mapping_GPIO[ADCStruct->ID_Pin].Periph;

	__CheckParameters(__IsADCxKO(PeriphADCx));

	ADCStruct->ID_Mesure 						= ID_AssignValue++;
	ADC_HAL.ConvValue_mV[ADCStruct->ID_Mesure] 	= ADCStruct->StoreValue_mV;
	ADC_HAL.IRQFunction [ADCStruct->ID_Mesure]	= ADCStruct->IRQFunction;

	if(IsADCAlreadyInited[__GetPeriphEnum(PeriphADCx)])
		return ADC_err_PeriphAlreadyInited;

	//-------- Init structure
	ADC_CommonStructInit(&ADC_CommonInitStruct);
	ADC_StructInit		(&xADC_Init);

	//-------- Remplissage structure
	ADC_CommonInitStruct.ADC_Mode				= ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler			= ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode		= ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay	= ADC_TwoSamplingDelay_5Cycles;

	xADC_Init.ADC_Resolution			= ADC_Resolution_12b;
	xADC_Init.ADC_ScanConvMode			= DISABLE;
	xADC_Init.ADC_ContinuousConvMode	= DISABLE;
	xADC_Init.ADC_ExternalTrigConvEdge	= ADC_ExternalTrigConvEdge_None;
	xADC_Init.ADC_ExternalTrigConv		= 0;
	xADC_Init.ADC_DataAlign				= ADC_DataAlign_Right;
	xADC_Init.ADC_NbrOfConversion		= 1;

	//---------- Init commune a tous les ADC
	ADC_CommonInit(&ADC_CommonInitStruct);

	//---------- Activation horloges
	RCC_APB2PeriphClockCmd(__GetADCPeriphRCC(PeriphADCx), ENABLE);

	//--------- Init ADC
	ADC_Init( PeriphADCx, &xADC_Init );

	//--------- Config interruption
	NVIC_InitStructure.NVIC_IRQChannel 						= ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;

	ADC_ITConfig( PeriphADCx, ADC_IT_EOC, ENABLE );
	NVIC_Init	( &NVIC_InitStructure );

	//---------------- Activation peripherique
	ADC_Cmd( PeriphADCx, ENABLE );

	IsADCAlreadyInited[__GetPeriphEnum(PeriphADCx)] = TRUE;

	return ADC_OK;
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Configuration d'un peripherique ADC
 */
ADC_Status_e
ADC_StartConv(
	ADC_s 	ADCStruct	/**<[in] Structure de l'ADC */
) {
	ADC_TypeDef* PeriphADCx = (ADC_TypeDef*)Mapping_GPIO[ADCStruct.ID_Pin].Periph;

	if (!IsADCAlreadyInited[__GetPeriphEnum(PeriphADCx)])	return ADC_err_PeriphNotInited;

	if (ADC_HAL.Conv_Running) {
		if(ID_QUEUE < 4) {
				ADC_HAL.IsConv_Queued      			= TRUE;
				ADC_HAL.ConvQueu[ID_QUEUE].ID_Pin   = ADCStruct.ID_Pin;
				ADC_HAL.ConvQueu[ID_QUEUE].ID_Mesure= ADCStruct.ID_Mesure;
				ADC_HAL.isConvQueuedON[ID_QUEUE]	= TRUE;
				ID_QUEUE++;
		} else 	ID_QUEUE = 0;
				return ADC_AlreadyRunning;
	}

	ADC_HAL.ID_CurrentValue = ADCStruct.ID_Mesure;
	ADC_HAL.Conv_Running 	= TRUE;

	ADC_RegularChannelConfig( PeriphADCx,
				  Mapping_GPIO[ADCStruct.ID_Pin].Parametre,
				  1,
				  ADC_TwoSamplingDelay_5Cycles			);

	ADC_SoftwareStartConv	(PeriphADCx);

	return ADC_OK;
}

uint8_t
ADC_GetNextConv() {
	uint8_t b_Conv = 0;

	for(b_Conv = 0; b_Conv<4; b_Conv++) {
		if(ADC_HAL.isConvQueuedON[b_Conv])
			return b_Conv;
	}	return 255;
}

/**-------------------------------------------------------------------------------------------------
 * @brief	Interruption ADC
 */
void
ADC_IRQHandler(
	void
) {
	uint32_t IRQ_QUEUE = 0;
	uint32_t Value_mV;

	if(ADC_GetITStatus(ADC1, ADC_IT_OVR)) {
		ADC_ClearITPendingBit(ADC1, ADC_IT_OVR);

	} else if(ADC_GetITStatus(ADC1, ADC_IT_EOC)) {

		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		Value_mV = ADC_GetConversionValue(ADC1);

		ADC_HAL.Conv_Running = FALSE;

		if(ADC_HAL.ConvValue_mV[ADC_HAL.ID_CurrentValue] != NULL) {
			*(ADC_HAL.ConvValue_mV[ADC_HAL.ID_CurrentValue])  = Value_mV;
		}

		if(ADC_HAL.IRQFunction [ADC_HAL.ID_CurrentValue] != NULL) {
			ADC_HAL.IRQFunction[ADC_HAL.ID_CurrentValue]((void *)&Value_mV);
		}

		if(ADC_HAL.IsConv_Queued)	{
			IRQ_QUEUE = ADC_GetNextConv();
			if(IRQ_QUEUE != 255) {
				ADC_StartConv(ADC_HAL.ConvQueu[IRQ_QUEUE]);
				ADC_HAL.isConvQueuedON[IRQ_QUEUE]	= FALSE;
			} else 	ADC_HAL.IsConv_Queued = FALSE;

		}
	}
}


