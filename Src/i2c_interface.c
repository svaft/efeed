/*
 * i2c_interface.c
 *
 *  Created on: 8Aug.,2017
 *      Author: yuri
 */

#include "i2c_interface.h"
#include "error_code.h"
#include "string.h"
#include "main.h"

#define USER_BUTTON_DEBOUNCE 5
//sample_log_t i2c_device_logging;
I2C_TypeDef *hi2c_ref;

volatile	uint32_t dma_delay = 0;
uint32_t 	dma_delay2 = 0;
uint8_t		device_ready = 0;
uint8_t		dma_data[6];

__IO uint8_t  ubTransferComplete = 1;



/**
  * @brief  This Function handle Master events to perform a transmission process
  * @note  This function is composed in different steps :
  *        -1- Enable DMA transfer.
  *        -2- Prepare acknowledge for Master data reception.
  *        -3- Initiate a Start condition to the Slave device.
  *        -4- Loop until end of transfer completed (DMA TC raised).
  *        -5- End of tranfer, Data consistency are checking into Slave process.
  * @param  None
  * @retval None
  */
//void Handle_I2C_MasterDMA_IT(void)
int Handle_I2C_MasterDMA_IT(I2C_TypeDef *I2Cx, uint8_t address, uint8_t *data, uint16_t count, uint8_t timeout)
{
#ifdef _SIMU
	return 0;
#endif

	if(ubTransferComplete == 1) {
		ubTransferComplete = 0;
	} else {
		while(!ubTransferComplete)
		{}
//		return 1;
	}
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, count);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)data, (uint32_t)LL_I2C_DMA_GetRegAddr(I2Cx), LL_DMA_DIRECTION_MEMORY_TO_PERIPH ); //  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);

	/* (1) Enable DMA transfer **************************************************/
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* (2) Prepare acknowledge for Master data reception ************************/
	LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);

	/* (3) Initiate a Start condition to the Slave device ***********************/
	/* Master Generate Start condition */
	LL_I2C_GenerateStartCondition(I2Cx);

	/* (4) Loop until end of transfer completed (DMA TC raised) *****************/

#if (USE_TIMEOUT == 1)
	int Timeout = timeout;
#endif /* USE_TIMEOUT */

	/* Loop until DMA transfer complete event */
	while(!ubTransferComplete) {
#if (USE_TIMEOUT == 1)
		/* Check Systick counter flag to decrement the time-out value */
		if (LL_SYSTICK_IsActiveCounterFlag()) {
			if(Timeout-- == 0) {
				/* Time-out occurred. Set LED to blinking mode */
				return -1;
			}
		}
#endif /* USE_TIMEOUT */
	}

	return 0;
	/* (5) End of tranfer, Data consistency are checking into Slave process *****/
}

int Handle_I2C_MasterDMA_IT_async(uint8_t address, uint8_t *data, uint16_t count)
{
#ifdef _SIMU
	return 0;
#endif

	if(ubTransferComplete == 1) {
		ubTransferComplete = 0;
	} else {
		return 1;
	}
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, count);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)data, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C2), LL_DMA_DIRECTION_MEMORY_TO_PERIPH ); //  LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);

	/* (1) Enable DMA transfer **************************************************/
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* (2) Prepare acknowledge for Master data reception ************************/
	LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);

	/* (3) Initiate a Start condition to the Slave device ***********************/
	/* Master Generate Start condition */
	LL_I2C_GenerateStartCondition(I2C2);
	return 0;
}




/**
  * @brief  This function Activate I2C2 peripheral (Master)
  * @note   This function is used to :
  *         -1- Enable I2C2.
  *         -2- Enable I2C2 transfer event/error interrupts.
  * @param  None
  * @retval None
  */
void Activate_I2C_Master(void)
{
	/* (1) Enable I2C2 **********************************************************/
	LL_I2C_Enable(I2C2);

	/* (2) Enable I2C2 transfer event/error interrupts:
	 *  - Enable Events interrupts
	 *  - Enable Errors interrupts
	*/
	LL_I2C_EnableIT_EVT(I2C2);
	LL_I2C_EnableIT_ERR(I2C2);
}

/**
  * @brief  DMA transfer complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
void Transfer_Complete_Callback()
{
	/* Generate Stop condition */
	while(!LL_I2C_IsActiveFlag_BTF(I2C2)) {
	}
	LL_I2C_GenerateStopCondition(I2C2);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	/* DMA transfer completed */
	ubTransferComplete = 1;
}

/**
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void Transfer_Error_Callback()
{
	/* Disable DMA1_Channel4_IRQn */
	NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	/* Error detected during DMA transfer */
	while(1) {};
}
