#include "i2c_slave.h"


/**
  * @brief  Flush 8-bit buffer.
  * @param  pBuffer1: pointer to the buffer to be flushed.
  * @retval None
  */
void FlushBuffer8(uint8_t* pBuffer1)
{
  uint8_t Index = 0;

  for (Index = 0; Index < sizeof(pBuffer1); Index++)
  {
    pBuffer1[Index] = 0;
  }
}


/**
  * @brief  Compares two 8-bit buffers and returns the comparison result.
  * @param  pBuffer1: pointer to the source buffer to be compared to.
  * @param  pBuffer2: pointer to the second source buffer to be compared to the first.
  * @param  BufferLength: buffer's length.
  * @retval 0: Comparison is OK (the two Buffers are identical)
  *         Value different from 0: Comparison is NOK (Buffers are different)
  */
uint8_t Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}


/**
  * @brief Variables related to Slave process
  */
const char*   aSlaveInfo[]              = {
                                          "STM32F103RBT6",
                                          "1.2.3"};

uint8_t       aSlaveReceiveBuffer[0xF]  = {0};
uint8_t*      pSlaveTransmitBuffer      = 0;
__IO uint8_t  ubSlaveNbDataToTransmit   = 0;
uint8_t       ubSlaveInfoIndex          = 0xFF;
__IO uint8_t  ubSlaveReceiveIndex       = 0;
__IO uint8_t  ubSlaveReceiveComplete    = 0;




/**
  * @brief  Function called from I2C IRQ Handler when TXE flag is set
  *         Function is in charge of transmit a byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Ready_To_Transmit_Callback(void)
{
  if(ubSlaveNbDataToTransmit > 0)
  {
    /* Send the Byte requested by the Master */
    LL_I2C_TransmitData8(I2C2, (uint8_t)(*pSlaveTransmitBuffer++));
    
    ubSlaveNbDataToTransmit--;
  }
  else
  {
    /* Send the NULL Byte until Master stop the communication */
    /* This is needed due to Master don't know how many data slave will sent */
    LL_I2C_TransmitData8(I2C2, 0x00);
  }
}

/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void Slave_Reception_Callback(void)
{
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */
  aSlaveReceiveBuffer[ubSlaveReceiveIndex++] = LL_I2C_ReceiveData8(I2C2);

  /* Check Command code */
//  if(Buffercmp8((uint8_t*)aSlaveReceiveBuffer, (uint8_t*)(aCommandCode[0][0]), (ubSlaveReceiveIndex-1)) == 0)
  {
    ubSlaveInfoIndex = 0;//SLAVE_CHIP_NAME;
    ubSlaveNbDataToTransmit = strlen(aSlaveInfo[ubSlaveInfoIndex]);
    pSlaveTransmitBuffer = (uint8_t*)(aSlaveInfo[ubSlaveInfoIndex]);
  }
//  else if(Buffercmp8((uint8_t*)aSlaveReceiveBuffer, (uint8_t*)(aCommandCode[1][0]), (ubSlaveReceiveIndex-1)) == 0)
  {
    ubSlaveInfoIndex = 1;//SLAVE_CHIP_REVISION;
    ubSlaveNbDataToTransmit = strlen(aSlaveInfo[ubSlaveInfoIndex]);
    pSlaveTransmitBuffer = (uint8_t*)(aSlaveInfo[ubSlaveInfoIndex]);
  }
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED2 is On if data are correct.
  * @param  None
  * @retval None
  */
void Slave_Complete_Callback(void)
{
  /* Clear and Reset process variables and arrays */
  ubSlaveReceiveIndex       = 0;
  ubSlaveReceiveComplete    = 0;
  FlushBuffer8(aSlaveReceiveBuffer);
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
  while(1);
}

/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  /* Disable I2C2_EV_IRQn */
  NVIC_DisableIRQ(I2C2_EV_IRQn);

  /* Disable I2C2_ER_IRQn */
  NVIC_DisableIRQ(I2C2_ER_IRQn);

  /* Unexpected event : Set LED2 to Blinking mode to indicate error occurs */
  while(1);//LED_Blinking(LED_BLINK_ERROR);
}


/**
  * @brief  This Function handle Slave events to perform a transmission process
  * @note  This function is composed in one step :
  *        -1- Prepare acknowledge for Slave address reception.
  * @param  None
  * @retval None
  */
void Handle_I2C_Slave(void)
{
  /* (1) Prepare acknowledge for Slave address reception **********************/
  LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
}

/**
  * @brief  This function Activate I2C2 peripheral (Slave)
  * @note   This function is used to :
  *         -1- Enable I2C2.
  *         -2- Enable I2C2 transfer event/error interrupts.
  * @param  None
  * @retval None
  */
void Activate_I2C_Slave(void)
{
  /* (1) Enable I2C2 **********************************************************/
  LL_I2C_Enable(I2C2);
  
  /* (2) Enable I2C2 transfer event/error interrupts:
   *  - Enable Event interrupts
   *  - Enable Error interrupts
   */
  LL_I2C_EnableIT_EVT(I2C2);
  LL_I2C_EnableIT_ERR(I2C2);
}

/**
  * @brief  This function configures I2C2 in Slave mode.
  * @note   This function is used to :
  *         -1- Enables GPIO clock.
  *         -2- Enable the I2C2 peripheral clock and configures the I2C2 pins.
  *         -3- Configure NVIC for I2C2.
  *         -4- Configure I2C2 functional parameters.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_I2C_Slave(void)
{
  /* (1) Enables GPIO clock */
  /* Enable the peripheral clock of GPIOB */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /* (2) Enable the I2C2 peripheral clock *************************************/

  /* Enable the peripheral clock for I2C2 */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
  
  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

  /* (3) Configure NVIC for I2C2 **********************************************/

  /* Configure Event IT:
   *  - Set priority for I2C2_EV_IRQn
   *  - Enable I2C2_EV_IRQn
   */
  NVIC_SetPriority(I2C2_EV_IRQn, 0xF);  
  NVIC_EnableIRQ(I2C2_EV_IRQn);

  /* Configure Error IT:
   *  - Set priority for I2C2_ER_IRQn
   *  - Enable I2C2_ER_IRQn
   */
  NVIC_SetPriority(I2C2_ER_IRQn, 0xF);  
  NVIC_EnableIRQ(I2C2_ER_IRQn);

  /* (4) Configure I2C2 functional parameters ***********************/
  
  /* Disable I2C2 prior modifying configuration registers */
  LL_I2C_Disable(I2C2);
  
  /* Configure the Own Address1 :
   *  - OwnAddress1 is SLAVE_OWN_ADDRESS
   *  - OwnAddrSize is LL_I2C_OWNADDRESS1_7BIT
   */
  LL_I2C_SetOwnAddress1(I2C2, SLAVE_OWN_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);

  /* Enable Clock stretching */
  /* Reset Value is Clock stretching enabled */
  //LL_I2C_EnableClockStretching(I2C2);
  
  /* Enable General Call                  */
  /* Reset Value is General Call disabled */
  //LL_I2C_EnableGeneralCall(I2C2);

  /* Configure the 7bits Own Address2     */
  /* Reset Values of :
   *     - OwnAddress2 is 0x00
   *     - Own Address2 is disabled
   */
  //LL_I2C_SetOwnAddress2(I2C2, 0x00);
  //LL_I2C_DisableOwnAddress2(I2C2);

  /* Enable Peripheral in I2C mode */
  /* Reset Value is I2C mode */
  //LL_I2C_SetMode(I2C2, LL_I2C_MODE_I2C);
}

