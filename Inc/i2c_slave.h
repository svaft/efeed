/*
 * i2c_slave.h
 *
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#include "main.h"
#include "nuts_bolts.h"




extern uint8_t       aSlaveReceiveBuffer[];
extern uint8_t*      pSlaveTransmitBuffer;
extern __IO uint8_t  ubSlaveNbDataToTransmit;
extern uint8_t       ubSlaveInfoIndex;
extern __IO uint8_t  ubSlaveReceiveIndex;
extern __IO uint8_t  ubSlaveReceiveComplete;


/* Define used to enable time-out management*/
#define USE_TIMEOUT       0

#define SLAVE_OWN_ADDRESS                       0x5A /* This value is a left shift of a real 7 bits of a slave address
                                                        value which can find in a Datasheet as example: b0101101
                                                        mean in uint8_t equivalent at 0x2D and this value can be
                                                        seen in the OAR1 register in bits ADD[1:7] */

#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01

void     Configure_I2C_Slave(void);
void     Activate_I2C_Slave(void);



void Slave_Ready_To_Transmit_Callback(void);
void Slave_Reception_Callback(void);
void Slave_Complete_Callback(void);
void Transfer_Complete_Callback(void);
void Transfer_Error_Callback(void);
void Error_Callback(void);

void     Handle_I2C_Slave(void);

uint8_t  Buffercmp8(uint8_t* pBuffer1, uint8_t* pBuffer2, uint8_t BufferLength);
void     FlushBuffer8(uint8_t* pBuffer1);

#endif /* I2C_SLAVE_H_ */
