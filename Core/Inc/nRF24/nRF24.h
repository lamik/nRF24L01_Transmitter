/*
 * nRF24.h
 *
 *  Created on: Apr 26, 2020
 *      Author: Mateusz Salamon
 */

#ifndef INC_NRF24_NRF24_H_
#define INC_NRF24_NRF24_H_

#include "main.h"

//
//	Configuration
//
#define NRF24_USE_INTERRUPT		1
#if (NRF24_USE_INTERRUPT == 1)
#define NRF24_USE_RINGBUFFER	1
#endif


#define NRF24_DYNAMIC_PAYLOAD	1

#if (NRF24_USE_RINGBUFFER == 1)
#define NRF24_RX_BUFFER_SIZE 32
#define NRF24_TX_BUFFER_SIZE 32
#endif

//
// Enums
//
typedef enum
{
	NRF24_RECEIVED_PACKET,		// 0
	NRF24_NO_RECEIVED_PACKET,	// 1
} nRF24_RX_Status;

typedef enum
{
	NRF24_TRANSMITTED_PACKET,		// 0
	NRF24_NO_TRANSMITTED_PACKET,	// 1
} nRF24_TX_Status;

//
// Init
//
void nRF24_Init(SPI_HandleTypeDef *hspi);

//
// READ/WRITE REGISTERS
//
uint8_t nRF24_ReadConfig(void);
void nRF24_WriteConfig(uint8_t conf);
uint8_t nRF24_ReadStatus();
void nRF24_WriteStatus(uint8_t st);

//
// FIFO Status register
//
uint8_t nRF24_IsTxReuse(void);
uint8_t nRF24_IsTxFull(void);
uint8_t nRF24_IsTxEmpty(void);
uint8_t nRF24_IsRxFull(void);
uint8_t nRF24_IsRxEmpty(void);

//
// SWITCHING BETWEEN RX AND TX
//
void nRF24_RX_Mode(void);
void nRF24_TX_Mode(void);

//
// RADIO SETTINGS
//
void nRF24_SetPALevel(uint8_t lev);
void nRF24_SetDataRate(uint8_t dr);
void nRF24_EnableCRC(uint8_t onoff);
void nRF24_SetCRCLength(uint8_t crcl);
void nRF24_SetRetries(uint8_t ard, uint8_t arc);
void nRF24_SetRFChannel(uint8_t channel);
void nRF24_SetPayloadSize(uint8_t pipe, uint8_t size);
void nRF24_EnablePipe(uint8_t pipe, uint8_t onoff);
void nRF24_AutoACK(uint8_t pipe, uint8_t onoff);
void nRF24_SetRXAddress(uint8_t pipe, uint8_t* address); // Remember to define RX address before TX
void nRF24_SetTXAddress(uint8_t* address);
void nRF24_SetAddressWidth(uint8_t size);
void nRF24_SetPayloadSize(uint8_t pipe, uint8_t size);

//
// INTERRUPT CONTROL
//
void nRF24_ClearInterrupts(void);
void nRF24_EnableRXDataReadyIRQ(uint8_t onoff);
void nRF24_EnableTXDataSentIRQ(uint8_t onoff);
void nRF24_EnableMaxRetransmitIRQ(uint8_t onoff);

#if (NRF24_USE_INTERRUPT == 1)
//
//	Main event for whole communication
//	Put it in main while(1) loop
//
void nRF24_Event(void);
#endif

//
// TRANSMITTING DATA
//
nRF24_TX_Status nRF24_SendData(uint8_t *Data, uint8_t Size);
nRF24_RX_Status nRF24_ReadData(uint8_t *Data, uint8_t *Size);

//
// FLUSHING FIFOs
//
void nRF24_FlushRX(void);
void nRF24_FlushTX(void);

//
// Interrupt mode
//
void nRF24_IRQ_Handler(void);

void nRF24_EventRxCallback(void);
void nRF24_EventTxCallback(void);
void nRF24_EventMrCallback(void);

#endif /* INC_NRF24_NRF24_H_ */
