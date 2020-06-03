/*
 * ring_buffer.h
 *
 *	The MIT License.
 *  Created on: 30.05.2020
 *      Author: Mateusz Salamon
 *		www.msalamon.pl
 *
 *      Website: https://msalamon.pl/
 *      GitHub: https://github.com/lamik/RingBuffer
 *      Contact: mateusz@msalamon.pl
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#define RINGBUFFER

typedef enum
{
	RB_OK,
	RB_ERROR,
	RB_NOTCREATED,
	RB_NOFREESPACE,
	RB_NOELEMENTS
} RB_Status;

typedef struct
{
	uint8_t Size;
	uint8_t Head;
	uint8_t Tail;
	uint8_t Elements;
	uint8_t Buffer[];	// Flexible Array Member
} RingBuffer;

RB_Status RB_CreateBuffer(RingBuffer **Buffer, uint8_t Size);
RB_Status RB_WriteToBuffer(RingBuffer *Buffer, uint8_t Data);
RB_Status RB_ReadFromBuffer(RingBuffer *Buffer, uint8_t *Data);
uint8_t RB_ElementsAvailable(RingBuffer *Buffer);

#endif /* INC_RING_BUFFER_H_ */
