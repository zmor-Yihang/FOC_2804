#ifndef __USART_H__
#define __USART_H__

#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "../utils/fifofast.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx; /* 声明USART2接收DMA句柄 */
extern DMA_HandleTypeDef hdma_usart2_tx; /* 声明USART2发送DMA句柄 */

#define RX_BUF_TEMP_SIZE 1024    /* DMA循环接收缓冲区大小 */
#define FIFO_UART_TX_SIZE 2048   /* FIFO发送缓冲区大小 */
#define TX_DMA_BUF_SIZE 2048     /* DMA发送临时缓冲区大小 */
#define FIFO_UART_RX_SIZE 1024   /* FIFO接收缓冲区大小 */
#define TX_DMA_TRIGGER_THRESH 32 /* DMA发送触发阈值*/

void usart_init(void);
void usart_send_data(uint8_t *data, uint16_t size);
uint16_t usart_read_data(uint8_t *buf, uint16_t max_size);

uint16_t usart_get_availableBuffer(void);
uint8_t usart_is_rxFifoEmpty(void);

#endif /* __USART_H__ */
