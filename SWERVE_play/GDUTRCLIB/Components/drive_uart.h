#ifndef __DRIVE_UART_H
#define __DRIVE_UART_H

#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif 

// define the uart call back function
typedef uint32_t (*usart_call_back)(uint8_t *buf, uint16_t len);

/** 
* @brief define the uart struct
*/
typedef struct
{
    UART_HandleTypeDef *uart_handle;
    uint16_t rx_buffer_size;
    uint8_t *rx_buffer;
    usart_call_back call_back_fun;
}usart_manager_t;


extern usart_manager_t usart1_manager;
extern usart_manager_t usart2_manager;
extern usart_manager_t usart3_manager;
extern usart_manager_t usart6_manager;


void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint16_t len, usart_call_back call_back_fun);
void Usart_Rx_Callback_Register(usart_manager_t *manager, usart_call_back fun);
void Uart_Receive_Handler(usart_manager_t *manager);

unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len);

#ifdef __cplusplus
}
#endif
#endif
