#pragma once
#include "stdint.h"
#include "data_pool.h"

#ifdef __cplusplus

class SerialTools
{
public:
    virtual uint8_t* get_header() 
    {
        static uint8_t header[2] = {0x55, 0xAA};
        return header;
    }

    virtual uint8_t* get_tail() 
    {
        static uint8_t tail[2] = {0x0D, 0x0A};
        return tail;
    }

    void Serial_SendData(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
    {
        TxMsg.huart = huart;
        TxMsg.len = len;
        TxMsg.data_addr = data;
        xQueueSend(UART_TxPort, &TxMsg, 0);
    }
private:
    unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len);
    UART_TxMsg TxMsg;
};



#endif
