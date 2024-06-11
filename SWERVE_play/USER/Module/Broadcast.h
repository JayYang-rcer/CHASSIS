#pragma once
#include "stdint.h"
#include "drive_uart.h"
#include "data_pool.h"
#include "tool.h"

typedef uint32_t (*SystemTick_Fun)(void);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}

class Broadcast
{
public:
Broadcast(){};
static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
void Send_To_Broadcast(Robot_Status_t status);

private:
static SystemTick_Fun get_systemTick;
uint8_t Checksum_Analise(uint8_t high, uint8_t low);
void play(PLAYLIST playlist);
uint8_t Header = 0x7E; uint8_t Tail = 0xEF;
uint8_t Opcode = 0x41;
uint8_t buffer[7];
};
#endif
