#include "Broadcast.h"

SystemTick_Fun Broadcast::get_systemTick = NULL;

uint8_t Broadcast::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        Broadcast::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}


void Broadcast::Send_To_Broadcast(Robot_Status_t status)
{
    PLAYLIST playlist = STOP;
    if(status.sensor == 0)
    {
        if(status.control_mode == 0)
        {
            if(status.path_mode == 0)
            {
                playlist = status.robot_init;   //机器人初始化
            }
            else
            {

                playlist = status.path_mode;    //路径规划模式
            }
        }
        else
        {
            playlist = status.control_mode;  //控制模式
        }
    }
    else
    {
        playlist = status.sensor;   //定位传感器故障
    }

    if(playlist != STOP)
        play(playlist);
}


void Broadcast::play(PLAYLIST playlist)
{
    UART_TxMsg UART_TxMsg; 
    static PLAYLIST last_playlist = STOP;
    static uint32_t start=0; uint32_t real_time=0;

    buffer[0] = Header;
    buffer[1] = 0x05;   //数据长度 (不包括帧头和帧尾)
    buffer[2] = Opcode;       //操作码
    buffer[3] = playlist>>8;       //曲目高位
    buffer[4] = playlist&0xff;   //曲目低位
    buffer[5] = Checksum_Analise(buffer[3], buffer[4]);
    buffer[6] = Tail;
	
    UART_TxMsg.len = 7;
    UART_TxMsg.data_addr = buffer;
    UART_TxMsg.huart = &huart1;
    
    if(last_playlist != playlist && playlist != POS_ERROR && playlist != STOP)
    {
        xQueueSend(UART_TxPort, &UART_TxMsg, 0);
    }

    if(playlist == POS_ERROR)   //2s播报一次
    {
        static uint8_t flag=0;
        if(flag==0)
        {
            start = get_systemTick();
            flag = 1;
			xQueueSend(UART_TxPort, &UART_TxMsg, 0);
        }

        if(get_systemTick() - start > 0)
            real_time = get_systemTick()/1000 - start/1000;
        else
            real_time = (get_systemTick()+0xFFFFFFFF)/1000 - start/1000;

        if(real_time > 3000)    //2s
        {
            flag = 0;
        }
    }

    last_playlist = playlist;
}


/**
 * @brief 校验码解析，目前只封装了1~255的校验码，也就是只能播报255个曲目
 * 
 * @param playlist 播放曲目
 * @return uint8_t 
 */
uint8_t Broadcast::Checksum_Analise(uint8_t high, uint8_t low)
{
    return 0x05^0x41^high^low;
}
