#include "serial_tool.h"




//放弃使用这种方式，在单片机尽量不适用动态内存分配
// std::vector<uint8_t> SerialTools::Pack(const void *const p_PDU, int PDU_len)
// {
//     uint8_t *p_PDU_data = (uint8_t *)p_PDU;

//     //计算所需的内存大小
//     int SDU_len = PDU_len + 6;  //添加包头包尾、校验位和数据长度
//     std::vector<uint8_t> SDU_data;
//     SDU_data.reserve(SDU_len);

//     //添加包头
//     SDU_data.push_back(0x55);
//     SDU_data.push_back(0xAA);
//     SDU_data.push_back(PDU_len);

//     //添加数据
//     for(uint8_t *p = p_PDU_data; p < p_PDU_data + PDU_len; p++)
//     {
//         SDU_data.push_back(*p);
//     }

//     //添加校验位
//     SDU_data.push_back(serial_get_crc8_value(p_PDU_data, PDU_len+3));
//     SDU_data.push_back(0x0D);
//     SDU_data.push_back(0x0A);

//     return SDU_data;
// }

