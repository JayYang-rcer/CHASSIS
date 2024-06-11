#ifndef __DRIVE_CAN_H
#define __DRIVE_CAN_H
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif 

#define CAN_RxExtId 0x0000
#define CAN_TxExtId 0x0000

#define CanFilter_0     (0  << 3)
#define CanFilter_1     (1  << 3)
#define CanFilter_2     (2  << 3)
#define CanFilter_3     (3  << 3)
#define CanFilter_4     (4  << 3)
#define CanFilter_5     (5  << 3)
#define CanFilter_6     (6  << 3)
#define CanFilter_7     (7  << 3)
#define CanFilter_8     (8  << 3)
#define CanFilter_9     (9  << 3)
#define CanFilter_10    (10 << 3)
#define CanFilter_11    (11 << 3)
#define CanFilter_12    (12 << 3)
#define CanFilter_13    (13 << 3)
#define CanFilter_14    (14 << 3)
#define CanFilter_15    (15 << 3)
#define CanFilter_16    (16 << 3)
#define CanFilter_17    (17 << 3)
#define CanFilter_18    (18 << 3)
#define CanFilter_19    (19 << 3)
#define CanFilter_20    (20 << 3)
#define CanFilter_21    (21 << 3)
#define CanFilter_22    (22 << 3)
#define CanFilter_23    (23 << 3)
#define CanFilter_24    (24 << 3)
#define CanFilter_25    (25 << 3)
#define CanFilter_26    (26 << 3)
#define CanFilter_27    (27 << 3)

#define CanFifo_0       (0 << 2)
#define CanFifo_1       (1 << 2)

#define Can_STDID       (0 << 1)
#define Can_EXTID       (1 << 1)

#define Can_DataType    (0 << 0)
#define Can_RemoteType  (1 << 0)

#define CAN_LINE_BUSY 0
#define CAN_SUCCESS   1
#define CAN_FIFO_SIZE 1024

typedef struct CAN_RxMessage
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
}CAN_RxBuffer;

uint8_t CAN_Init(CAN_HandleTypeDef* hcan, void (*pFunc)(CAN_RxBuffer*));
void CAN_Filter_Init(CAN_HandleTypeDef * hcan, uint8_t object_para,uint32_t Id,uint32_t MaskId);
void comm_can_transmit_extid(CAN_HandleTypeDef* hcan, uint32_t ExtId, uint8_t *pdata, uint8_t length);	   //拓展帧发送函数
void comm_can_transmit_stdid(CAN_HandleTypeDef* hcan, uint16_t StdId,uint8_t *pdata, uint8_t length);		   //标准帧发送函数

#ifdef __cplusplus
}
#endif

#endif
