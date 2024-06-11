/**
 * @file drive_can.c
 * @author Yang Jianyi 
 * @brief 1)can底层驱动文件，使用该文件，需要在cubeMX中配置好can硬件(参考大疆电机所需的CAN的配置)，并在main.c中调用CAN_Init函数进行初始化
 *        调用CAN_Filter_Init函数进行滤波器配置，comm_can_transmit_extid或comm_can_transmit_stdid函数进行can数据发送
 * 
 *        2)该文件使用双fifo的接收方式，当接收到数据时，会调用CAN_RxFifo0MsgPendingCallback或CAN_RxFifo1MsgPendingCallback函数。
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "drive_can.h"

static void (*pCAN1_RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN2_RxCpltCallback)(CAN_RxBuffer *);


/**
 * @brief CAN接收滤波器初始化
 * 
 * @param hcan can句柄
 * @param object_para 滤波器参数配置，滤波器序号|FIFO绑定|ID类型(帧类型)|数据类型
 * @param Id 
 * @param MaskId ID掩码 
 */
void CAN_Filter_Init(CAN_HandleTypeDef * hcan, uint8_t object_para,uint32_t Id,uint32_t MaskId) 
{
    CAN_FilterTypeDef  CAN_FilterInitStructure;
	/* Check the parameters */
	assert_param(hcan != NULL);

	/* Communication frame */
	if( (object_para&0x02))     /*拓展帧or标准帧*/
	{
        CAN_FilterInitStructure.FilterIdHigh         = Id<<3<<16;                       /* 掩码后ID的高16bit */
        CAN_FilterInitStructure.FilterIdLow          = Id<<3| ((object_para&0x03)<<1);  /* 掩码后ID的低16bit */
        CAN_FilterInitStructure.FilterMaskIdHigh     = MaskId<<3<<16;                   /* ID掩码值高16bit */
        CAN_FilterInitStructure.FilterMaskIdLow      = MaskId<<3| ((object_para&0x03)<<1);;   /* ID掩码值低16bit */
	}
	else/* Other frame */
	{
        CAN_FilterInitStructure.FilterIdHigh         = Id<<5;                           /* 掩码后ID的高16bit */
        CAN_FilterInitStructure.FilterIdLow          = ((object_para&0x03)<<1);         /* 掩码后ID的低16bit */
        CAN_FilterInitStructure.FilterMaskIdHigh     = MaskId<<5;                       /* ID掩码值高16bit */
        CAN_FilterInitStructure.FilterMaskIdLow      = ((object_para&0x03)<<1);;        /* ID掩码值低16bit */
	}

    CAN_FilterInitStructure.FilterBank           = object_para>>3;                  /* 滤波器序号*/
    CAN_FilterInitStructure.FilterFIFOAssignment = (object_para>>2)&0x01;           /* 滤波器绑定FIFO 0 */
    CAN_FilterInitStructure.FilterActivation     = ENABLE;                          /* 使能滤波器 */
    CAN_FilterInitStructure.FilterMode         = CAN_FILTERMODE_IDMASK;             /* 滤波器模式，设置ID掩码模式 */
    CAN_FilterInitStructure.FilterScale        = CAN_FILTERSCALE_32BIT;             /* 32位滤波 */
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;                              /* 过滤器开始组别，单can芯片无意义 */
    
    if(HAL_CAN_ConfigFilter(hcan, &CAN_FilterInitStructure)!=HAL_OK)
    {
		/* Filter configuration Error */
		Error_Handler();
	}

}


uint8_t CAN_Init(CAN_HandleTypeDef* hcan, void (*pFunc)(CAN_RxBuffer*))
{
    assert_param(hcan != NULL);
	
    if (HAL_CAN_Start(hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
  
	if(hcan->Instance == CAN1)
	{
		pCAN1_RxCpltCallback = pFunc;
		return SUCCESS;
	}
	else if(hcan->Instance == CAN2)
	{
		pCAN2_RxCpltCallback = pFunc;
		return SUCCESS;
	}
	else
		return ERROR;
}


/**
 * @brief hal库CAN_FIFO0回调函数
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxBuffer CAN_RxBuffer;
    if(hcan == &hcan1)
    {
       if(HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&CAN_RxBuffer.header,CAN_RxBuffer.data)==HAL_ERROR){};
		pCAN1_RxCpltCallback(&CAN_RxBuffer);
    }

    if(hcan == &hcan2)
    {
       if(HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO0,&CAN_RxBuffer.header,CAN_RxBuffer.data)==HAL_ERROR){};
		pCAN2_RxCpltCallback(&CAN_RxBuffer);
    }
}


/**
 * @brief hal库CAN_FIFO1回调函数
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static CAN_RxBuffer CAN_RxBuffer;
    if(hcan == &hcan1)
    {
       if(HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&CAN_RxBuffer.header,CAN_RxBuffer.data)==HAL_ERROR){};
		pCAN1_RxCpltCallback(&CAN_RxBuffer);
    }

    if(hcan == &hcan2)
    {
       if(HAL_CAN_GetRxMessage(hcan,CAN_FILTER_FIFO1,&CAN_RxBuffer.header,CAN_RxBuffer.data)==HAL_ERROR){};
		pCAN2_RxCpltCallback(&CAN_RxBuffer);
    }
}


/**
 * @brief can命令发送函数，拓展帧
 * @param hcan 使用哪个can，hcan1 or hcan2
 * @param controller_id 使用的id
 * @param pdata 发送的数据结构体
 * @param 数据长度
 * @return NULL
*/
void comm_can_transmit_extid(CAN_HandleTypeDef* hcan, uint32_t ExtId, uint8_t *pdata, uint8_t length)
{
    CAN_TxHeaderTypeDef			TxHeader;   //can数据发送句柄
	uint32_t tx_mailbox = 0;
	
    if (length > 8)
    {
        length = 8;
    }

    TxHeader.ExtId = ExtId; //VESC电调id
    TxHeader.IDE = CAN_ID_EXT;     //使用拓展帧
    TxHeader.RTR = CAN_RTR_DATA;   
	TxHeader.DLC = length;     //数据长度
    TxHeader.TransmitGlobalTime = DISABLE;  //不发送标记时间
    // while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)    //等待发送邮箱空   //此处改为在can发送任务中进行处理
        // HAL_Delay(1);                                       //防止炸邮箱
    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, pdata, &tx_mailbox)!= HAL_OK)
    {
        Error_Handler();
    }
}


/**
 * @brief can命令发送函数，标准帧
 * @param hcan 使用哪个can，hcan1 or hcan2
 * @param controller_id 使用的id
 * @param pdata 发送的数据结构体
 * @param 数据长度
 * @return NULL
*/
void comm_can_transmit_stdid(CAN_HandleTypeDef* hcan, uint16_t StdId,uint8_t *pdata, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t tx_mailbox = 0;
    if(length > 8)
        length = 8;
    
    TxHeader.StdId = StdId;         //标准标识符
    TxHeader.IDE = CAN_ID_STD;      //帧模式(标准帧或拓展帧)
    TxHeader.RTR = CAN_RTR_DATA;    //帧类型(数据帧或远程帧)
    TxHeader.DLC = length;          //数据长度
    TxHeader.TransmitGlobalTime = DISABLE;  //不发送标记时间

	// while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);    //等待发送邮箱空  //此处改为在can发送任务中进行处理
        // HAL_Delay(1);                                       //防止炸邮箱
    if (HAL_CAN_AddTxMessage(hcan,&TxHeader,pdata,&tx_mailbox) != HAL_OK)
    {
        Error_Handler();
    }
}
