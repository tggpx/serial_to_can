#pragma once
#include <serial_to_can/RT_COM.h>
#include <boost/shared_ptr.hpp>

//如果CAN消息中的ch部分为0xff，则下面是配置用ID；否则，为普通ID-------------------------//
#define CAN_RESET_ID		0x01FFFEc0		//ID初始化
#define CAN_ABOM_ID			0x01fffEb0		//自动总线管理
#define CAN_ART_ID			0x01fffEa0		//自动重发
#define CAN_FILTER_BASE_ID	0x01fffEe0		//过滤器
#define CAN_BAUD_ID			0x01fffEd0		//CAN波特率
#define CAN_INIT_FLASH_ID	0x01fffEff		//恢复出厂设置

#define CAN_FILTER_ID(x)	(CAN_FILTER_BASE_ID + (x))	//过滤器ID，共14个可配置过滤器
#define CAN_FILTER_ENABLE   0x80000000		//使能过滤器

#define MAX_CAN_BAUD		1000000			//最大支持的CAN波特率
#define CAN_CONFIG_CHANNEL	0xff			//CAN的配置通道
#define FILTER_ID_MASK		0x0000000f		//过滤器ID掩码
#define ENABLE_MASK			0x80000000		//使能掩码
#define DISABLE_MASK		0				//失能掩码

typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;		//CAN的两种数据格式--标准帧和扩展帧
typedef enum {DATA_FRAME = 0, REMOTE_FRAME}         CAN_FRAME;		//CAN的两种帧格式--数据帧和远程帧
typedef enum
{
	CAN_ERR_TRANS = 0,		//传输错误
	CAN_OK,					//没有错误
	CAN_ERR_ACK,			//没有返回应答
	CAN_ERR_CHANNEL,		//CHANNEL错误
	CAN_ERR_PARAM,			//参数错误
	CAN_ERR_TIMEOUT			//接收超时
}CAN_ERROR;

//CAN消息结构体
typedef struct  {
	DWORD id;                 // 29 位ID                             
	BYTE  data[8];            // 数据域                                      
	BYTE  len;                // 数据域长度
	BYTE  ch;                 // 通道，在这里用于区分配置用数据包(0xff)和普通数据包(其他)
	BYTE  format;             // 数据格式，标准帧和扩展帧
	BYTE  type;               // 帧格式，数据帧和远程帧
}  CAN_msg;

class RT_CAN :
	public RT_COM
{
public:
	RT_CAN(void);
	~RT_CAN(void);

	CAN_ERROR SetConfigValue(DWORD dwID, void *pValueBuf, DWORD *pdwLen, DWORD dwTimeOut);
	CAN_ERROR GetConfigValue(DWORD dwID, void *pValueBuf, DWORD *pdwLen, DWORD dwTimeOut);
	CAN_ERROR SendCANMessage(CAN_msg *pMsg, DWORD dwTimeout = 0xFFFFFFFF);
	CAN_ERROR RecvCANMessage(CAN_msg *pMsg, DWORD dwTimeout = 0xFFFFFFFF);

	virtual void OnRecvPackage(const BYTE * byBuf, DWORD dwLen);
	virtual DWORD GetConfigDataLen(DWORD dwID);

protected:
	CAN_ERROR SendConfigData(DWORD dwID, void *pBufData, DWORD dwBufLen, DWORD dwTimeout);
	CAN_ERROR RequestConfigData(DWORD dwID, DWORD dwTimeout);

	CAN_msg m_CANConfigMsg;
	CAN_msg m_CANRecvMsg;
};
typedef boost::shared_ptr<RT_CAN> RTCanSharedPtr;