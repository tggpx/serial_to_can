#include <serial_to_can/RT_CAN.h>

RT_CAN::RT_CAN(void)
{
	//m_hConfigEvnet = CreateEvent(NULL, FALSE, FALSE, NULL);
	//m_hRecvMsgEvnet = CreateEvent(NULL, FALSE, FALSE, NULL);
}

RT_CAN::~RT_CAN(void)
{
#if 0
	if(m_hConfigEvnet != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hConfigEvnet);
		m_hConfigEvnet = INVALID_HANDLE_VALUE;
	}

	if(m_hRecvMsgEvnet != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hRecvMsgEvnet);
		m_hRecvMsgEvnet = INVALID_HANDLE_VALUE;
	}
#endif
}

/*******************************************************************************
* Function Name  : OnRecvPackage
* Description    : 在串口的接收线程内被调用，表示接收到一个完整的CAN数据包
* Input          : byBuf: 接收到的数据
                   dwLen: 接收到的数据长度
* Output         : None
* Return         : None
*******************************************************************************/
void RT_CAN::OnRecvPackage(const BYTE * byBuf, DWORD dwLen)
{
	CAN_msg* msg;
	msg = (CAN_msg *)byBuf;
	if(dwLen == sizeof(CAN_msg))
	{
		if (msg->ch == CAN_CONFIG_CHANNEL)
		{
			m_CANConfigMsg = *msg;
			//SetEvent(m_hConfigEvnet);
            sleep(1);
		} 
		else 
		{		
			m_CANRecvMsg = *msg;
			//SetEvent(m_hRecvMsgEvnet);
			RT_COM::OnRecvPackage(byBuf, dwLen);
		}
	}
}

/*******************************************************************************
* Function Name  : SendConfigData
* Description    : 发送配置信息
* Input          : dwID: 数据包ID
				   pBufData: 数据
				   dwBufLen: 数据长度
				   dwTimeout: 超时时间
* Output         : None
* Return         : 错误代码
*******************************************************************************/
CAN_ERROR RT_CAN::SendConfigData(DWORD dwID, void *pBufData, DWORD dwBufLen, DWORD dwTimeout)
{
	CAN_msg msg;

	if((pBufData == NULL) || (dwBufLen > 8))
		return CAN_ERR_PARAM;

	msg.id = dwID;
	msg.ch = CAN_CONFIG_CHANNEL;
	memcpy(msg.data, pBufData, dwBufLen);
	msg.format = EXTENDED_FORMAT;
	msg.len = (BYTE)dwBufLen;
	msg.type = DATA_FRAME;

    //ResetEvent(m_hConfigEvnet);
	if(WritePackage((BYTE*)&msg, sizeof(msg)))
	{
		//if(WAIT_TIMEOUT == WaitForSingleObject(m_hConfigEvnet,dwTimeout))
		//	return CAN_ERR_ACK;
		//else
			return CAN_OK;
	}
	else
	{
		return CAN_ERR_TRANS;
	}
}

/*******************************************************************************
* Function Name  : RequestConfigData
* Description    : 获取配置信息
* Input          : dwID: 数据包ID
				   dwTimeout: 超时时间
* Output         : None
* Return         : 错误代码
*******************************************************************************/
CAN_ERROR RT_CAN::RequestConfigData(DWORD dwID, DWORD dwTimeout)
{
	CAN_msg msg;
	msg.id = dwID;
	msg.ch = CAN_CONFIG_CHANNEL;
	msg.format = EXTENDED_FORMAT;
	msg.type = REMOTE_FRAME;
	msg.len = 8;

	//ResetEvent(m_hConfigEvnet);
	if(!WritePackage((BYTE *)&msg, sizeof(CAN_msg)))
		return CAN_ERR_TRANS;

	//等待收到数据
	//if(WAIT_TIMEOUT == WaitForSingleObject(m_hConfigEvnet,dwTimeout))
	//	return CAN_ERR_ACK;

	return CAN_OK;
}

/*******************************************************************************
* Function Name  : GetConfigDataLen
* Description    : 获取配置数据包数据域长度
* Input          : dwID: 数据包ID
* Output         : None
* Return         : 配置用数据包数据域长度
*******************************************************************************/
DWORD RT_CAN::GetConfigDataLen(DWORD dwID)
{
	DWORD RequesetBufLen = 0;
	switch(dwID)
	{
    case CAN_ART_ID:
	case CAN_ABOM_ID:
        RequesetBufLen = sizeof(BYTE);
    	break;
	case CAN_BAUD_ID:
	case CAN_INIT_FLASH_ID:
		RequesetBufLen = sizeof(DWORD);
		break;
	default:
		if((dwID >= CAN_FILTER_BASE_ID) && (dwID <= (CAN_FILTER_BASE_ID + 13))) //Fileter
			RequesetBufLen = (sizeof(DWORD) * 2);
		break;
	}
	return RequesetBufLen;
}

/*******************************************************************************
* Function Name  : GetConfigValue
* Description    : 获取配置数据信息
* Input          : dwID: 数据包ID
				   pValueBuf: 数据
				   pdwLen: 数据长度
				   dwTimeOut: 超时时间
* Output         : None
* Return         : 错误信息
*******************************************************************************/
CAN_ERROR RT_CAN::GetConfigValue(DWORD dwID, void *pValueBuf, DWORD *pdwLen, DWORD dwTimeOut)
{	
    CAN_ERROR CAN_err = CAN_ERR_PARAM;

	DWORD RequesetBufLen = GetConfigDataLen(dwID);

	if(pValueBuf == NULL)	
		return CAN_ERR_PARAM;

	if((*pdwLen >= RequesetBufLen) && RequesetBufLen)
	{
        *pdwLen = RequesetBufLen;
        CAN_err = RequestConfigData(dwID, dwTimeOut);
		if(CAN_err == CAN_OK)
		{
			if(m_CANConfigMsg.id == dwID)
            {
				memcpy(pValueBuf, m_CANConfigMsg.data, RequesetBufLen);
                CAN_err = CAN_OK;
            }
			else
				CAN_err = CAN_ERR_TRANS;
		}
	}

	return CAN_err;
}

/*******************************************************************************
* Function Name  : SetConfigValue
* Description    : 发送配置数据信息
* Input          : dwID: 数据包ID
				   pValueBuf: 数据
				   pdwLen: 数据长度
				   dwTimeOut: 超时时间
* Output         : None
* Return         : 错误信息
*******************************************************************************/
CAN_ERROR RT_CAN::SetConfigValue(DWORD dwID, void *pValueBuf, DWORD *pdwLen, DWORD dwTimeOut)
{
	DWORD RequesetBufLen = GetConfigDataLen(dwID);

	if(pValueBuf == NULL)	
		return CAN_ERR_PARAM;

	if((*pdwLen >= RequesetBufLen) && RequesetBufLen)
    {
    	*pdwLen = RequesetBufLen;
		return SendConfigData(dwID, pValueBuf, RequesetBufLen, dwTimeOut);
    }
	return CAN_ERR_PARAM;		
}

/*******************************************************************************
* Function Name  : SendCANMessage
* Description    : 发送CAN消息
* Input          : pMsg: CAN消息数据包
				   dwTimeOut: 超时时间
* Output         : None
* Return         : 错误信息
*******************************************************************************/
CAN_ERROR RT_CAN::SendCANMessage(CAN_msg *pMsg, DWORD dwTimeout)
{
	if(pMsg == NULL)
		return CAN_ERR_PARAM;

	if(pMsg->ch == CAN_CONFIG_CHANNEL)
		return CAN_ERR_CHANNEL;
	
	if(WritePackage((BYTE *)pMsg, sizeof(CAN_msg), dwTimeout) == 0)
		return CAN_ERR_TRANS;

	return CAN_OK;
}

/*******************************************************************************
* Function Name  : RecvCANMessage
* Description    : 接收CAN消息
* Input          : dwTimeOut: 超时时间
* Output         : pMsg: CAN消息数据包
* Return         : 错误信息
*******************************************************************************/
CAN_ERROR RT_CAN::RecvCANMessage(CAN_msg *pMsg, DWORD dwTimeout)
{
	if(pMsg == NULL)
		return CAN_ERR_PARAM;

	//if(WAIT_TIMEOUT == WaitForSingleObject(m_hRecvMsgEvnet, dwTimeout))
	//	return CAN_ERR_TIMEOUT;
	
	*pMsg = m_CANRecvMsg;
	return CAN_OK;
}