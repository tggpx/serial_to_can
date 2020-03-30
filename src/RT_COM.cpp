#include <serial_to_can/RT_COM.h>

RT_COM::RT_COM(DWORD MaxPackageSize, BOOL UsePackage)
{
	m_RxPackageCallBack = nullptr;
	m_RecvThreadStopFlag = FALSE;

    m_bUsePackage = UsePackage;

	m_FrameHead = 0xAA;
	m_FrameTail = 0x55;
	m_FrameCtrl = 0xA5;

	m_IsOpened = FALSE;
	m_MaxPackageSize =  MaxPackageSize * 2 +2;

	m_pRxBuf = new BYTE[m_MaxPackageSize];
	m_pTxBuf = new BYTE[m_MaxPackageSize];
	m_RxPackageDataCount = 0;

	m_dwSysErrCode = 0;
	ResetCounters();
}

RT_COM::~RT_COM()
{
	Close();
    if(m_pRxBuf)
    {
		delete [] m_pRxBuf;
    	m_pRxBuf = nullptr;
    }
    if(m_pTxBuf)
    {
        delete [] m_pTxBuf;
        m_pTxBuf = nullptr;
    }
}

void RT_COM::ResetCounters()
{
	//m_u64Total_Rx_Bytes = 0;
	//m_u64Total_Tx_Bytes = 0;
	m_u64Total_Rx_Packages = 0;
	m_u64Total_Tx_Packages = 0;
    m_u64Total_Rx_LostPackages = 0;
}


BOOL RT_COM::Open(string dwPortNo, DWORD dwBaud)
{
	if(m_IsOpened)
	{
		return TRUE;
	}
	m_PortNo = dwPortNo;

    m_Serial = new serial::Serial(dwPortNo, dwBaud, serial::Timeout::simpleTimeout(1000));
    m_RecvThreadStopFlag = FALSE;
	m_RecvThread = thread(RecvThread, this);
	m_IsOpened = TRUE;
	return TRUE;
}

void RT_COM::Close()
{
    m_RecvThreadStopFlag = TRUE;
	m_RecvThread.join();
    m_Serial->close();
    delete m_Serial;
    m_Serial = nullptr;
	m_IsOpened = FALSE;
}

void RT_COM::SetRxPackageCallBack(RxCallBackFunc func, void *pParam)
{
	m_pRxPackageCallBackParam = pParam;
	m_RxPackageCallBack = func;
}

void RT_COM::OnRecvPackage(const BYTE * byBuf, DWORD dwLen)
{
	if(m_RxPackageCallBack)
		m_RxPackageCallBack(m_pRxPackageCallBackParam, byBuf, dwLen);

	m_u64Total_Rx_Packages ++;
}

#if 0
void RT_COM::OnRecvBuf(const BYTE * byBuf, DWORD dwLen)
{
	if(m_RxBufCallBack)
		m_RxBufCallBack(m_pRxPBufCallBackParam, byBuf, dwLen);

	m_u64Total_Rx_Bytes += dwLen;
}
#endif

void RT_COM::RecvThreadLoop()
{
    std::vector<uint8_t> buff;
    while(1)
    {
        if (m_RecvThreadStopFlag)
			return;
        if(true == m_Serial->waitReadable())
        {
            usleep(50000);
            m_Serial->read(buff, m_Serial->available());
            for(uint8_t n: buff)
            {
                if(AnalyzePackage(n))
                {
                    OnRecvPackage(m_pRxBuf, m_RxPackageDataCount);
                }
            }
            buff.clear();
        }
    }
}

BOOL RT_COM::AnalyzePackage(BYTE data)
{
	static BYTE USART_LastByte = 0;
	static bool USART_BeginFlag = FALSE;
	static bool USART_CtrlFlag = FALSE;
	static DWORD USART_RevOffset = 0;
	static BYTE CheckSum = 0;

	if(((data == m_FrameHead) && (USART_LastByte == m_FrameHead)) || (USART_RevOffset > m_MaxPackageSize))
	{
    	if(USART_RevOffset < 17 && USART_RevOffset > 0)
        {
            m_u64Total_Rx_LostPackages ++;
        }
		//RESET
		USART_RevOffset = 0;
		USART_BeginFlag = TRUE;
		USART_LastByte = data ;
		return FALSE;
	}
	if( (data==m_FrameTail)&&(USART_LastByte==m_FrameTail)&&USART_BeginFlag )
	{
		USART_RevOffset--;
		m_RxPackageDataCount = USART_RevOffset - 1;
		CheckSum -= m_FrameTail;
		CheckSum -= m_pRxBuf[m_RxPackageDataCount];
		USART_LastByte = data;
		USART_BeginFlag = FALSE;
		if(CheckSum == m_pRxBuf[m_RxPackageDataCount])
		{
			CheckSum = 0;
			return TRUE;
		}
        m_u64Total_Rx_LostPackages ++;

		CheckSum = 0;
		return FALSE;
	}
	USART_LastByte = data ;
	if(USART_BeginFlag)
	{
		if(USART_CtrlFlag)
		{
			m_pRxBuf[USART_RevOffset++] = data;
			CheckSum += data;
			USART_CtrlFlag = FALSE;
			USART_LastByte = m_FrameCtrl;
		}
		else if(data == m_FrameCtrl)
		{
			USART_CtrlFlag = TRUE;
		}
		else
		{
			m_pRxBuf[USART_RevOffset++] = data;
			CheckSum += data;
		}
	}

	return FALSE;
}

DWORD RT_COM::WritePackage(BYTE * buf, DWORD len, DWORD dwTimeout)
{

	DWORD i;
	BYTE *pBuf;
	BYTE CheckSum = 0;

	if (buf == nullptr)
	{
		return FALSE;
	}

 	if(m_bUsePackage == FALSE)
    	return 0;

	pBuf = m_pTxBuf;
	*pBuf++ = m_FrameHead;
	*pBuf++ = m_FrameHead;

	for (i=0 ; i < len ; i++)
	{
		if ((buf[i] == m_FrameCtrl)|| (buf[i] == m_FrameHead) || (buf[i] == m_FrameTail))
		{
			*pBuf++ = m_FrameCtrl;
		}
		*pBuf++ = buf[i];
		CheckSum += buf[i];
	}

	if ((CheckSum == m_FrameCtrl) || (CheckSum == m_FrameHead) || (CheckSum == m_FrameTail))
	{
		*pBuf++ = m_FrameCtrl;
	}
	*pBuf++ = CheckSum;

	//Send Tail USART_FRAMETAIL USART_FRAMETAIL
	*pBuf++ = m_FrameTail;
	*pBuf++ = m_FrameTail;

	len =  pBuf - m_pTxBuf;

	DWORD nw = WriteBuf(m_pTxBuf, len, dwTimeout);

	if(nw) m_u64Total_Tx_Packages ++;

	return nw;
}

DWORD RT_COM::WriteBuf(const BYTE *byBuf, DWORD dwLen, DWORD dwTimeout)
{
	DWORD nw = 0;

	if(byBuf == nullptr)
		return 0;

	if(!IsOpened())
		return 0;

	nw = m_Serial->write(byBuf, dwLen);

	//m_u64Total_Tx_Bytes += nw;
	return nw;
}
