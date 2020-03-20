#ifndef __RT_COM_H
#define __RT_COM_H

#include "serial/serial.h"
#include <thread>
#include <unistd.h>
using namespace std;

#define BYTE unsigned char
#define DWORD unsigned int
#define BOOL BYTE
#define TRUE 1
#define FALSE 0
#define UINT64 unsigned long long int
#define LPVOID void*

typedef void (*RxCallBackFunc)(void *pParam, const BYTE * byBuf, DWORD dwLen);

class RT_COM
{
public:
	RT_COM(DWORD dwMaxPackageSize = 128, BOOL UsePackage = TRUE);
	~RT_COM();
	BOOL Open(string dwPortNo, DWORD dwBaud);
	void Close();

	void SetRxPackageCallBack(RxCallBackFunc func, void *pParam);
	//void SetRxBufCallBack(RxCallBackFunc func, void *pParam);
	DWORD GetSysErrCode(){ return m_dwSysErrCode;}
	DWORD WritePackage(BYTE * byBuf, DWORD dwLen, DWORD dwTimeout = 0xFFFFFFFF);
	DWORD WriteBuf(const BYTE *byBuf, DWORD dwLen, DWORD dwTimeout = 0xFFFFFFFF);

	BOOL IsOpened(){ return m_IsOpened; }
	//DWORD GetBaudRate(){ return m_DCB.BaudRate;}
	string GetPortNo(){ return m_PortNo;}

	//UINT64 GetTotalRxByteCount(){ return m_u64Total_Rx_Bytes; }
	//UINT64 GetTotalTxByteCount(){ return m_u64Total_Tx_Bytes; }
	UINT64 GetTotalRxPackageCount(){ return m_u64Total_Rx_Packages; }
	UINT64 GetTotalTxPackageCount(){ return m_u64Total_Tx_Packages; }
    UINT64 GetTotalRxLostPackageCount(){ return m_u64Total_Rx_LostPackages; }
	void ResetCounters();

    void EnablePackage(BOOL flag = TRUE) {m_bUsePackage = flag;}
    BOOL IsUsePackage() {return m_bUsePackage;}

protected:
	virtual void OnRecvPackage(const BYTE * byBuf, DWORD dwLen);
	//virtual void OnRecvBuf(const BYTE * byBuf, DWORD dwLen);

private:
	//UINT64 m_u64Total_Rx_Bytes;
	UINT64 m_u64Total_Rx_Packages;
	UINT64 m_u64Total_Tx_Packages;
    //UINT64 m_u64Total_Tx_Bytes;
	UINT64 m_u64Total_Rx_LostPackages;

	void *m_pRxPackageCallBackParam;
	void *m_pRxPBufCallBackParam;
	RxCallBackFunc m_RxPackageCallBack;
	//RxCallBackFunc m_RxBufCallBack;

	BYTE m_FrameHead, m_FrameTail, m_FrameCtrl;
	
	BOOL m_RecvThreadStopFlag;
	BOOL m_IsOpened;
	DWORD m_MaxPackageSize;
	BYTE * m_pRxBuf;
	BYTE * m_pTxBuf;
	DWORD m_RxPackageDataCount;	
	string m_PortNo;
	DWORD m_dwSysErrCode;
    BOOL m_bUsePackage;
    serial::Serial *m_Serial;

    thread m_RecvThread;
	void RecvThreadLoop();
	static DWORD RecvThread(LPVOID lpParameter)
	{
		RT_COM *pGComm = (RT_COM *)lpParameter;
		pGComm->RecvThreadLoop();
		return 0;
	}

	BOOL AnalyzePackage(BYTE data);
} ;


#endif