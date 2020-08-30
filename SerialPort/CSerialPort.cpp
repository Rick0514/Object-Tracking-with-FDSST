#include "CSerialPort.h"
#include <algorithm>
#include <tchar.h>

CSerialPort::CSerialPort()
{
	mComm = INVALID_HANDLE_VALUE;
}

CSerialPort::~CSerialPort()
{
	closePort();
}

bool CSerialPort::initPort(UINT portNo)
{
	// whether the port is open
	if (!openPort(portNo))	return false;

	DCB dcb;
	GetCommState(mComm, &dcb);
	dcb.BaudRate = CBR_9600;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;

	// config the port params
	if (!SetCommState(mComm, &dcb))
		return false;

	COMMTIMEOUTS commTimeouts;
	commTimeouts.ReadIntervalTimeout = 0;
	commTimeouts.ReadTotalTimeoutConstant = 0;
	commTimeouts.ReadTotalTimeoutMultiplier = 0;
	commTimeouts.WriteTotalTimeoutMultiplier = 0;
	commTimeouts.WriteTotalTimeoutConstant = 0;

	if (!SetCommTimeouts(mComm, &commTimeouts))
		return false;

	// clear port buff
	PurgeComm(mComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

	return true;

}

bool CSerialPort::openPort(UINT portNo)
{

	char portName[10];
	sprintf_s(portName, 10, "COM%d", portNo);

	mComm = CreateFile(portName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,	// try sync mode firstly
		NULL);

	if (mComm == INVALID_HANDLE_VALUE)
		return false;

	return true;
}

void CSerialPort::closePort()
{
	if (mComm != INVALID_HANDLE_VALUE)
	{
		CloseHandle(mComm);
		mComm = INVALID_HANDLE_VALUE;
	}
}

DWORD CSerialPort::getBytesNumInCom()
{
	DWORD dwError = 0;
	COMSTAT comstat;
	memset(&comstat, 0, sizeof(COMSTAT));

	DWORD byteInQue = 0;
	ClearCommError(mComm, &dwError, &comstat);
	byteInQue = comstat.cbInQue;
	return byteInQue;
}


bool CSerialPort::readData(unsigned char *buff, unsigned int len)
{
	DWORD byteLen = min(getBytesNumInCom(), len);
	bool readFlag = true;
	DWORD bytesRead = 0;

	readFlag = ReadFile(mComm, buff, byteLen, &bytesRead, NULL);

	if (byteLen == 0)
	{
		return false;
	}

	if (!readFlag)
	{
		DWORD dwError = GetLastError();
		PurgeComm(mComm, PURGE_RXCLEAR | PURGE_RXABORT);
		memset(buff, 0, len);
		return false;
	}
	return true;
}

void CSerialPort::writeData(unsigned char* dat, unsigned int len)
{
	bool writeFlag = true;
	DWORD bytesWritten = 0;
	if (mComm == INVALID_HANDLE_VALUE)	return;

	writeFlag = WriteFile(mComm, dat, len, &bytesWritten, NULL);

	if (!writeFlag)
	{
		DWORD dwError = GetLastError();
		PurgeComm(mComm, PURGE_TXCLEAR | PURGE_TXABORT);
	}

}


// comm operation

bool CSerialPort::cmp(string s1, string s2)
{
	if (atoi(s1.substr(3).c_str()) < atoi(s2.substr(3).c_str()))//升序
		return true;
	else
		return false;
}

void CSerialPort::getComList(std::vector<string> &comList)
{
	HKEY hkey;
	int result;
	int i = 0;

	result = RegOpenKeyEx(HKEY_LOCAL_MACHINE,
		_T("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
		NULL,
		KEY_READ,
		&hkey);

	if (ERROR_SUCCESS == result)   //   打开串口注册表   
	{
		TCHAR portName[0x100], commName[0x100];
		DWORD dwLong, dwSize;
		do
		{
			dwSize = sizeof(portName) / sizeof(TCHAR);
			dwLong = dwSize;
			result = RegEnumValue(hkey, i, portName, &dwLong, NULL, NULL, (LPBYTE)commName, &dwSize);
			if (ERROR_NO_MORE_ITEMS == result)
			{
				//   枚举串口
				break;   //   commName就是串口名字"COM4"
			}

			comList.push_back(commName);
			i++;
		} while (1);

		RegCloseKey(hkey);
	}
	sort(comList.begin(), comList.end(), cmp);
}






