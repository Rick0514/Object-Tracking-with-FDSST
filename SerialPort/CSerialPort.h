#pragma once
#ifndef SERIALPORT_H
#define SERIALPORT_H
#include <Windows.h>
#include <string>
#include <iostream>
#include <vector>
#include <stdio.h>
using namespace std;

class CSerialPort
{

private:

	HANDLE mComm;

public:
	CSerialPort();
	~CSerialPort();

	bool initPort(UINT portNo);

	void writeData(unsigned char* dat, unsigned int len);

	DWORD getBytesNumInCom();
	bool readData(unsigned char* buff, unsigned int len);

	void static getComList(vector<string> &comList);
	

private:

	bool openPort(UINT portNo);
	void closePort();
	//static UINT WINAPI listenThread(void* params);

	bool static cmp(string s1, string s2);

};

#endif // !SERIALPORT_H
