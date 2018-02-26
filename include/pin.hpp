/*
 * pin.hpp
 * target: raspbian
 * ver1.0 2015/01/19
 *
 */
#ifndef PIN_HPP
#define PIN_HPP

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <thread>
#include <vector>
#include <iostream>
#include <mutex>
#include <errno.h>
#include <unistd.h>
#include "serial.hpp"
using namespace std;

char* board_name();

int baudRateSelect();

/*********Serial0*********/
class Serial0 : public Serial{
private:
	static SerialInterface *interface;
	int _setup(int baudrate,SerialInterface &interfaceArg,int parity,int wordLength);
	char writeValue[1],readValue[1];
	int returnValue;
	thread readerThread;
	mutex mtx;
public:
	Serial0();
	void reader();
	void writeChar(char value);
};

/**********SerialDevice*************/
class SerialDevice : public Serial{
private:
	static SerialInterface *interface;
	int _setup(int baudrate,SerialInterface &interfaceArg,int parity,int wordLength);
	bool readerTruth,rebootTruth; //���M�A�Đڑ���ON OFF
	int openflag,ioFailCount,ioFailLimit; //�Đڑ��֘A
	char dev_name[128]; //�f�o�C�X�̃p�X
	int br; //�{�[���[�g
	int lenR,lenW; //read�Awrite�̖߂��l
	int fd,returnValue; //�t�@�C��
	thread readerThread; //reader�Ŏg�p
	mutex mtx; //�����h�~

public:
	SerialDevice();
	char writeValue[8],readValue[8]; //�ꕶ�����M�A���M�p�ϐ�
	int openFile(); //�����ڑ��֐� �߂��l0�Ő���
	void filePath(char path[]); //�t�@�C���p�X�w���֐�
	void automaticConnection();  //USB�Đڑ��֐�
	void reader(); //���M�֐� �ʃX���b�h�Ŏg�p
	void readerOff(); //���M�֐�OFF���AUSB�Đڑ��֐��N���p
	void writeChar(char value); //�������z�֐�writeChar
	void writeString(char *value); //�������̑��M
	void rebootSelect(bool select,int limit); //�Đڑ���ONOFF�A�Đڑ��̎��s�񐔏���
	void readerSelect(bool select); //���M��ONOFF
};

char* board_name() {
	return (char*) "\n pin.cpp ver.1.2\n";
}
/*******BaudRate********/
int baudRateSelect(int baudrate) {
	if (baudrate == 9600) {
		cout << "baudrate : 9600" << endl;
		return B9600;
	} else if (baudrate == 115200) {
		cout << "baudrate : 115200" << endl;
		return B115200;
	} else if (baudrate == 230400) {
		cout << "baudrate : 230400" << endl;
		return B230400;
	} else if (baudrate == 460800) {
		cout << "baudrate : 460800" << endl;
		return B460800;
	} else if (baudrate == 500000) {
		cout << "baudrate : 500000" << endl;
		return B500000;
	} else if (baudrate == 576000) {
		cout << "baudrate : 576000" << endl;
		return B576000;
	} else if (baudrate == 921600) {
		cout << "baudrate : 921600" << endl;
		return B921600;
	}

	return 1;
}

/**********************************/
/*           Serial0              */
/**********************************/
SerialInterface *Serial0::interface;
Serial0::Serial0() :
		Serial() {
	returnValue = 0;
}
int Serial0::_setup(int baudrate, SerialInterface &interfaceArg, int parity,
		int wordLength) {
	cout << "Serial0:SETUP NOW.....";
	interface = &interfaceArg;
	interface->serialInterfaceSetup(this);
	readerThread = thread([this] {Serial0::reader();});
	readerThread.detach();
	cout << "Serial0:START" << endl;
	return returnValue;
}
void Serial0::reader() {
	while (1) {
		readValue[0] = fgetc(stdin);
		Serial0::interface->serialReadChar(readValue[0]);
	}
}
void Serial0::writeChar(char value) {
	writeValue[0] = value;
	fputc(writeValue[0], stdout);
	fflush(stdout);
}

/**********************************/
/*        SerialDevice            */
/**********************************/
SerialInterface *SerialDevice::interface;
SerialDevice::SerialDevice() :
		Serial() {
	lenR = 0, lenW = 0;
	fd = -1;
	returnValue = 0;
	openflag = 1;
	br = B9600;
	rebootTruth = true;
	readerTruth = true;
	ioFailCount = 0;
	ioFailLimit = 6;
	strcpy(dev_name, "/dev/ttyUSB0");
}
void SerialDevice::rebootSelect(bool select, int limit) {
	rebootTruth = select;
	ioFailLimit = limit;
}
void SerialDevice::readerSelect(bool select) {
	readerTruth = select;
}
void SerialDevice::filePath(char path[]) {
	strcpy(dev_name, path);
}
int SerialDevice::openFile() {
	fd = open(dev_name, O_RDWR);
	if (fd < 0) {
		//perror("openFile:OPEN FAIL");
		return 1;
	}
	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	tio.c_lflag = 0;
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_cc[VTIME] = 10;
	tio.c_cc[VMIN] = 1;
	cout << "IN : " << cfsetispeed(&tio, br) << endl;
	cout << "OUT : " << cfsetospeed(&tio, br) << endl;
	cout << "FLUSH : " << tcflush(fd, TCIFLUSH) << endl;
	cout << "ATTR : " << tcsetattr(fd, TCSANOW, &tio) << endl;

	return tcsetattr(fd, TCSANOW, &tio);
}
int SerialDevice::_setup(int baudrate, SerialInterface &interfaceArg,
		int parity, int wordLength) {
	cout << "SerialDevice:SETUP NOW....." << endl;
	br = baudRateSelect(baudrate);
	returnValue = SerialDevice::openFile();

	interface = &interfaceArg;
	interface->serialInterfaceSetup(this);

	if (readerTruth) {
		readerThread = thread([this] {SerialDevice::reader();});
		readerThread.detach();
	} else {
		cout << "reader OFF" << endl;
		readerThread = thread([this] {SerialDevice::readerOff();});
		readerThread.detach();
	}

	cout << "SerialDevice:START" << endl;

	return returnValue;
}
void SerialDevice::reader() {
	while (1) {
		lenR = read(fd, readValue, 1);
		if (lenR == 0) {
			perror("reader:NOT RECEIVED");
			ioFailCount++;
		} else if (lenR < 0) {
			perror("reader:ERROR");
			ioFailCount++;
		} else {
			//cout << readValue[0]; //���M�l
			SerialDevice::interface->serialReadChar(readValue[0]);
		}
		if (rebootTruth) {
			SerialDevice::automaticConnection();
		}
	}
}
void SerialDevice::readerOff() {
	while (1) {
		if (rebootTruth) {
			SerialDevice::automaticConnection();
		}
	}
}
void SerialDevice::automaticConnection() {
	if (ioFailCount >= ioFailLimit) {
		cout << "SerialDevice:RECONNECTING..." << ioFailCount << endl;
		close(fd);
		openflag = 1;
		do {
			if (SerialDevice::openFile() == 0) {
				openflag = 0;
				cout << "END" << endl;
			}
		} while (openflag == 1);
		ioFailCount = 0;
	}
}
void SerialDevice::writeString(char *value) {
	mtx.lock();
	for (int i = 0; value[i] != '\0'; i++) {
		writeChar(value[i]);
	}
	mtx.unlock();
}
void SerialDevice::writeChar(char value) {
	writeValue[0] = value;
	lenW = write(fd, writeValue, 1);
	if (lenW == 0) {
		//perror("writeChar:NOT TRANSMITTED");
		ioFailCount++;
	} else if (lenW < 0) {
		//perror("writeChar:ERROR");
		ioFailCount++;
	}
}

#endif//PIN_HPP
