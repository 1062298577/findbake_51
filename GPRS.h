#ifndef __GPRS_H__
#define __GPRS_H__
#define uchar unsigned char
#define uint8 unsigned int

void	gprs_start();//启动GPRS模块

void	setRevBuf(uchar b);//设置接收的数据

void	sendData(uchar buf[]);//发送数据

void 	GPS_TxByte(uchar c);	//发送字节

void	GPS_TxString(uchar code *p);//GPS发送字符串

void 	delay(int i);	//延时

unsigned char[]	getCcid();		//获取CCID号   

#endif
