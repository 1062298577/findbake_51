#ifndef __GPRS_H__
#define __GPRS_H__

void	GPRS_StartUp();//启动GPRS模块

void	setRevBuf(uchar b);//设置接收的数据

void	sendData(uchar buf[]);//发送数据

#endif
