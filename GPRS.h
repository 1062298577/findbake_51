#ifndef __GPRS_H__
#define __GPRS_H__
#define uchar unsigned char
#define uint8 unsigned int

void	gprs_start();//����GPRSģ��

void	setRevBuf(uchar b);//���ý��յ�����

void	sendData(uchar buf[]);//��������

void 	GPS_TxByte(uchar c);	//�����ֽ�

void	GPS_TxString(uchar code *p);//GPS�����ַ���

void 	delay(int i);	//��ʱ

unsigned char[]	getCcid();		//��ȡCCID��   

#endif
