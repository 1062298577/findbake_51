/**
**	FINDBAKE MAIN C FILE
**
** 	TODO:
**		 run program at the first	
**
**	@luodongseu	github.com/luodongseu	 2016/5/6
**/

//
// ģ��ӿ�ʵ��˵����
//					 ����1��GPRSģ�飨���պͷ������ݣ�
//					 ����2��GPSģ��	������GPS���ݣ�
//
//

/*************** �û�������� *****************************/

#define MAIN_Fosc	22118400L	//define main clock

#define Baudrate1	9600		//define the baudrate, ���ʹ��BRT�������ʷ�����,�����ʸ�����2һ��
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2	19200		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define	BUF_LENTH	128			//���崮�ڽ��ջ��峤��

/**********************************************************/

#include	<reg51.h>
#include 	"GPS.h"
#include 	"GPRS.h"

sfr AUXR1 = 0xA2;
sfr	AUXR  = 0x8E;
sfr S2CON = 0x9A;	//12C5A60S2˫����ϵ��
sfr S2BUF = 0x9B;	//12C5A60S2˫����ϵ��
sfr IE2   = 0xAF;	//STC12C5A60S2ϵ��
sfr BRT   = 0x9C;

//����1��д
unsigned char 	uart1_wr;		//дָ��
unsigned char 	uart1_rd;		//��ָ��
unsigned char 	xdata RX1_Buffer[BUF_LENTH];
bit		B_TI;

//����2��д
unsigned char 	uart2_wr;		//дָ��
unsigned char 	uart2_rd;		//��ָ��
unsigned char 	xdata RX2_Buffer[BUF_LENTH];
bit		B_TI2;

/****************** �������Զ����ɣ��û������޸� ************************************/

#define T1_TimerReload	(256 - MAIN_Fosc / 192 / Baudrate1)			//Calculate the timer1 reload value	at 12T mode
#define BRT_Reload		(256 - MAIN_Fosc / 12 / 16 / Baudrate2)		//Calculate BRT reload value

#define	TimeOut1		(28800 / (unsigned long)Baudrate1 + 2)
#define	TimeOut2		(28800 / (unsigned long)Baudrate2 + 2)

#define	TI2				(S2CON & 0x02) != 0
#define	RI2				(S2CON & 0x01) != 0
#define	CLR_TI2()		S2CON &= ~0x02
#define	CLR_RI2()		S2CON &= ~0x01

/**********************************************************/


//GPRS���ڳ�ʼ��
void	GPRS_init(void)
{
	PCON |= 0x80;		//UART0 Double Rate Enable
	SCON = 0x50;		//UART0 set as 10bit , UART0 RX enable
	TMOD &= ~(1<<6);		//Timer1 Set as Timer, 12T
	TMOD = (TMOD & ~0x30) | 0x20;	//Timer1 set as 8 bits auto relaod
	TH1 = T1_TimerReload;		//Load the timer
	TR1  = 1;
	ES  = 1;
	EA = 1;
}

//GPS���ڳ�ʼ��
void	GPS_init(void)
{
	AUXR1 |= (1<<4);	//��UART2��P1���л��� RXD2--P1.2�л���P4.2   TXD2---P1.3�л���P4.3
	AUXR |=  (1 << 3);		//����2�����ʼӱ�
	//S2CON  = (S2CON & 0x3f) | (1<<6);	//����2ģʽ1��8λUART��(2^S2SMOD / 32) * BRT�����
	//S2CON |= 1 << 4;		//����2����
	S2CON = 0x50;   //����2�����ڷ�ʽ1  10λ�첽�շ� S2REN=1�������

	AUXR |=  1 << 4;	//baudrate use BRT
	BRT = BRT_Reload;

	IE2 |=  1;			//������2�ж�
}


//GPRS�ڷ�������
void	GPRS_TxByte(unsigned char dat)
{
	//B_TI = 0;
	SBUF = dat;
	//while(!B_TI);
	//B_TI = 0;
}

//GPRS�ڷ���һ���ַ���
void GPRS_TxString(unsigned char code *puts)		
{
    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//����ֹͣ��0����
}


/**
����1�ж���Ӧ
*************
����GPRS����
*************
��Ӧ--GPRSģ��
*************
**/
void GPRS_RCV (void) interrupt 4
{
	if(RI)
	{
		RI = 0;

		RX2_Buffer[uart2_wr] = SBUF;
		if(++uart2_wr >= BUF_LENTH)	uart2_wr = 0;

		RX1_Buffer[uart1_wr] = SBUF;
		if(++uart1_wr >= BUF_LENTH)	uart1_wr = 0;
	}

	if(TI)
	{
		TI = 0;
		B_TI = 1;
	}
}

/**
����1�ж���Ӧ
*************
����GPS����
*************
��Ӧ--GPSģ��
*************
**/
void GPS_RCV (void) interrupt 8
{
	if(RI2)
	{	
		CLR_RI2();
		//UART1_TxByte(S2BUF);	
		RX1_Buffer[uart1_wr] = S2BUF;
		if(++uart1_wr >= BUF_LENTH)	uart1_wr = 0;
	}

	if(TI2)
	{
		CLR_TI2();
		B_TI2 = 1;
	}
}


//���������
void main(){
	uart1_rd = 0;
	uart1_wr = 0;
	uart2_rd = 0;
	uart2_wr = 0;

	GPRS_init();
	GPS_init();

	GPRS_TxString("����1--GPRS");

	while(1)
	{
		if(uart1_rd != uart1_wr)	//����GPRS����
		{
			GPRS_TxByte(RX1_Buffer[uart1_rd]);		  //��������
			if(++uart1_rd >= BUF_LENTH)		uart1_rd = 0;
		}
	}
}
