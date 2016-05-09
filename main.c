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
//					 ����1 	P30,P31��GPRSģ�飨���պͷ������ݣ�
//					 ����2	P42��GPSģ��	������GPS���ݣ�
//
// ����˵����
//			GPS���ڲ��Ͻ������ݣ��洢������GPSINFO������
//			GPRS��ʱ��GPSINFO�����е����ݷ��͵���������	
//

/*************** �û�������� *****************************/

#define MAIN_Fosc	22118400L	//define main clock

#define Baudrate1	19200		//define the baudrate, ���ʹ��BRT�������ʷ�����,�����ʸ�����2һ��
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2	9600		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define	BUF_LENTH	128			//���崮�ڽ��ջ��峤��

/**********************************************************/

#include	<reg51.h>
#include 	"GPS.h"
#include 	"GPRS.h"

#define unsigned char uchar;


sfr AUXR1 = 0xA2;
sfr	AUXR  = 0x8E;
sfr S2CON = 0x9A;	//12C5A60S2˫����ϵ��
sfr S2BUF = 0x9B;	//12C5A60S2˫����ϵ��
sfr IE2   = 0xAF;	//STC12C5A60S2ϵ��
sfr BRT   = 0x9C;

//����1��д
uchar 	GPRS_wr;		//дָ��
uchar 	GPRS_rd;		//��ָ��
uchar 	xdata GPRS_Buffer[BUF_LENTH];//���ջ��棺
bit		B_TI;	  		//�жϱ�־λ
uchar	GPRS_Listening;	//�����ɹ������


//����2��д
uchar 	GPS_wr;		//дָ��
uchar 	GPS_rd;		//��ָ��
uchar 	xdata GPS_Buffer[BUF_LENTH];//���ջ���	[0]��ʾд����λ [1]��ʾ������λ
uchar	gps_rev_start;//���ձ�־

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
void	GPRS_TxByte(uchar dat)
{
	SBUF = dat;	   	//�����յ����ݷ��ͻ�ȥ
	while(TI == 0);	//��鷢���жϱ�־λ
	TI = 0;			//����жϱ�־λΪ0��������㣩
}

//GPRS�ڷ���һ���ַ���
void GPRS_TxString(uchar code *puts)		
{
    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//����ֹͣ��0����
}

//GPS����ڷ������� ���ڵ�������
void GPS_TxByte(uchar c)
{
	S2BUF = c;
	while(TI2);
	CLR_TI2();
}

//GPS�ڷ���һ���ַ���
void GPS_TxString(uchar *puts)		
{
    for (; *puts != '\0';	puts++)  GPS_TxByte(*puts); 	//����ֹͣ��0����
}

//���GPRS�������ݻ���
void clear_gprs_rev_buf()
{
	uchar index=0;
	 
	for(uchar=0;uchar<MAX_LEN;uchar++)
	{
		GPRS_Buffer[uchar]='\0';
	}

	GPRS_Buffer[0] = 0;		//����дλ
	GPRS_Buffer[1] = 0;		//�����λ
	GPRS_wr = 0;			//дָ��λ
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
		EA = 0;	//��ͣ�ж�
		RI = 0;	//��λ

		uchar ch;
		ch = SBUF;

		if(GPRS_Listening != 1)	   //GPRS�ڳ�ʼ����,���ж����ݷ��ظ�GPRS
		{
			setRevBuf(ch);	
		}
		else
		{
		  	if(ch == '{')  // ������������Ч������ʼλΪ'{'
			{
				GPRS_Buffer[0] = 1;	//���ݻ�������λ��1����ʾ�������
				GPRS_Buffer[1] = 0;	//���ݻ���ɶ�λ��0����ʾ���ݲ��ɶ�
				GPRS_wr = 0;		
			}
			else if (GPRS_Buffer[0] == 1 && ch == '}') //������������Ч���ݽ���λΪ'}'
			{
				GPRS_Buffer[1] = 1;		//���ݻ���ɶ�λ��1����ʾ���ݿɶ�	
				GPRS_wr = 1;	
			}
			else if (GPRS_Buffer[0] == 1)
			{
				if(++GPRS_wr < BUF_LENTH -1)		//��ֹ�������
				{
					GPRS_Buffer[GPRS_wr] = ch;
				}
				else
				{
					clear_gprs_rev_buf();
				}
				
			}
			else	//��Ч����
			{
				clear_gprs_rev_buf();
			}	
		}

		EA = 1;	//���ж�
	}
}

/**
����1�ж���Ӧ
*************
����GPS����
*************
��Ӧ--GPSģ�� ֻ����GPRMC����
*************
**/
void GPS_RCV (void) interrupt 8
{
	uchar ch;	
	ch=S2BUF;
	if (ch == '$')  //����յ��ַ�'$' ��־Ϊ1
	{
	 	gps_rev_start = 1;
	}	
	else if (gps_rev_start == 1 && ch == 'G') //��� ��־λΪ1,�յ��ַ�'G'���жϵ�2λ
	{
	 	gps_rev_start = 2;			
	}
	else if (gps_rev_start == 2 && ch == 'P')  //��� ��־λΪ2,�յ��ַ�'P'���жϵ�3λ
	{
		gps_rev_start = 3;
	}
	else if (gps_rev_start == 3 && ch == 'R')  //��� ��־λΪ3,�յ��ַ�'R'���жϵ�4λ
	{
		gps_rev_start = 4;
	}
	else if (gps_rev_start == 4 && ch == 'M')  //��� ��־λΪ4,�յ��ַ�'M'���жϵ�5λ
	{
		gps_rev_start = 5;
	}
	else if (gps_rev_start == 5 && ch == 'C')  //��� ��־λΪ5,�յ��ַ�'C'���жϵ�6λ
	{
		gps_rev_start = 6;
	}
	else if (gps_rev_start == 6 && ch == ',')  //��� ��־λΪ6,�յ��ַ�','����ʼ��������
	{
		gps_rev_start = 7;
		num = 1;
		GPS_Buffer[1]=0;
	}

	if(gps_rev_start == 7)
	{
		if(1)  //GPS_Buffer[0]==0
		{
			GPS_Buffer[++num] = ch;  //�ַ��浽������
			if (ch == 0x0D)     //������յ�����
			{
				GPS_Buffer[++num] = '\0';
				GPS_Buffer[0]=1; //close present write
				GPS_Buffer[1]=1; //open prensnt read
			}
		} 
	}
	S2CON&=~S2RI;
}
//���������
void main(){
	uart1_rd = 0;
	uart1_wr = 0;
	uart2_rd = 0;
	uart2_wr = 0;
	GPRS_Listening = 0;

	GPRS_init();
	GPS_init();

	GPRS_StartUp();	   				//����GPRS��ʼ������
	GPRS_Listening = 1;				//��ʼ�������紫������

	GPRS_TxString("����1--GPRS");

	while(1)
	{
		if(GPS_Buffer[1] == 1)	//GPS���ݿɶ�ʱ����
		{
			uchar datas[256] = "GET / HTTP/1.1\r\n\r\ndata=";
			for(uchar i = 28;i<256;i++)			//��GPS_Buffer����datas��β��
			{
				uchar j = i - 28 + 2;
				if(j > BUF_LEN - 1 || GPS_Buffer[j] == '\0')
				{
					datas[i] = '\0';	
					break;
				}
				datas[i] = GPS_Buffer[j];	
			}
		   	sendData(datas);
		}



		//����GPRS���յ�����
		if(GPRS_Buffer[1] == 1)	//���ݿɶ�ʱ����
		{
		   	GPS_TxString(&GPRS_Buffer[2]);
		}
	}
}
