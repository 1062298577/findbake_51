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

#define Baudrate1	38400		//define the baudrate, ���ʹ��BRT�������ʷ�����,�����ʸ�����2һ��
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2	9600		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define	BUF_LENTH	128			//���崮�ڽ��ջ��峤��



/**********************************************************/

#include	<reg51.h>
#include 	"GPS.h"
#include 	"GPRS.h"

//#define uchar unsigned char;

sbit BEEP = P1^0;	//������

sfr AUXR1 = 0xA2;
sfr	AUXR  = 0x8E;
sfr S2CON = 0x9A;	//12C5A60S2˫����ϵ��
sfr S2BUF = 0x9B;	//12C5A60S2˫����ϵ��
sfr IE2   = 0xAF;	//STC12C5A60S2ϵ��
sfr BRT   = 0x9C;

//����1��д
uchar 	GPRS_wr;		//дָ��
uchar 	GPRS_rd;		//��ָ��
uchar 	xdata	GPRS_Buffer[BUF_LENTH];//���ջ��棺
bit		B_TI;	  		//�жϱ�־λ
bit		GPRS_Listening;	//�����ɹ������
bit		GPRS_SEND;		//���ͱ�־
bit		GPRS_REV;		//���ձ�־


//����2��д
uchar 	GPS_wr = 0;		//дָ��
uchar 	GPS_rd = 0;		//��ָ��
uchar 	xdata	GPS_Buffer[BUF_LENTH];//���ջ���	[0]��ʾд����λ [1]��ʾ������λ
unsigned int	gps_rev_start = 0;//���ձ�־


bit 	voice =	0;		//������

uchar 	UPLOAD_FREQ	= 10;//����Ƶ�ʣ���λ��
uchar 	count = 0;		//����

//��������18,43,57
uchar 	xdata 	request[128] = "GET /?data=1&ccid=00000000000000000000&lat=0000.0000&lon=00000.0000 HTTP/1.1\r\nHost:\r\n\r\n";
//��λʧ��ʱ������19
uchar 	xdata 	fail_request[128] = "GET /?data=-1&ccid=00000000000000000000 HTTP/1.1\r\nHost:\r\n\r\n";

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
/*********************************************************
 500us��ʱ����
 ����22.1184MHz
*********************************************************/
void delay500us(void)
{
  	uchar  i,j;
	for(j=500;j>0;j--)
  		for(i=81;i>0;i--);
}

//GPRS���ڳ�ʼ��
void	GPRS_init(void)
{
	TMOD = 0x20;    //��ʱ��1�����ڷ�ʽ2  8λ�Զ���װ
	//PCON |= 0x80;		//UART0 Double Rate Enable
	SCON = 0x50;		//UART0 set as 10bit , UART0 RX enable
	//TMOD &= ~(1<<6);		//Timer1 Set as Timer, 12T
	//TMOD = (TMOD & ~0x30) | 0x20;	//Timer1 set as 8 bits auto relaod
	TH1 = T1_TimerReload;		//Load the timer
	TL1 = TH1;
	TR1  = 1;
	ES  = 1;
	EA = 1;
}													 

//GPS���ڳ�ʼ��
void	GPS_init(void)
{
	AUXR1 |= (1<<4);	//��UART2��P1���л��� RXD2--P1.2�л���P4.2   TXD2---P1.3�л���P4.3
	AUXR |=  (1 << 3);		//����2�����ʼӱ�
	S2CON  = (S2CON & 0x3f) | (1<<6);	//����2ģʽ1��8λUART��(2^S2SMOD / 32) * BRT�����
	S2CON |= 1 << 4;		//����2����
	//S2CON = 0x50;   //����2�����ڷ�ʽ1  10λ�첽�շ� S2REN=1�������

	AUXR |=  1 << 4;	//baudrate use BRT
	BRT = BRT_Reload;

	IE2 |=  1;			//������2�ж�
}


////GPRS�ڷ�������
//void	GPRS_TxByte(uchar dat)
//{
//	SBUF = dat;	   	//�����յ����ݷ��ͻ�ȥ
//	while(TI == 0);	//��鷢���жϱ�־λ
//	TI = 0;			//����жϱ�־λΪ0��������㣩
//}
//
////GPRS�ڷ���һ���ַ���
//void GPRS_TxString(uchar code *puts)		
//{
//    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//����ֹͣ��0����
//}

//GPS����ڷ������� ���ڵ�������
void Test_TxByte(uchar c)
{
	S2BUF = c;
	while(TI2);
	CLR_TI2();
}

//GPS�ڷ���һ���ַ���
void Test_TxString(uchar *puts)		
{
    for (; *puts != '\0';	puts++)  GPS_TxByte(*puts); 	//����ֹͣ��0����
}

//���GPRS�������ݻ���
void clear_gprs_rev_buf()
{
	uchar index=0;
	 
	for(index = 0 ; index < BUF_LENTH ; index++)
	{
		GPRS_Buffer[index]='\0';
	}
//	GPRS_Buffer[BUF_LENTH-1] = '\0';
	GPRS_Buffer[0] = '0';		//����дλ
	GPRS_Buffer[1] = '0';		//�����λ
	GPRS_wr = 0;			//дָ��λ
}

//���GPS�������ݻ���
void clear_gps_rev_buf()
{
	uchar index=0;
	 
	for(index = 0 ; index < BUF_LENTH ; index++)
	{
		GPS_Buffer[index]='\0';
	}

	GPS_Buffer[0] = 2;		//����дλ
	GPS_Buffer[1] = 2;		//�����λ
	GPS_wr = 0;			//дָ��λ
	gps_rev_start = 0;	
}
uchar a[5] = "__";//���Ե�����
/**
����1�ж���Ӧ
*************
����GPRS����
*************
��Ӧ--GPRSģ��
*************
**/
void 	GPRS_RCV (void) interrupt 4
{
	uchar ch;
	if(RI)
	{
		
		//EA = 0;	//��ͣ�ж�
		RI = 0;	//��λ
		//BEEP = 1;	//��������ͣ

		ch = SBUF;

		if(!GPRS_Listening) 	  //GPRS�ڳ�ʼ����,���ж����ݷ��ظ�GPRS	 
		{
			setRevBuf(ch);	
		}
		else
		{	
			//if(GPRS_REV)
			{
				if(ch=='{')  	// ������������Ч������ʼλΪ'{'
				{
					clear_gprs_rev_buf();
					GPRS_Buffer[0] = '1';	//���ݻ�������λ��1����ʾ�������
					GPRS_Buffer[1] = '0';	//���ݻ���ɶ�λ��0����ʾ���ݲ��ɶ�
					GPRS_wr = 2;
					GPRS_SEND = 0;
					//GPRS_Buffer[9] = 'B';	//����״̬		
				}
				else if(GPRS_Buffer[0]=='1') 
				{
					if(ch=='}')	 //������������Ч���ݽ���λΪ'}'
					{
						GPRS_Buffer[1] = '1';		//���ݻ���ɶ�λ��1����ʾ���ݿɶ�
						GPRS_Buffer[0] = '0';		//���ݻ�������λ��2����ʾ���������
						GPRS_REV == 0;
						//GPRS_Buffer[10] = 'C';  //����״̬
					}
					else
					{
						if(GPRS_wr < BUF_LENTH)		//��ֹ�������
						{
							GPRS_Buffer[GPRS_wr] = ch;
							
							GPRS_wr = 1 + GPRS_wr;
						}
	//					else
	//					{
	//						clear_gprs_rev_buf();
	//					}
					}		
				}
			}						   
			
			if(GPRS_SEND)  //��������ʱ,�����豸��ʼ���������Ҫ������յ����� && GPRS_Buffer[0]=='0'	 
			{
				setRevBuf(ch);
			}
			
			//*********************************************************************
			//*********************************************************************
			//����GPRS��������ר�ô���
			//
//			if(GPRS_wr == BUF_LENTH)
//			{
//				GPRS_wr = 0;
//			}
//			GPRS_Buffer[GPRS_wr++] = ch;
			//*********************************************************************
			//*********************************************************************
	
		}
		
//		a[2] = ch;
//		Test_TxString(a);
//		//EA = 1;	//���ж�
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
	
	if(!RI2)
	{
		return;
	}

	if(GPRS_Listening != 1 ) //ֻ�е�����Ϊ���ֵʱ���Ҵ��ڼ���״̬�ſ��Զ�ȡ����
	{
		CLR_RI2();
		return;
	}

	ch=S2BUF;
	BEEP = 1;	//��������ͣ

	if (gps_rev_start == 0 && ch == '$')  //����յ��ַ�'$' ��־Ϊ1
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
		GPS_wr = 2;
		GPS_Buffer[1]=2;   	//�������
		GPS_Buffer[0]=1;  	//����д
	}
	else if(gps_rev_start >= 1 && gps_rev_start < 7)
	{
		gps_rev_start = 0;	
	}

	if(gps_rev_start == 7 && GPS_Buffer[1] != 1)
	{

		if (ch == '\n')     //������յ�����
		{
			count = count + 1;
			gps_rev_start = 0; 	//ֻҪ�������з�����������ƥ�俪ͷ
			//Test_TxString("GO END\n");
			if(count >= UPLOAD_FREQ + 1) // ����������count+1 ��ʱ��ż�¼����,�õ���10��ʱ���¼����
			{
				//Test_TxString("GOT A END\n");		
				GPS_Buffer[0]=2; //������д
				GPS_Buffer[1]=1; //����� 
				count = 0;		//���ü�����
				delay(1000);
			}			
		}
		else //$GPRMC���뻻�з��м������
		{				
			if(count >= UPLOAD_FREQ && GPS_Buffer[0] == 1)//��д
			{
			   	GPS_Buffer[GPS_wr] = ch;  //�ַ��浽������
				GPS_wr = GPS_wr + 1;
			}				
		}
			
	}
	CLR_RI2();
}


//���������
void main(){
	uchar index = 0;
	uchar*  ccid = getCcid();
	uchar fmq = 0;
	uchar orderIndex = 0;

	GPRS_wr = 0;
	GPS_wr = 0;
	GPRS_Listening = 0;
	GPRS_SEND = 0;
	
	clear_gps_rev_buf();
	clear_gprs_rev_buf();
	
	GPS_init();
	GPRS_init();

	Test_TxString("main____\n");
	gprs_start();	   				//����GPRS��ʼ������

	Test_TxString("main start listenning\n");
	
	//���request�е�CCID��
	
	
	for(index = 0;index < 20;index++)
	{
		request[18+index] = ccid[index];
		fail_request[19+index] = ccid[index];
	}
	
	GPRS_Listening = 1;				//��ʼ�������紫������

	//EA = 0;
	while(1)
	{	
		if(GPS_Buffer[1] == 1)	//GPS���ݿɶ�ʱ����
		{
			//EA = 0;
			Test_TxString("GPS Buffer:");
			Test_TxString(&GPS_Buffer[2]);
			Test_TxString("\n");
			/**
			**[13]-��λ���״̬V/A
			**[43-51]-γ�� ddmm.mmmm
			**[57-65]-���� ddmm.mmmm
			**/
			if(GPS_Buffer[14] != 'A')
			{
				//��λʧ��,����-1
				Test_TxString(fail_request);
				GPRS_SEND = 1;
				GPRS_REV = 1;
				//EA = 1;		
				sendData(fail_request);
				GPRS_SEND = 0;
			}
			else
			{
				//�����ʵ��������
				//γ��
				request[43]=GPS_Buffer[16];					
				request[44]=GPS_Buffer[17];
				request[45]=GPS_Buffer[18];
				request[46]=GPS_Buffer[19];
				//request[47]=GPS_Buffer[20];//С����
				request[48]=GPS_Buffer[21];
				request[49]=GPS_Buffer[22];
				request[50]=GPS_Buffer[23];
				request[51]=GPS_Buffer[24];
				//����
				request[57]=GPS_Buffer[28];
				request[58]=GPS_Buffer[29];
				request[59]=GPS_Buffer[30];
				request[60]=GPS_Buffer[31];
				request[61]=GPS_Buffer[32];
				//request[62]=GPS_Buffer[33];//С����
				request[63]=GPS_Buffer[34];
				request[64]=GPS_Buffer[35];
				request[65]=GPS_Buffer[36];
				request[66]=GPS_Buffer[37];
			   	Test_TxString(request);
				GPRS_SEND = 1;
				GPRS_REV = 1;
				//EA = 1;
		   		sendData(request);
				GPRS_SEND = 0;
			}
			clear_gps_rev_buf();
			
		}
		
		//*********************************************************************
		//*********************************************************************
		//����GPRS��������ר�ô���
		//
//		Test_TxString("Got GPRS Buffer2:");
//		Test_TxString(GPRS_Buffer);
//		Test_TxString("\n");
		//*********************************************************************
		//*********************************************************************

		if(GPRS_Buffer[1]=='1')	 //����GPRS���յ����� //GPRS�յ����������ص�����
		{
			//EA = 0;
			Test_TxString("Got GPRS Buffer:");
			Test_TxString(&GPRS_Buffer[2]);
			Test_TxString("\n");

			//TODO ������յ�������
			if(GPRS_Buffer[2]=='2' && GPRS_Buffer[3]=='0' && GPRS_Buffer[4]=='1')	//���������ָ��
			{
			  	//��ȡ����ָ��
				for(orderIndex=6;orderIndex<BUF_LENTH;orderIndex++)
				{
					if(GPRS_Buffer[orderIndex]=='\0')
					{
						break;
					}
					if(GPRS_Buffer[orderIndex]=='S' && GPRS_Buffer[orderIndex+2] != '\0') //�������S_Xָ��
					{
						if(GPRS_Buffer[orderIndex+2] == 'O')	 //��ָ��
						{
							voice = 1;
							//break;
						}
						else if(GPRS_Buffer[orderIndex+2] == 'C')	 //�ر�ָ��
						{
							voice = 0;
							//break;
						}
					}

					if(GPRS_Buffer[orderIndex]=='F' && GPRS_Buffer[orderIndex+2] != '\0') //�������F_Xָ��
					{
						if(GPRS_Buffer[orderIndex+2] == 'R')	 //�����ٶ�ָ��
						{
							UPLOAD_FREQ = 30;
							//break;
						}
						else if(GPRS_Buffer[orderIndex+2] == 'H')	 //����ָ��
						{
							UPLOAD_FREQ = 10;;
							//break;
						}
					}
				}
			}

			clear_gprs_rev_buf();//��ջ���
			//EA = 1;
		}
		
	
		if(voice == 1)	   	//����������
		{	
			for(fmq=100;fmq>0;fmq--)
			{
			  BEEP=~BEEP;       //���Ƶ��1KHz
			  delay500us();       //��ʱ500us
			}
		
			for(fmq=100;fmq>0;fmq--)
			{
			  BEEP=~BEEP;       //���Ƶ��500Hz
			  delay500us();       //��ʱ1ms
			  delay500us();
			}
		}

		//delay(2000);
	}
}
