/**
**	FINDBAKE MAIN C FILE
**
** 	TODO:
**		 run program at the first	
**
**	@luodongseu	github.com/luodongseu	 2016/5/6
**/

//
// 模块接口实现说明：
//					 串口1 	P30,P31：GPRS模块（接收和发送数据）
//					 串口2	P42：GPS模块	（接收GPS数据）
//
// 功能说明：
//			GPS串口不断接收数据，存储到本地GPSINFO变量中
//			GPRS定时将GPSINFO变量中的数据发送到服务器中	
//

/*************** 用户定义参数 *****************************/

#define MAIN_Fosc	22118400L	//define main clock

#define Baudrate1	19200		//define the baudrate, 如果使用BRT做波特率发生器,则波特率跟串口2一样
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2	9600		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define	BUF_LENTH	128			//定义串口接收缓冲长度

/**********************************************************/

#include	<reg51.h>
#include 	"GPS.h"
#include 	"GPRS.h"

#define unsigned char uchar;


sfr AUXR1 = 0xA2;
sfr	AUXR  = 0x8E;
sfr S2CON = 0x9A;	//12C5A60S2双串口系列
sfr S2BUF = 0x9B;	//12C5A60S2双串口系列
sfr IE2   = 0xAF;	//STC12C5A60S2系列
sfr BRT   = 0x9C;

//串口1读写
uchar 	GPRS_wr;		//写指针
uchar 	GPRS_rd;		//读指针
uchar 	xdata GPRS_Buffer[BUF_LENTH];//接收缓存：
bit		B_TI;	  		//中断标志位
uchar	GPRS_Listening;	//启动成功后监听


//串口2读写
uchar 	GPS_wr;		//写指针
uchar 	GPS_rd;		//读指针
uchar 	xdata GPS_Buffer[BUF_LENTH];//接收缓存	[0]表示写允许位 [1]表示读允许位
uchar	gps_rev_start;//接收标志

/****************** 编译器自动生成，用户请勿修改 ************************************/

#define T1_TimerReload	(256 - MAIN_Fosc / 192 / Baudrate1)			//Calculate the timer1 reload value	at 12T mode
#define BRT_Reload		(256 - MAIN_Fosc / 12 / 16 / Baudrate2)		//Calculate BRT reload value

#define	TimeOut1		(28800 / (unsigned long)Baudrate1 + 2)
#define	TimeOut2		(28800 / (unsigned long)Baudrate2 + 2)

#define	TI2				(S2CON & 0x02) != 0
#define	RI2				(S2CON & 0x01) != 0
#define	CLR_TI2()		S2CON &= ~0x02
#define	CLR_RI2()		S2CON &= ~0x01

/**********************************************************/


//GPRS串口初始化
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

//GPS串口初始化
void	GPS_init(void)
{
	AUXR1 |= (1<<4);	//将UART2从P1口切换到 RXD2--P1.2切换到P4.2   TXD2---P1.3切换到P4.3
	AUXR |=  (1 << 3);		//串口2波特率加倍
	//S2CON  = (S2CON & 0x3f) | (1<<6);	//串口2模式1，8位UART，(2^S2SMOD / 32) * BRT溢出率
	//S2CON |= 1 << 4;		//允许串2接收
	S2CON = 0x50;   //串口2工作在方式1  10位异步收发 S2REN=1允许接收

	AUXR |=  1 << 4;	//baudrate use BRT
	BRT = BRT_Reload;

	IE2 |=  1;			//允许串口2中断
}


//GPRS口发送数据
void	GPRS_TxByte(uchar dat)
{
	SBUF = dat;	   	//将接收的数据发送回去
	while(TI == 0);	//检查发送中断标志位
	TI = 0;			//令发送中断标志位为0（软件清零）
}

//GPRS口发送一串字符串
void GPRS_TxString(uchar code *puts)		
{
    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//遇到停止符0结束
}

//GPS输出口发送数据 用于调试数据
void GPS_TxByte(uchar c)
{
	S2BUF = c;
	while(TI2);
	CLR_TI2();
}

//GPS口发送一串字符串
void GPS_TxString(uchar *puts)		
{
    for (; *puts != '\0';	puts++)  GPS_TxByte(*puts); 	//遇到停止符0结束
}

//清除GPRS接收数据缓存
void clear_gprs_rev_buf()
{
	uchar index=0;
	 
	for(uchar=0;uchar<MAX_LEN;uchar++)
	{
		GPRS_Buffer[uchar]='\0';
	}

	GPRS_Buffer[0] = 0;		//允许写位
	GPRS_Buffer[1] = 0;		//允许读位
	GPRS_wr = 0;			//写指针位
}

/**
串口1中断响应
*************
接收GPRS数据
*************
对应--GPRS模块
*************
**/
void GPRS_RCV (void) interrupt 4
{
	if(RI)
	{
		EA = 0;	//暂停中断
		RI = 0;	//复位

		uchar ch;
		ch = SBUF;

		if(GPRS_Listening != 1)	   //GPRS在初始化中,将中断数据返回给GPRS
		{
			setRevBuf(ch);	
		}
		else
		{
		  	if(ch == '{')  // 服务器返回有效数据起始位为'{'
			{
				GPRS_Buffer[0] = 1;	//数据缓存允许位置1，表示允许接收
				GPRS_Buffer[1] = 0;	//数据缓存可读位置0，表示数据不可读
				GPRS_wr = 0;		
			}
			else if (GPRS_Buffer[0] == 1 && ch == '}') //服务器返回有效数据结束位为'}'
			{
				GPRS_Buffer[1] = 1;		//数据缓存可读位置1，表示数据可读	
				GPRS_wr = 1;	
			}
			else if (GPRS_Buffer[0] == 1)
			{
				if(++GPRS_wr < BUF_LENTH -1)		//防止数据溢出
				{
					GPRS_Buffer[GPRS_wr] = ch;
				}
				else
				{
					clear_gprs_rev_buf();
				}
				
			}
			else	//无效数据
			{
				clear_gprs_rev_buf();
			}	
		}

		EA = 1;	//打开中断
	}
}

/**
串口1中断响应
*************
接收GPS数据
*************
对应--GPS模块 只解析GPRMC数据
*************
**/
void GPS_RCV (void) interrupt 8
{
	uchar ch;	
	ch=S2BUF;
	if (ch == '$')  //如果收到字符'$' 标志为1
	{
	 	gps_rev_start = 1;
	}	
	else if (gps_rev_start == 1 && ch == 'G') //如果 标志位为1,收到字符'G'，判断第2位
	{
	 	gps_rev_start = 2;			
	}
	else if (gps_rev_start == 2 && ch == 'P')  //如果 标志位为2,收到字符'P'，判断第3位
	{
		gps_rev_start = 3;
	}
	else if (gps_rev_start == 3 && ch == 'R')  //如果 标志位为3,收到字符'R'，判断第4位
	{
		gps_rev_start = 4;
	}
	else if (gps_rev_start == 4 && ch == 'M')  //如果 标志位为4,收到字符'M'，判断第5位
	{
		gps_rev_start = 5;
	}
	else if (gps_rev_start == 5 && ch == 'C')  //如果 标志位为5,收到字符'C'，判断第6位
	{
		gps_rev_start = 6;
	}
	else if (gps_rev_start == 6 && ch == ',')  //如果 标志位为6,收到字符','，开始接收数据
	{
		gps_rev_start = 7;
		num = 1;
		GPS_Buffer[1]=0;
	}

	if(gps_rev_start == 7)
	{
		if(1)  //GPS_Buffer[0]==0
		{
			GPS_Buffer[++num] = ch;  //字符存到数组中
			if (ch == 0x0D)     //如果接收到换行
			{
				GPS_Buffer[++num] = '\0';
				GPS_Buffer[0]=1; //close present write
				GPS_Buffer[1]=1; //open prensnt read
			}
		} 
	}
	S2CON&=~S2RI;
}
//入口主函数
void main(){
	uart1_rd = 0;
	uart1_wr = 0;
	uart2_rd = 0;
	uart2_wr = 0;
	GPRS_Listening = 0;

	GPRS_init();
	GPS_init();

	GPRS_StartUp();	   				//启动GPRS初始化功能
	GPRS_Listening = 1;				//开始监听网络传输数据

	GPRS_TxString("串口1--GPRS");

	while(1)
	{
		if(GPS_Buffer[1] == 1)	//GPS数据可读时操作
		{
			uchar datas[256] = "GET / HTTP/1.1\r\n\r\ndata=";
			for(uchar i = 28;i<256;i++)			//将GPS_Buffer加在datas的尾部
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



		//处理GPRS接收的数据
		if(GPRS_Buffer[1] == 1)	//数据可读时操作
		{
		   	GPS_TxString(&GPRS_Buffer[2]);
		}
	}
}
