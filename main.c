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

#define Baudrate1	38400		//define the baudrate, 如果使用BRT做波特率发生器,则波特率跟串口2一样
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define Baudrate2	9600		//define the baudrate2,
									//12T mode: 600~115200 for 22.1184MHZ, 300~57600 for 11.0592MHZ

#define	BUF_LENTH	128			//定义串口接收缓冲长度

#define UPLOAD_FREQ	10			//计数频率，单位秒

/**********************************************************/

#include	<reg51.h>
#include 	"GPS.h"
#include 	"GPRS.h"

//#define uchar unsigned char;


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
uchar 	GPS_wr = 0;		//写指针
uchar 	GPS_rd = 0;		//读指针
uchar 	xdata GPS_Buffer[BUF_LENTH];//接收缓存	[0]表示写允许位 [1]表示读允许位
unsigned int	gps_rev_start = 0;//接收标志


uchar 	count = 0;		//计数

//请求数据18,43,57
uchar 	xdata 	request[128] = "GET /?data=1&ccid=00000000000000000000&lon=0000.0000&lat=0000.0000 HTTP/1.1\r\nHost:www.luodongseu.top\r\n\r\n\r\n";

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
	TMOD = 0x20;    //定时器1工作在方式2  8位自动重装
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

//GPS串口初始化
void	GPS_init(void)
{
	AUXR1 |= (1<<4);	//将UART2从P1口切换到 RXD2--P1.2切换到P4.2   TXD2---P1.3切换到P4.3
	AUXR |=  (1 << 3);		//串口2波特率加倍
	S2CON  = (S2CON & 0x3f) | (1<<6);	//串口2模式1，8位UART，(2^S2SMOD / 32) * BRT溢出率
	S2CON |= 1 << 4;		//允许串2接收
	//S2CON = 0x50;   //串口2工作在方式1  10位异步收发 S2REN=1允许接收

	AUXR |=  1 << 4;	//baudrate use BRT
	BRT = BRT_Reload;

	IE2 |=  1;			//允许串口2中断
}


////GPRS口发送数据
//void	GPRS_TxByte(uchar dat)
//{
//	SBUF = dat;	   	//将接收的数据发送回去
//	while(TI == 0);	//检查发送中断标志位
//	TI = 0;			//令发送中断标志位为0（软件清零）
//}
//
////GPRS口发送一串字符串
//void GPRS_TxString(uchar code *puts)		
//{
//    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//遇到停止符0结束
//}

//GPS输出口发送数据 用于调试数据
void Test_TxByte(uchar c)
{
	S2BUF = c;
	while(TI2);
	CLR_TI2();
}

//GPS口发送一串字符串
void Test_TxString(uchar *puts)		
{
    for (; *puts != '\0';	puts++)  GPS_TxByte(*puts); 	//遇到停止符0结束
}

//清除GPRS接收数据缓存
void clear_gprs_rev_buf()
{
	uchar index=0;
	 
	for(index = 0 ; index < BUF_LENTH ; index++)
	{
		GPRS_Buffer[index]='\0';
	}

	GPRS_Buffer[0] = 2;		//允许写位
	GPRS_Buffer[1] = 2;		//允许读位
	GPRS_wr = 0;			//写指针位
}

//清除GPS接收数据缓存
void clear_gps_rev_buf()
{
	uchar index=0;
	 
	for(index = 0 ; index < BUF_LENTH ; index++)
	{
		GPS_Buffer[index]='\0';
	}

	GPS_Buffer[0] = 2;		//允许写位
	GPS_Buffer[1] = 2;		//允许读位
	GPS_wr = 0;			//写指针位
	gps_rev_start = 0;	
}

/**
串口1中断响应
*************
接收GPRS数据
*************
对应--GPRS模块
*************
**/
void 	GPRS_RCV (void) interrupt 4
{
	uchar ch;
	if(RI)
	{
		//EA = 0;	//暂停中断
		RI = 0;	//复位

		
		ch = SBUF;

		//GPS_TxString("gprs rev");
		if(GPRS_Listening != 1)	   //GPRS在初始化中,将中断数据返回给GPRS
		{
			setRevBuf(ch);	
		}
		else
		{
		  	if(ch == '{')  // 服务器返回有效数据起始位为'{'
			{
				GPRS_Buffer[0] = 1;	//数据缓存允许位置1，表示允许接收
				GPRS_Buffer[1] = 2;	//数据缓存可读位置2，表示数据不可读
				GPRS_wr = 0;		
			}
			else if (GPRS_Buffer[0] == 1 && ch == '}') //服务器返回有效数据结束位为'}'
			{
				GPRS_Buffer[1] = 1;		//数据缓存可读位置1，表示数据可读		
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

		//EA = 1;	//打开中断
	}
}

uchar a[5] = "abc\n";//测试的数据

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
	
	if(!RI2)
	{
		return;
	}

	if(GPRS_Listening != 1 ) //只有当计数为最大值时，且处于监听状态才可以读取数据
	{
		CLR_RI2();
		return;
	}

	ch=S2BUF;

	if (gps_rev_start == 0 && ch == '$')  //如果收到字符'$' 标志为1
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
		GPS_wr = 2;
		GPS_Buffer[1]=2;   	//不允许读
		GPS_Buffer[0]=1;  	//允许写
	}
	else if(gps_rev_start >= 1 && gps_rev_start < 7)
	{
		gps_rev_start = 0;	
	}

	if(gps_rev_start == 7 && GPS_Buffer[1] != 1)
	{

		if (ch == '\n')     //如果接收到换行
		{
			count = count + 1;
			gps_rev_start = 0; 	//只要遇到换行符，必须重新匹配开头
			//Test_TxString("GO END\n");
			if(count == UPLOAD_FREQ + 1) // 当计数等于count+1 的时候才记录数据,让等于10的时候记录数据
			{
				//Test_TxString("GOT A END\n");		
				GPS_Buffer[0]=2; //不允许写
				GPS_Buffer[1]=1; //允许读 
				count = 0;		//重置计数器
				delay(1000);
			}			
		}
		else //$GPRMC，与换行符中间的数据
		{				
			if(count == UPLOAD_FREQ && GPS_Buffer[0] == 1)//可写
			{
			   	GPS_Buffer[GPS_wr] = ch;  //字符存到数组中
				GPS_wr = GPS_wr + 1;
			}				
		}
			
	}
	CLR_RI2();
}

//入口主函数
void main(){
	GPRS_wr = 0;
	GPS_wr = 0;
	GPRS_Listening = 0;
	
	clear_gps_rev_buf();
	clear_gprs_rev_buf();
	
	GPS_init();
	GPRS_init();

	Test_TxString("main____\0");
	gprs_start();	   				//启动GPRS初始化功能

	Test_TxString("main start listenning\n");
	
	//填充request中的CCID号
	uchar[]	ccid = getCcid();
	for(uchar index = 0;index < 20;index++)
	{
		request[18+index] = ccid[index];
	}
	
	GPRS_Listening = 1;				//开始监听网络传输数据

	//EA = 0;
	while(1)
	{
		
		//除了GPS数据
		if(GPS_Buffer[1] == 1)	//GPS数据可读时操作
		{
			Test_TxString("Send GPS Buffer:");
			Test_TxString(&GPS_Buffer[2]);
			Test_TxString("\n");
			/**
			**[13]-定位结果状态V/A
			**[43-51]-纬度 ddmm.mmmm
			**[57-65]-经度 ddmm.mmmm
			**/
			if(GPS_Buffer[13] != 'A')
			{
				//定位失败,发送-1		
				sendData("GET /?data=-1 HTTP/1.1\r\nHost:www.luodongseu.top\r\n\r\n\r\n");
			}
			else
			{
				//填充真实坐标数据
				//纬度
				request[43]=GPS_Buffer[15];					
				request[44]=GPS_Buffer[16];
				request[45]=GPS_Buffer[17];
				request[46]=GPS_Buffer[18];
				//request[47]=GPS_Buffer[19];//小数点
				request[48]=GPS_Buffer[20];
				request[49]=GPS_Buffer[21];
				request[50]=GPS_Buffer[22];
				request[51]=GPS_Buffer[23];
				//经度
				request[57]=GPS_Buffer[27];
				request[58]=GPS_Buffer[28];
				request[59]=GPS_Buffer[29];
				request[60]=GPS_Buffer[30];
				//request[61]=GPS_Buffer[31];//小数点
				request[62]=GPS_Buffer[32];
				request[63]=GPS_Buffer[33];
				request[64]=GPS_Buffer[34];
				request[65]=GPS_Buffer[35];

		   		sendData(datas);
			}
			clear_gps_rev_buf();
		}
		//处理GPRS接收的数据
		if(GPRS_Buffer[1] == 1)	 //GPRS收到服务器返回的数据
		{
			Test_TxString("Got GPRS Buffer:");
			Test_TxString(&GPRS_Buffer[2]);
			Test_TxString("\n");
			clear_gprs_rev_buf();
		}
		//delay(2000);
	}
}
