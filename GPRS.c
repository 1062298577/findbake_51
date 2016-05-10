#include "GPRS.h"
#include<STC12C5A60S2.h>
#include "string.h"

#define S2RI 		0x01	//串口2接收中断请求标志位
#define S2TI 		0x02	//串口2发送中断请求标志位 

#define RST      	0x31
#define AT     		0x32
#define CCID  		0x33
#define STACK     	0x34
#define APN     	0x35
#define INIT_PPP    0x36
#define CHECK_PPP   0x37
#define LINK     	0x38
#define CHECK_NET 	0x39
#define LINK_FINISH 0x3A	  

#define MAX_LEN 	128	  	//缓存最大长度

#define	TI2				(S2CON & S2TI)
#define	CLR_TI2()		S2CON &= ~S2TI

uchar 	gprs_state;			//初始化状态标志位
uint8 	num;				//计数

uchar  	xdata	rev_buf[MAX_LEN]; 	//接收缓存
uchar 	xdata	send_buf[MAX_LEN];	//发送缓存

uchar	xdata	ccid[30];		//存储CCID   	 

uchar 	retry_count=0; 		//重试次数标记  


/*=========延时=====================*/
void 	delay(int i)               
{
	int j,k;
	for(j=0;j<i;j++)
		for(k=0;k<990;k++);
}

//GPRS口发送数据
void	GPRS_TxByte(uchar dat)
{
	SBUF = dat;	   	//将接收的数据发送回去
	while(TI == 0);	//检查发送中断标志位
	TI = 0;			//令发送中断标志位为0（软件清零）
}

//GPRS口发送一串字符串
void 	GPRS_TxString(char *puts)		
{
    while(*puts != 0)  
		GPRS_TxByte(*puts++); 	//遇到停止符0结束
}

//GPS输出口发送数据 用于调试数据
void 	GPS_TxByte(uchar c)
{
	S2BUF = c;
	while(!TI2);
	CLR_TI2();
}

//GPS口发送一串字符串
void 	GPS_TxString(char *puts)		
{
    while(*puts != 0)  
		GPS_TxByte(*puts++); 	//遇到停止符0结束
}

//清除接收缓存
void 	clear_rev_buf()
{
	uint8 index=0;
	num = 0;
	 
	for(index=0;index<MAX_LEN;index++)
	{
		rev_buf[index]='\0';
	}       
}

//检查AT指令是否可用 指令AT\r\n
void 	gprs_at()
{
	GPS_TxString("gprs_at()\n");

	GPS_TxString(rev_buf);
	GPS_TxString("\n");

	if(gprs_state == RST)
	{	
		clear_rev_buf();
	    retry_count=0;
		GPRS_TxString("AT\r\n");
		gprs_state=AT;
		delay(500);
	}
}

//检查SIM卡的CCID号	指令AT+CCID\r\n
void 	gprs_check_ccid()
{
	GPS_TxString("gprs_check_ccid()\n");
	if(gprs_state != AT)
	{
		return;
	}
	
	GPS_TxString(rev_buf);
	GPS_TxString("\n");
	
	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{	
		clear_rev_buf();
	    retry_count=0;
		GPRS_TxString("AT+CCID\r\n");
		gprs_state=CCID;
		delay(500);
	}
	else
	{	
		clear_rev_buf();
		GPRS_TxString("AT\r\n");
		gprs_state=AT;
		delay(500);
		retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}
}

//设置为内部协议 指令AT+XISP=0\r\n
void 	gprs_stack()
{
	GPS_TxString("gprs_stack()\n");
	if(gprs_state != CCID)
		return;
	
	GPS_TxString(rev_buf);
	GPS_TxString("\n");
			
	if(rev_buf[2] == '+' && rev_buf[3]=='C')
	{
	    clear_rev_buf();
		retry_count=0;
		gprs_state=STACK;
		GPRS_TxString("AT+XISP=0\r\n");
		delay(500);
	}
	else
	{	
		clear_rev_buf();
		GPRS_TxString("AT+CCID\r\n");
		gprs_state=CCID;
		delay(500);
		retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}
}

//设置访问节点APN	指令 AT+CGDCONT=1,"IP","CMNET"\r\n
void 	gprs_apn()
{
	GPS_TxString("gprs_apn()\n");
	if(gprs_state != STACK)
		return;

	GPS_TxString(rev_buf);
	GPS_TxString("\n");

	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{
	    clear_rev_buf();
		retry_count=0;
	    gprs_state=APN;
		GPRS_TxString("AT+CGDCONT=1,") ;	
		GPRS_TxByte('"');
		GPRS_TxString("IP");
		GPRS_TxByte('"');
		GPRS_TxByte(',');
		GPRS_TxByte('"');					 
		GPRS_TxString("CMNET" );	 
		GPRS_TxByte('"');
		GPRS_TxString("\r\n" );	
		delay(500);
			
	}
	else
	{
      	clear_rev_buf();
	    gprs_state=STACK;
		GPRS_TxString("AT+XISP=0\r\n");
		delay(500);
		retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}
}

//检查网络状态	指令 AT+CREG?\r\n
void 	gprs_check_net()
{
	GPS_TxString("gprs_check_net()\n");
	if(gprs_state != APN)
		return;

	GPS_TxString(rev_buf);
	GPS_TxString("\n");

	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{
		clear_rev_buf();
		retry_count=0;
		gprs_state=CHECK_NET;
		GPRS_TxString("AT+CREG?\r\n");		  
		delay(500);
	}
	else
	{
     	clear_rev_buf();
		gprs_state=APN;
		GPRS_TxString("AT+CGDCONT=1,") ;	
		GPRS_TxByte('"');
		GPRS_TxString("IP");
		GPRS_TxByte('"');
		GPRS_TxByte(',');
		GPRS_TxByte('"');					 
		GPRS_TxString("CMNET" );	 
		GPRS_TxByte('"');
		GPRS_TxString("\r\n" );	
		delay(1000);
		retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}

}

//初始化PPP连接	指令 AT+XIIC=1\r\n
void 	gprs_init_ppp()
{
	GPS_TxString("gprs_init_ppp()\n");
	if(gprs_state != CHECK_NET)
		return;

	GPS_TxString(rev_buf);
	GPS_TxString("\n");	

	if(rev_buf[16] == 'O' && rev_buf[17]=='K')
	{	   	
		clear_rev_buf();
		retry_count=0;
		gprs_state=INIT_PPP;
		GPRS_TxString("AT+XIIC=1\r\n");		  
		delay(500);
	}
	else
	{
	  	clear_rev_buf();
		gprs_state=CHECK_NET;
		GPRS_TxString("AT+CREG?\r\n");		  
		delay(500);
	 	retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}
}

//检查PPP连接状态	指令AT+XIIC?\r\n
void 	gprs_check_ppp()
{
	GPS_TxString("gprs_check_ppp()\n");
	if(gprs_state != INIT_PPP)
		return;

	GPS_TxString(rev_buf);
	GPS_TxString("\n");

	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{  
		clear_rev_buf();
		retry_count=0;
		gprs_state=CHECK_PPP;
		GPRS_TxString("AT+XIIC?\r\n");	   	  
		delay(500);
	}
	else
	{  	clear_rev_buf();
		gprs_state=INIT_PPP;
		GPRS_TxString("AT+XIIC=1\r\n");		  
		delay(500);
	 	retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}

}

//建立连接	指令 AT+TCPSETUP=0,120.27.125.31,80\r\n
void 	gprs_setup_link()
{
	GPS_TxString("gprs_setup_link()\n");
	if(gprs_state != CHECK_PPP)
		return;
		
	GPS_TxString(rev_buf);
	GPS_TxString("\n");
			 
	if(rev_buf[12] == '1' )
	{		
		clear_rev_buf();
		retry_count=0;
		gprs_state=LINK;
		GPRS_TxString("AT+TCPSETUP=0,120.27.125.31,80\r\n");		  
		delay(1000);
	}
	else
	{
	   	clear_rev_buf();
		gprs_state=CHECK_PPP;
		GPRS_TxString("AT+XIIC? \r\n");	   	  
		delay(1000);
	 	retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}

}

void 	gprs_link_finish()
{
	GPS_TxString("gprs_link_finish()\n");
	if(gprs_state != LINK)
		return;	
	
	GPS_TxString(rev_buf);
	GPS_TxString("\n");

	if(rev_buf[20] == 'O' && rev_buf[21]=='K')
	{
		retry_count=0;
		gprs_state=LINK_FINISH;
	}
	else
	{
		gprs_state=LINK;
		GPRS_TxString("AT+TCPSETUP=0,120.27.125.31,80\r\n");		  
		delay(1000);
	 	retry_count++;
		if(retry_count == 5)
		{
		  	GPRS_TxString("AT+XIIC=0\r\n");		  
			delay(500);
		  	gprs_state = RST ;
		  	retry_count=0;
		}
	}
	clear_rev_buf();
}

//char a[13] = "1gprs_state=0";
// gprs模块初始化函数
void 	gprs_start()
{
	GPS_TxString("gprs_s()\n");
	gprs_state=RST;
	clear_rev_buf();
	num = 0;
	
	while(1)
	{
		delay(2000);
		gprs_at() ;
		gprs_check_ccid();
		gprs_stack();
		gprs_apn();
		gprs_check_net();
		gprs_init_ppp();
		gprs_check_ppp();
		gprs_setup_link();
		gprs_link_finish();
		delay(2000);

		GPS_TxByte(gprs_state);
		delay(2000);
		GPS_TxString("again");
		if(gprs_state== LINK_FINISH)
			return;
	}	 
	
}

//解析接收到的数据
void 	GPRS_RECEIVE(uchar revBuf[])	//gprs ---> mcu
{
    uchar *p;
	uint8 len=0;
	uint8 index=0;
	if(revBuf[0]=='\0')
		return;

	p=strstr(revBuf,"TCPRECV");//返回匹配点以后的字符串
	if(p != NULL)
	{
		p=strstr(p,",");//第一个逗号
		if(p!=NULL)	 
		{
			p++;
			//两个逗号之间是长度
		    p=strstr(p,",");
			if(p!=NULL)	 //第二个逗号
			{
			    p++;
	            while(*p!= '\0')
				GPRS_TxByte(*p++);
			}

		}
	}
	else
	{

	}
	clear_rev_buf();
}

//发送数据
void 	sendData(uchar buf[])
{
	uint8 index=0,length=0,tmp ; 
	
	while(buf[length ] != '\0')
	    length++;
	
	if(length == 0)
		return;
	
	tmp=length;
	//send command
	GPRS_TxString("AT+TCPSEND=0,"); 

	if(length >=10)
	{
		uchar shi,ge;
		shi= length /10 +'0';
		ge= length %10 +'0';
		GPRS_TxByte(shi);
		GPRS_TxByte(ge);
	}
	else
	{
	  length=length+'0';
	  GPRS_TxByte(length);
	}
	GPRS_TxString("\r\n");
	
	delay(50);
	for(index=0;index<tmp;index++)	 
	{
		GPRS_TxByte(buf[index]);
	}

	GPRS_TxString("\r\n");

}


//设置接收数据
void	setRevBuf(uchar b)
{
	if(num==MAX_LEN-1)
	{
		num = 0;
		return;
	}
    rev_buf[num++] = b;
}