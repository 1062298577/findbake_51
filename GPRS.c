#include "GPRS.h"
#include<STC12C5A60S2.h>
#include "string.h"


#define S2TI 		0x02	//串口2发送中断请求标志位 

#define RST      	48
#define AT     		49
#define CCID  		50
#define CHECK_NET 	51
#define STACK     	52
#define APN     	53
#define INIT_PPP    54
#define CHECK_PPP   55
#define CLOSE     	56
#define LINK     	57
#define CONNECT 	65
#define LINK_FINISH 66	  

#define MAX_LEN 	128	  	//缓存最大长度

#define	TI2				(S2CON & S2TI)
#define	CLR_TI2()		S2CON &= ~S2TI

uchar 	gprs_state;			//初始化状态标志位
uint8 	num;				//计数

uchar  	xdata	rev_buf[MAX_LEN]; 	//接收缓存
uchar 	xdata	send_buf[MAX_LEN];	//发送缓存
uchar	xdata	ccid[20];			//存储CCID   	 

uchar 	retry_count=0; 				//重试次数标记  


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
	GPS_TxString("init AT\n");
	//GPS_TxString("2:gprs_at()\n");

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");

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
	//GPS_TxString("3:gprs_check_ccid()\n");
	if(gprs_state != AT)
	{
		return;
	}
	
	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");
	
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

//检查网络状态	指令 AT+CREG?\r\n
void 	gprs_check_net()
{
	uchar index = 0;
	//GPS_TxString("4:gprs_check_net()\n");
	if(gprs_state != CCID)
		return;

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");

	if(rev_buf[33] == 'O' && rev_buf[34]=='K')
	{
		//存储CCID号
		for(index = 0 ; index < 20 ; index ++)
		{
			ccid[index] = rev_buf[index + 9];//从第9个开始
		}		

		clear_rev_buf();
		retry_count=0;
		gprs_state=CHECK_NET;
		GPRS_TxString("AT+CREG?\r\n");		  
		delay(1000);
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

//设置为内部协议 指令AT+XISP=0\r\n
void 	gprs_stack()
{
	//GPS_TxString("5:gprs_stack()\n");
	if(gprs_state != CHECK_NET)
		return;
	
	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");
			
	if(rev_buf[16] == 'O' && rev_buf[17]=='K')
	{
	    clear_rev_buf();
		retry_count=0;
		gprs_state=STACK;
		GPRS_TxString("AT+XISP=0\r\n");
		delay(1000);
	}
	else
	{	
		clear_rev_buf();
		GPRS_TxString("AT+CCID\r\n");
		gprs_state=CCID;
		delay(1000);
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
	//GPS_TxString("6:gprs_apn()\n");
	if(gprs_state != STACK)
		return;

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");

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
		delay(1000);
			
	}
	else
	{
      	clear_rev_buf();
	    gprs_state=STACK;
		GPRS_TxString("AT+XISP=0\r\n");
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
	//GPS_TxString("7:gprs_init_ppp()\n");
	if(gprs_state != APN)
		return;

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");	

	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{	   	
		clear_rev_buf();
		retry_count=0;
		gprs_state=INIT_PPP;
		GPRS_TxString("AT+XIIC=1\r\n");		  
		delay(1000);
	}
	else
	{
	  	clear_rev_buf();
		gprs_state=CHECK_NET;
		GPRS_TxString("AT+CREG?\r\n");		  
		delay(1000);
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
	//GPS_TxString("8:gprs_check_ppp()\n");
	if(gprs_state != INIT_PPP)
		return;

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");

	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{  
		clear_rev_buf();
		retry_count=0;
		gprs_state=CHECK_PPP;
		GPRS_TxString("AT+XIIC?\r\n");	   	  
		delay(1000);
	}
	else
	{  	clear_rev_buf();
		gprs_state=INIT_PPP;
		GPRS_TxString("AT+XIIC=1\r\n");		  
		delay(1000);
	 	retry_count++;
		if(retry_count == 5)
		{
		  gprs_state = RST ;
		  retry_count=0;
		}
	}

}

//关闭0链路	指令 AT+TCPCLOSE=0\r\n
void 	gprs_close_r0()
{
//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//打印进度	

	//GPS_TxString("9:gprs_close_r0()\n");
	if(gprs_state != CHECK_PPP)
		return;
		
	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");
			 
	if(rev_buf[12] == '1' )
	{		
		clear_rev_buf();
		retry_count=0;
		gprs_state=CLOSE;
		GPRS_TxString("AT+TCPCLOSE=0\r\n");		  
		delay(1000);
	}
	else
	{
	   	clear_rev_buf();
		gprs_state=CHECK_PPP;
		GPRS_TxString("AT+XIIC?\r\n");	   	  
		delay(1000);
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
//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//打印进度	


	//GPS_TxString("10:gprs_setup_link()\n");
	if(gprs_state != CLOSE)
		return;
		
	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");
			 
//	if(1)//rev_buf[12] == '1' )
//	{		
		clear_rev_buf();
		retry_count=0;
		gprs_state=LINK;
		GPRS_TxString("AT+TCPSETUP=0,120.27.125.31,80\r\n");		  
		delay(1000);
//	}
//	else
//	{
//	   	clear_rev_buf();
//		gprs_state=CHECK_PPP;
//		GPRS_TxString("AT+TCPCLOSE=0\r\n");	   	  
//		delay(1000);
//	 	retry_count++;
//		if(retry_count == 5)
//		{
//		  gprs_state = RST ;
//		  retry_count=0;
//		}
//	}

}

//查看链路0状态	指令 AT+IPSTATUS=0\r\n
void 	gprs_check_r0()
{
	uchar i = 0;

//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//打印进度

	//GPS_TxString("11:gprs_check_r0()\n");
	if(gprs_state != LINK)
		return;	

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");


	if(rev_buf[2]=='O' && rev_buf[3]=='K')
	{
		retry_count=0;
		gprs_state=CONNECT;
		clear_rev_buf();
		GPRS_TxString("AT+IPSTATUS=0\r\n");		  
		delay(1000);
	}
	else
	{
		for(i=0;i<MAX_LEN;i++)	   //判断+IPSTATUS:0,CONNECT,TCP,2047
		{
			if(rev_buf[i]=='\0')
			{
				break;
			}
			if(rev_buf[i]==',' && i < MAX_LEN - 9 && rev_buf[i+1]=='C' && rev_buf[i+7]=='T' && rev_buf[i+8]==',')
			{
				clear_rev_buf();
				GPS_TxString("link success\n");
				retry_count=0;
				gprs_state=LINK_FINISH;
				return;
			}
		}
		
		clear_rev_buf();
		gprs_state=CLOSE;
		GPRS_TxString("AT+TCPCLOSE=0\r\n");		  
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
}

void 	gprs_link_finish()
{
	uchar i = 0;	
	
//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//打印进度
//	
	//GPS_TxString("12:gprs_link_finish()\n");
	if(gprs_state != CONNECT)
		return;	

	//GPS_TxString(rev_buf);
	//GPS_TxString("\n");

	if(rev_buf[14] == 'O' && rev_buf[15]=='K')
	{
		clear_rev_buf();
		GPS_TxString("link success\n");
		retry_count=0;
		gprs_state=LINK_FINISH;
	}
	else
	{	
		for(i=0;i<MAX_LEN;i++)	   //判断+IPSTATUS:0,CONNECT,TCP,2047
		{
			if(rev_buf[i]=='\0')
			{
				break;
			}
			if(rev_buf[i]==',' && i < MAX_LEN - 9 && rev_buf[i+1]=='C' && rev_buf[i+7]=='T' && rev_buf[i+8]==',')
			{
				clear_rev_buf();
				GPS_TxString("link success\n");
				retry_count=0;
				gprs_state=LINK_FINISH;
				return;
			}
		}
		
		clear_rev_buf();
		GPS_TxString("link failed\n");
		gprs_state=LINK;
		GPRS_TxString("AT+IPSTATUS=0\r\n");		  
		delay(1000);
	 	retry_count++;
		if(retry_count == 5)
		{
		  	GPRS_TxString("AT+XIIC=0\r\n");		  
			delay(500);
		  	gprs_state=RST ;
		  	retry_count=0;
		}
	}
	
}

//char a[13] = "1gprs_state=0";
// gprs模块初始化函数
void 	gprs_start()
{
	uchar plusIndex = 0;//记录遇到+号的位置
	uchar ready = 1;
	uchar i = 0;
	
	GPS_TxString("gprs_start()\n");
	gprs_state=RST;
	num = 0;
	clear_rev_buf();	
	 	
	//检查是否硬件启动完成，即是否接收到+PBREADY
	while(!ready){	//一直循环检测
		delay(2000);
		GPS_TxString("check gsm ready()\n");
		//GPS_TxString(rev_buf);
		if(rev_buf[0] == '\0')
		{
			continue;
		}
		for(i = 0;i < MAX_LEN;i++)
		{
			//此处仅匹配了+......Y的格式
			if(rev_buf[i] == '+')
			{
				plusIndex = i;
			}
			else if(rev_buf[i] == 'Y' && plusIndex != 0 && i == plusIndex + 7)
			{
				ready = 1;
				break;
			}
			else if(plusIndex != 0 && i >= plusIndex + 7)
			{
				plusIndex = 0;
				break;
			}
		}
	}
	
	//按步骤初始化
	while(gprs_state != LINK_FINISH)
	{	
		gprs_at() ;
		gprs_check_ccid();
		gprs_check_net();
		gprs_stack();
		gprs_apn();
		gprs_init_ppp();
		gprs_check_ppp();
		gprs_close_r0();
		gprs_setup_link();
		gprs_check_r0();
		gprs_link_finish();
		
		delay(500);
//		GPS_TxString("state:");
//		GPS_TxByte(gprs_state);
//		GPS_TxString("\n\n");
	}	 
}

//解析接收到的数据
//void 	GPRS_RECEIVE(uchar revBuf[])	//gprs ---> mcu
//{
//    uchar *p;
//	uint8 len=0;
//	uint8 index=0;
//	if(revBuf[0]=='\0')
//		return;
//
//	p=strstr(revBuf,"TCPRECV");//返回匹配点以后的字符串
//	if(p != NULL)
//	{
//		p=strstr(p,",");//第一个逗号
//		if(p!=NULL)	 
//		{
//			p++;
//			//两个逗号之间是长度
//		    p=strstr(p,",");
//			if(p!=NULL)	 //第二个逗号
//			{
//			    p++;
//	            while(*p!= '\0')
//				GPRS_TxByte(*p++);
//			}
//
//		}
//	}
//	else
//	{
//
//	}
//	//clear_rev_buf();
//}

//发送数据
void 	sendData(uchar buf[])
{
	uint8 index=0,length=0,tmp ; 
	gprs_state = CHECK_PPP;
	//关闭链路，重新连接
	while(gprs_state != LINK_FINISH)
	{	
		clear_rev_buf();
		rev_buf[12] = '1';
		gprs_close_r0();
		gprs_setup_link();
		gprs_check_r0();
		gprs_link_finish();
		
		if(gprs_state==RST)
		{
			gprs_start();
		}
		//delay(2000); 
	}	

	//开始发送
	//EA = 0;	//中断关闭

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
	
	//while(rev_buf[2]!='>')
	retry_count = 0;
	while(rev_buf[2] != '>')
	{
		delay(50);
		retry_count++;
		if(retry_count>10)
		{
			return;
		}
		GPS_TxString("wait >\n");
	}
	
	//GPS_TxString(rev_buf);
	EA = 0;
	for(index=0;index<tmp;index++)	 
	{
		GPRS_TxByte(buf[index]);
	}
	GPRS_TxString("\r\n");
	EA = 1;
	//GPRS_TxString(buf);
	//GPRS_TxString("");
	//GPS_TxString("SEND FIN\n");
	//GPS_TxString(rev_buf);
	
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

//获取CCID号
unsigned 	char*	getCcid()
{
	return ccid;
}		