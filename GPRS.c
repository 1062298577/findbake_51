#include "GPRS.h"
#include<STC12C5A60S2.h>
#include "string.h"


#define S2RI 0x01	//串口2接收中断请求标志位
#define S2TI 0x02	//串口2发送中断请求标志位 

#define RST      0X01
#define AT     0X02
#define STACK     0X03
#define APN     0X04
#define INIT_PPP     0X05
#define CHECK_PPP     0X06
#define LINK     0X07
#define CHECK_NET 0x08
#define LINK_FINISH 0x09


#define uchar unsigned char
#define uint8 unsigned int

sbit LED = P3^4;
uchar gprs_state;
#define MAX_LEN 200
uchar  rev_buf1[MAX_LEN];        //接收缓存
uchar  rev_buf2[MAX_LEN];        //接收缓存


uchar send_buf1[MAX_LEN] ;
uchar send_buf2[MAX_LEN] ;
char send_buf1_num=2;
char send_buf2_num=2;


uchar num_buf1=0,num = 0;     	 

uchar retry_count=0;   

uchar gprs_state;

uchar send_flag=0;
uchar revc_flag=0;

/*=========延时=====================*/
void delay(int i)               
{
	int j,k;
		for(j=0;j<i;j++)
			for(k=0;k<990;k++);
}



/******************************************************************************************

 ****/

/******************************************************************************************

 ***
 函数名：UART串口发送字符串函数
 调  用：UART_TC (?);
 参  数：需要UART串口发送的数据（8位/1字节）
 返回值：无 
 结  果：向串口发送一个字符串,长度不限。
 备  注：例：UART_TC("d9887321$"); 此函数需要#include <string.h>头文件支持。
/******************************************************************************************

 ****/

//GPRS口发送数据
void	GPRS_TxByte(unsigned char dat)
{
	//B_TI = 0;
	SBUF = dat;
	//while(!B_TI);
	//B_TI = 0;
}

//GPRS口发送一串字符串
void 	GPRS_TxString(unsigned char code *puts)		
{
    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//遇到停止符0结束
}

/*
#define RST      0X01
#define AT     0X02
#define STACK     0X03
#define APN     0X04
#define INIT_PPP     0X05
#define CHECK_PPP     0X06
#define LINK     0X07

*/

//检查AT指令是否可用 指令AT\r\n
void gprs_at()
{
   
	if(gprs_state == RST)
	{	clear_buf2();
	    retry_count=0;
		UART2_Write_str("AT\r\n");
		gprs_state=AT;
		delay(500);
	}
	//clear_buf2();
}

//设置为内部协议 指令AT+XISP=0\r\n
void gprs_stack()
{
	if(gprs_state != AT)
		return;
	//LED=~LED;
			
	if(rev_buf2[2] == 'O' && rev_buf2[3]=='K')
	{
	    clear_buf2();
		retry_count=0;
		gprs_state=STACK;
		GPRS_TxString("AT+XISP=0\r\n");
		delay(500);
	}
	else
	{	
		clear_buf2();
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
	//	clear_buf2();
}

//设置访问节点APN	指令 AT+CGDCONT=1,"IP","CMNET"\r\n
void gprs_apn()
{
	if(gprs_state != STACK)
		return;
	if(rev_buf2[2] == 'O' && rev_buf2[3]=='K')
	{
	    clear_buf2();
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
      	clear_buf2();
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
void gprs_check_net()
{
	if(gprs_state != APN)
		return;
	if(rev_buf2[2] == 'O' && rev_buf2[3]=='K')
	{
		clear_buf2();
		retry_count=0;
		gprs_state=CHECK_NET;
		GPRS_TxString("AT+CREG?\r\n");		  
		delay(500);
	}
	else
	{
     	clear_buf2();
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
void gprs_init_ppp()
{
	if(gprs_state != CHECK_NET)
		return;
	if(rev_buf2[16] == 'O' && rev_buf2[17]=='K')
	{	   	
		clear_buf2();
		retry_count=0;
		gprs_state=INIT_PPP;
		GPRS_TxString("AT+XIIC=1\r\n");		  
		delay(500);
	}
	else
	{
	  	clear_buf2();
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
void gprs_check_ppp()
{
	if(gprs_state != INIT_PPP)
		return;
	if(rev_buf2[2] == 'O' && rev_buf2[3]=='K')
	{  
		clear_buf2();
		retry_count=0;
		gprs_state=CHECK_PPP;
		GPRS_TxString("AT+XIIC?\r\n");	   	  
		delay(500);
	}
	else
	{  	clear_buf2();
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
void gprs_setup_link()
{

	if(gprs_state != CHECK_PPP)
		return;	 
	if(rev_buf2[12] == '1' )
	{		
		clear_buf2();
		retry_count=0;
		gprs_state=LINK;
		GPRS_TxString("AT+TCPSETUP=0,120.27.125.31,80\r\n");		  
		delay(1000);
	}
	else
	{
	   	clear_buf2();
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

void gprs_link_finish()
{
	
	if(gprs_state != LINK)
		return;	
	GPRS_TxByte(rev_buf2[20] );
	GPRS_TxByte(rev_buf2[21] );
	if(rev_buf2[20] == 'O' && rev_buf2[21]=='K')
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
	clear_buf2();
}

// gprs模块初始化函数
void gprs_init()
{
	while(1)
	{
		gprs_at() ;
		gprs_stack();
		gprs_apn();
		gprs_check_net();
		gprs_init_ppp();
		gprs_check_ppp();
		gprs_setup_link();
		gprs_link_finish();

		if(	gprs_state== LINK_FINISH)
		return;
	}	 
	
}


uchar re_enter=0;
void GPRS_RECEIVE()	//gprs ---> mcu
{
 	//send rev_buf2 to mcu
    uchar *p;
	uint8 len=0;
	uint8 index=0;
	if(rev_buf2[0]=='\0')
	return;
/*	for(index=0;rev_buf2[index] != '\0';index++)
	{
	    UART_1SendOneByte(rev_buf2[index] );
	}  	*/

	p=strstr(rev_buf2,"TCPRECV");
	if(p != NULL)
	{
		p=strstr(p,",");	//第一个逗号
		if(p!=NULL)	 
		{
			p++;
			//两个逗号之间是长度
		    p=strstr(p,",");
			if(p!=NULL)	 //第二个逗号
			{
			    p++;
	            while(*p!= '\0')
				UART_1SendOneByte(*p++);
			}

		}
	 	//UART_TC("\r\nI  find it! \r\n");
	
	}
	else
	{
	 	//	UART_TC("I cannot find it! \r\n");
	}
	clear_buf2();
}

//发送数据
void gprs_send_data(uchar *buf)
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

//清除发送缓存
void clear_send_buf(uchar *p)
{
    uint8 index=0;
    for(index=0;index<MAX_LEN;index++)	 
	{
		p[index]='\0';
	}
}


void GPRS_SSEND( )  //mcu --->gprs
{
	uchar *p;
	if(send_buf1[0]==1)
	{
	    send_buf1[1] =1;// now in process
		p=&send_buf1[2];
		gprs_send_data(p);
		clear_send_buf(send_buf1);
		send_buf1_num=2;
	    UART_TC("af send  buf1...\r\n");
	}
}

/*****************主函数******************/

void main(void)
{
	//   rev_buf[0]=0;
	num=0;
	InitUART();	//串行口初始化
	gprs_state=RST;
    UART_TC("bf gprs int\r\n");
	gprs_init();
    UART_TC("af gprs int\r\n");
 	clear_buf1();
 	clear_buf2();
	clear_send_buf(send_buf1);
	clear_send_buf(send_buf2);
	while(1)
	{
 //   DTU_A:
	 	GPRS_RECEIVE();
	
		GPRS_SSEND( );
		delay(50);
		LED=~LED;
	}
}
/************串行口1中断处理函数*************/
void UART_1Interrupt(void) interrupt 4
{
	EA=0;
	if(RI==1)
	{
		if(send_buf1[1] !=1)//not in sent process
       	{
	   		send_buf1[send_buf1_num++]=SBUF;
			if(send_buf1_num == MAX_LEN-1)
	   		{
	 			send_buf1[1]=1;
	   		}
			send_buf1[0]=1;// send_buf have data now
 		}
		RI=0;
	}
	EA=1;
}