#include "GPRS.h"
#include<STC12C5A60S2.h>
#include "string.h"

#define S2RI 		0x01	//����2�����ж������־λ
#define S2TI 		0x02	//����2�����ж������־λ 

#define RST      	0X01
#define AT     		0X02
#define STACK     	0X03
#define APN     	0X04
#define INIT_PPP    0X05
#define CHECK_PPP   0X06
#define LINK     	0X07
#define CHECK_NET 	0x08
#define LINK_FINISH 0x09


#define uchar unsigned char
#define uint8 unsigned int

uchar 	gprs_state;

#define MAX_LEN 128
uchar  	rev_buf[MAX_LEN];        //���ջ���
uchar 	send_buf[MAX_LEN] ;

char 	send_buf_num=2;

uchar 	num_buf=0,num = 0;     	 

uchar 	retry_count=0;   

uchar 	gprs_state;

uchar 	send_flag=0;
uchar 	revc_flag=0;

/*=========��ʱ=====================*/
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
 ��������UART���ڷ����ַ�������
 ��  �ã�UART_TC (?);
 ��  ������ҪUART���ڷ��͵����ݣ�8λ/1�ֽڣ�
 ����ֵ���� 
 ��  �����򴮿ڷ���һ���ַ���,���Ȳ��ޡ�
 ��  ע������UART_TC("d9887321$"); �˺�����Ҫ#include <string.h>ͷ�ļ�֧�֡�
/******************************************************************************************

 ****/

//GPRS�ڷ�������
void	GPRS_TxByte(unsigned char dat)
{
	//B_TI = 0;
	SBUF = dat;
	//while(!B_TI);
	//B_TI = 0;
}

//GPRS�ڷ���һ���ַ���
void 	GPRS_TxString(unsigned char code *puts)		
{
    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//����ֹͣ��0����
}

//������ջ���
void clear_buf()
{
	uint8 index=0;
	 
	for(index=0;index<MAX_LEN;index++)
	{
		rev_buf[index]='\0';
	}
	num = 0;       
}

//���ATָ���Ƿ���� ָ��AT\r\n
void gprs_at()
{
   
	if(gprs_state == RST)
	{	
		clear_buf();
	    retry_count=0;
		GPRS_TxString("AT\r\n");
		gprs_state=AT;
		delay(500);
	}
}

//����Ϊ�ڲ�Э�� ָ��AT+XISP=0\r\n
void gprs_stack()
{
	if(gprs_state != AT)
		return;
			
	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{
	    clear_buf();
		retry_count=0;
		gprs_state=STACK;
		GPRS_TxString("AT+XISP=0\r\n");
		delay(500);
	}
	else
	{	
		clear_buf();
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

//���÷��ʽڵ�APN	ָ�� AT+CGDCONT=1,"IP","CMNET"\r\n
void gprs_apn()
{
	if(gprs_state != STACK)
		return;
	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{
	    clear_buf();
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
      	clear_buf();
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

//�������״̬	ָ�� AT+CREG?\r\n
void gprs_check_net()
{
	if(gprs_state != APN)
		return;
	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{
		clear_buf();
		retry_count=0;
		gprs_state=CHECK_NET;
		GPRS_TxString("AT+CREG?\r\n");		  
		delay(500);
	}
	else
	{
     	clear_buf();
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

//��ʼ��PPP����	ָ�� AT+XIIC=1\r\n
void gprs_init_ppp()
{
	if(gprs_state != CHECK_NET)
		return;
	if(rev_buf[16] == 'O' && rev_buf[17]=='K')
	{	   	
		clear_buf();
		retry_count=0;
		gprs_state=INIT_PPP;
		GPRS_TxString("AT+XIIC=1\r\n");		  
		delay(500);
	}
	else
	{
	  	clear_buf();
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

//���PPP����״̬	ָ��AT+XIIC?\r\n
void gprs_check_ppp()
{
	if(gprs_state != INIT_PPP)
		return;
	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
	{  
		clear_buf();
		retry_count=0;
		gprs_state=CHECK_PPP;
		GPRS_TxString("AT+XIIC?\r\n");	   	  
		delay(500);
	}
	else
	{  	clear_buf();
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

//��������	ָ�� AT+TCPSETUP=0,120.27.125.31,80\r\n
void gprs_setup_link()
{

	if(gprs_state != CHECK_PPP)
		return;	 
	if(rev_buf[12] == '1' )
	{		
		clear_buf();
		retry_count=0;
		gprs_state=LINK;
		GPRS_TxString("AT+TCPSETUP=0,120.27.125.31,80\r\n");		  
		delay(1000);
	}
	else
	{
	   	clear_buf();
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
	//GPRS_TxByte(rev_buf[20] );
	//GPRS_TxByte(rev_buf[21] );
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
	clear_buf();
}

// gprsģ���ʼ������
void gprs_init()
{
	num=0;
	gprs_state=RST;
	clear_buf();
	clear_send_buf(send_buf);
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
	if(rev_buf[0]=='\0')
	return;
/*	for(index=0;rev_buf2[index] != '\0';index++)
	{
	    UART_1SendOneByte(rev_buf2[index] );
	}  	*/

	p=strstr(rev_buf,"TCPRECV");
	if(p != NULL)
	{
		p=strstr(p,",");	//��һ������
		if(p!=NULL)	 
		{
			p++;
			//��������֮���ǳ���
		    p=strstr(p,",");
			if(p!=NULL)	 //�ڶ�������
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

//��������
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

//������ͻ���
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
	if(send_buf[0]==1)
	{
	    send_bu1[1] =1;// now in process
		p=&send_buf[2];
		gprs_send_data(p);
		clear_send_buf(send_buf1);
		send_buf_num=2;
	}
}


//void main(void)
//{
//	//   rev_buf[0]=0;
//	num=0;
//	InitUART();	//���пڳ�ʼ��
//	gprs_state=RST;
//	gprs_init();
// 	clear_buf();
//	clear_send_buf(send_buf);
//	while(1)
//	{
//	 	GPRS_RECEIVE();
//		GPRS_SSEND( );
//		delay(50);
//		LED=~LED;
//	}
//}