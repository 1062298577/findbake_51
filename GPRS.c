#include "GPRS.h"
#include<STC12C5A60S2.h>
#include "string.h"

#define S2RI 		0x01	//����2�����ж������־λ
#define S2TI 		0x02	//����2�����ж������־λ 

#define RST      	0X01
#define AT     		0X02
#define CCID  		0X03
#define STACK     	0X04
#define APN     	0X05
#define INIT_PPP    0X06
#define CHECK_PPP   0X07
#define LINK     	0X08
#define CHECK_NET 	0x09
#define LINK_FINISH 0x0A	  

#define MAX_LEN 	128	  	//������󳤶�

#define uchar unsigned char
#define uint8 unsigned int

uchar 	gprs_state;			//��ʼ��״̬��־λ
uint8 	num;				//����

uchar  	rev_buf[MAX_LEN]; 	//���ջ���
uchar 	send_buf[MAX_LEN];	//���ͻ���

uchar	ccid[MAX_LEN];		//�洢CCID   	 

uchar 	retry_count=0; 		//���Դ������  


/*=========��ʱ=====================*/
void 	delay(int i)               
{
	int j,k;
	for(j=0;j<i;j++)
		for(k=0;k<990;k++);
}

//GPRS�ڷ�������
void	GPRS_TxByte(unsigned char dat)
{
	SBUF = dat;	   	//�����յ����ݷ��ͻ�ȥ
	while(TI == 0);	//��鷢���жϱ�־λ
	TI = 0;			//����жϱ�־λΪ0��������㣩
}

//GPRS�ڷ���һ���ַ���
void 	GPRS_TxString(unsigned char code *puts)		
{
    for (; *puts != 0;	puts++)  GPRS_TxByte(*puts); 	//����ֹͣ��0����
}

//������ջ���
void 	clear_rev_buf()
{
	uint8 index=0;
	 
	for(index=0;index<MAX_LEN;index++)
	{
		rev_buf[index]='\0';
	}       
}

//���ATָ���Ƿ���� ָ��AT\r\n
void 	gprs_at()
{
	if(gprs_state == RST)
	{	
		clear_rev_buf();
	    retry_count=0;
		GPRS_TxString("AT\r\n");
		gprs_state=AT;
		delay(500);
	}
}

//���SIM����CCID��	ָ��AT+CCID\r\n
void 	gprs_check_ccid()
{
	if(gprs_state == AT)
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

//����Ϊ�ڲ�Э�� ָ��AT+XISP=0\r\n
void 	gprs_stack()
{
	if(gprs_state != CCID)
		return;
			
	if(rev_buf[2] == 'O' && rev_buf[3]=='K')
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

//���÷��ʽڵ�APN	ָ�� AT+CGDCONT=1,"IP","CMNET"\r\n
void 	gprs_apn()
{
	if(gprs_state != STACK)
		return;
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

//�������״̬	ָ�� AT+CREG?\r\n
void 	gprs_check_net()
{
	if(gprs_state != APN)
		return;
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

//��ʼ��PPP����	ָ�� AT+XIIC=1\r\n
void 	gprs_init_ppp()
{
	if(gprs_state != CHECK_NET)
		return;
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

//���PPP����״̬	ָ��AT+XIIC?\r\n
void 	gprs_check_ppp()
{
	if(gprs_state != INIT_PPP)
		return;
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

//��������	ָ�� AT+TCPSETUP=0,120.27.125.31,80\r\n
void 	gprs_setup_link()
{

	if(gprs_state != CHECK_PPP)
		return;	 
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
	clear_rev_buf();
}

// gprsģ���ʼ������
void 	gprs_init()
{
	gprs_state=RST;
	clear_rev_buf();
	clear_send_buf();
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

//�������յ�������
void 	GPRS_RECEIVE(uchar revBuf[])	//gprs ---> mcu
{
    uchar *p;
	uint8 len=0;
	uint8 index=0;
	if(revBuf[0]=='\0')
		return;

	p=strstr(revBuf,"TCPRECV");//����ƥ����Ժ���ַ���
	if(p != NULL)
	{
		p=strstr(p,",");//��һ������
		if(p!=NULL)	 
		{
			p++;
			//��������֮���ǳ���
		    p=strstr(p,",");
			if(p!=NULL)	 //�ڶ�������
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

//��������
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


//���ý�������
void	setRevBuf(uchar b)
{
	if(num==MAX_LEN-1)
	{
		return;
	}
    rev_buf[num++] = b;
}

void	GPRS_StartUp()
{
	gprs_init();
}