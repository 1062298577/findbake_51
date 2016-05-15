#include "GPRS.h"
#include<STC12C5A60S2.h>
#include "string.h"


#define S2TI 		0x02	//����2�����ж������־λ 

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

#define MAX_LEN 	128	  	//������󳤶�

#define	TI2				(S2CON & S2TI)
#define	CLR_TI2()		S2CON &= ~S2TI

uchar 	gprs_state;			//��ʼ��״̬��־λ
uint8 	num;				//����

uchar  	xdata	rev_buf[MAX_LEN]; 	//���ջ���
uchar 	xdata	send_buf[MAX_LEN];	//���ͻ���
uchar	xdata	ccid[20];			//�洢CCID   	 

uchar 	retry_count=0; 				//���Դ������  


/*=========��ʱ=====================*/
void 	delay(int i)               
{
	int j,k;
	for(j=0;j<i;j++)
		for(k=0;k<990;k++);
}

//GPRS�ڷ�������
void	GPRS_TxByte(uchar dat)
{
	SBUF = dat;	   	//�����յ����ݷ��ͻ�ȥ
	while(TI == 0);	//��鷢���жϱ�־λ
	TI = 0;			//����жϱ�־λΪ0��������㣩
}

//GPRS�ڷ���һ���ַ���
void 	GPRS_TxString(char *puts)		
{
    while(*puts != 0)  
		GPRS_TxByte(*puts++); 	//����ֹͣ��0����
}

//GPS����ڷ������� ���ڵ�������
void 	GPS_TxByte(uchar c)
{
	S2BUF = c;
	while(!TI2);
	CLR_TI2();
}

//GPS�ڷ���һ���ַ���
void 	GPS_TxString(char *puts)		
{
    while(*puts != 0)  
		GPS_TxByte(*puts++); 	//����ֹͣ��0����
}

//������ջ���
void 	clear_rev_buf()
{
	uint8 index=0;
	num = 0;
	 
	for(index=0;index<MAX_LEN;index++)
	{
		rev_buf[index]='\0';
	}       
}

//���ATָ���Ƿ���� ָ��AT\r\n
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

//���SIM����CCID��	ָ��AT+CCID\r\n
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

//�������״̬	ָ�� AT+CREG?\r\n
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
		//�洢CCID��
		for(index = 0 ; index < 20 ; index ++)
		{
			ccid[index] = rev_buf[index + 9];//�ӵ�9����ʼ
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

//����Ϊ�ڲ�Э�� ָ��AT+XISP=0\r\n
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

//���÷��ʽڵ�APN	ָ�� AT+CGDCONT=1,"IP","CMNET"\r\n
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



//��ʼ��PPP����	ָ�� AT+XIIC=1\r\n
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

//���PPP����״̬	ָ��AT+XIIC?\r\n
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

//�ر�0��·	ָ�� AT+TCPCLOSE=0\r\n
void 	gprs_close_r0()
{
//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//��ӡ����	

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

//��������	ָ�� AT+TCPSETUP=0,120.27.125.31,80\r\n
void 	gprs_setup_link()
{
//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//��ӡ����	


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

//�鿴��·0״̬	ָ�� AT+IPSTATUS=0\r\n
void 	gprs_check_r0()
{
	uchar i = 0;

//	uchar xx[8] = "g_s:0\n"; 
//	xx[4] = gprs_state;
//	GPS_TxString(xx);//��ӡ����

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
		for(i=0;i<MAX_LEN;i++)	   //�ж�+IPSTATUS:0,CONNECT,TCP,2047
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
//	GPS_TxString(xx);//��ӡ����
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
		for(i=0;i<MAX_LEN;i++)	   //�ж�+IPSTATUS:0,CONNECT,TCP,2047
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
// gprsģ���ʼ������
void 	gprs_start()
{
	uchar plusIndex = 0;//��¼����+�ŵ�λ��
	uchar ready = 1;
	uchar i = 0;
	
	GPS_TxString("gprs_start()\n");
	gprs_state=RST;
	num = 0;
	clear_rev_buf();	
	 	
	//����Ƿ�Ӳ��������ɣ����Ƿ���յ�+PBREADY
	while(!ready){	//һֱѭ�����
		delay(2000);
		GPS_TxString("check gsm ready()\n");
		//GPS_TxString(rev_buf);
		if(rev_buf[0] == '\0')
		{
			continue;
		}
		for(i = 0;i < MAX_LEN;i++)
		{
			//�˴���ƥ����+......Y�ĸ�ʽ
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
	
	//�������ʼ��
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

//�������յ�������
//void 	GPRS_RECEIVE(uchar revBuf[])	//gprs ---> mcu
//{
//    uchar *p;
//	uint8 len=0;
//	uint8 index=0;
//	if(revBuf[0]=='\0')
//		return;
//
//	p=strstr(revBuf,"TCPRECV");//����ƥ����Ժ���ַ���
//	if(p != NULL)
//	{
//		p=strstr(p,",");//��һ������
//		if(p!=NULL)	 
//		{
//			p++;
//			//��������֮���ǳ���
//		    p=strstr(p,",");
//			if(p!=NULL)	 //�ڶ�������
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

//��������
void 	sendData(uchar buf[])
{
	uint8 index=0,length=0,tmp ; 
	gprs_state = CHECK_PPP;
	//�ر���·����������
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

	//��ʼ����
	//EA = 0;	//�жϹر�

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


//���ý�������
void	setRevBuf(uchar b)
{
	if(num==MAX_LEN-1)
	{
		num = 0;
		return;
	}
    rev_buf[num++] = b;
}

//��ȡCCID��
unsigned 	char*	getCcid()
{
	return ccid;
}		