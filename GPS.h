#ifndef __GPS_H_
#define __GPS_H_

#define uchar unsigned char
#define uint  unsigned int

typedef struct{
	int year;  
	int month; 
	int  day;
	int hour;
	int minute;
	int second;
}DATE_TIME;

typedef xdata struct{
	double  latitude;  //经度
	double  longitude; //纬度
	int     latitude_Degree;	//度
	int		latitude_Cent;		//分
	int   	latitude_Second;    //秒
	int     longitude_Degree;	//度
	int		longitude_Cent;		//分
	int   	longitude_Second;   //秒
	float 	speed;      //速度
	float 	direction;  //航向
	float 	height_ground;    //水平面高度
	float 	height_sea;       //海拔高度
	int     satellite;
	uchar 	NS;
	uchar 	EW;
	DATE_TIME D;
}GPS_INFO;

int GPS_RMC_Parse(char *line,GPS_INFO *GPS);//解析坐标

void Int_To_Str(int x,char *Str);

#endif  //__GPS_H_