#include<reg52.h>
#define unit unsigned int

sbit p10=P0^0;   //ѭ���� ��������0,1,2~  
sbit p11=P0^1;
sbit p12=P0^2;
sbit p13=P0^3;
sbit p14=P0^4;
sbit p15=P0^5;
sbit p16=P0^6;

sbit ENB=P1^0;    //ENB
sbit ENA=P1^1;	  //ENA
sbit p22=P2^2;    //IN1  ��
sbit p23=P2^3;	  //IN2
sbit p24=P2^4;    //IN3	 ��
sbit p25=P2^5;	  //IN4

int num=0,y;

void init()
{ 
  p22=1;
  p23=0;
  p24=1;
  p25=0;
  TMOD=0x01;
  TH0=(65535-100)/256;
  TL0=(65535-100)%256;  //��100usΪ������λ
  EA=1;
   ET0=1;
   TR0=1;
}

void pwm1(int x)       //����pwm
{
	if(num<=50-x)   //������50Ϊ��׼�ٶ�
	ENA=1;
	else
	ENA=0;
}

void pwm2(float z)     //����pwm
{
	if(num<=55-z)   //������55Ϊ��׼�ٶ�
	ENB=1;
	else
	ENB=0;
}

void straight()		 //ֱ��
{
  pwm1(0);
  pwm2(0);
}


void left1()	   //���м�Ϊ��׼����ߵ�1����⵽
{
   pwm1(20);
  pwm2(0);
}


void right1()	   //���м�Ϊ��׼���ұߵ�1����⵽
{
  pwm1(0);
  pwm2(5);
}

void left2()	   //���м�Ϊ��׼����ߵ�2����⵽
{
  pwm1(45);
  pwm2(-25);
}

void right2()	   //���м�Ϊ��׼���ұߵ�2����⵽
{
  pwm1(0);
  pwm2(10);
}


void left3()	   //���м�Ϊ��׼����ߵ�3����⵽
{
  pwm1(55);
  pwm2(-20);
}

void right3()	   //���м�Ϊ��׼���ұߵ�3����⵽
{

  pwm1(0);
  pwm2(50);	
}

void T0_time()interrupt 1
{
  TH0=(65536-100)/256;
  TL0=(65536-100)%256; 
  num++;
  if(num==100)
  {
  	num=0;
  }
}

void run()
{
  if(p10==0)
  	y=0;
  if(p11==0)
  	y=1;
  if(p12==0)
  	y=2;
  if(p13==0)
  	y=3;
  if(p14==0)
  	y=4;
  if(p15==0)
  	y=5;
  if(p16==0)
  	y=6;
	switch(y)
	{
		case 0:left3();break;
		case 1:left2();break;
		case 2:left1();break;
		case 3:straight();break;
		case 4:right1();break;
		case 5:right2();break;
		case 6:right3();break;
		default:
		{
		 pwm1(0);
                 pwm2(0);
		}	
		break;
	}
}

  void main()
{ 
  init();
  while(1)
  {
	  run();
  }
}