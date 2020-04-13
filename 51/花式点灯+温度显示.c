#include<reg52.h>
#include <intrins.h>
#include <temperature.h>

#define uc unsigned char
#define ui  unsigned int 
sbit key1=P2^0;
sbit key2=P2^1;
sbit key3=P2^2;
sbit key4=P2^3;
sbit key5=P2^4;
sbit Bee=P3^2;

//**************单独扫描数码管*****************//
sbit one=P3^4;
sbit two=P3^5;
sbit three=P3^6;
sbit four=P3^7;

uc disp[6];

uc code smgduan[]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
//*******************************************************//

void delay(); //延时函数             花式1延时函数

void Delayms(ui x)			     //花式2,3的延时函数
{
uc t;
while(x--)for(t=0;t<120;t++);
}

void delay_ms(int x)                   //按键及定时器服务函数内延时函数
{
	unsigned char i, j;
	for(i=x;i<0;i--)
		for(j=110;j>0;j--);
}



////********************单独扫描**************************//
//void display()
//{
//     one=0;
//	 P0=disp[1];
//	 delay_ms(1);
//	 P0=0x00;
//	 one=1;
//
//	 two=0;
//	 P0=disp[2];
//	 delay_ms(1);
//	 P0=0x00;
//	 two=1;
//
//	 three=0;
//	 P0=disp[3];
//	 delay_ms(1);
//	 P0=0x00;
//	 three=1;
//
//	 four=0;
//	 P0=disp[4];
//	 delay_ms(1);
//	 P0=0x00;
//	 four=1;
//}
/*************************************************/


/*********************************************/
//定时器刷新方式扫描数码管
unsigned char const wei[4]={0xEF,0xDF,0xBF,0x7F};
unsigned char const duan[20]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71,0x76,0x74,0x38,0x73};// 显示段码值0123456789AbCdEFHhLP对应的数码管点亮
uc data0[4]={0xa0,0xa0,0xa0,0xa0};               //用于定时器刷新取显示数据



/************************
LED花式
**************************/
uc code Pattern_P0[]=
{
0xFC,0xF9,0xF3,0xE7,0xCF,0x9F,0x3F,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xE7,0xD8,0xBD,0x7E,0xBD,0xDB,0xE7,0xFF,0xE7,0xC3,0x81,0x00,0x81,0xC3,0xE7,0xFF,
0xAA,0x55,0x18,0xFF,0xF0,0x0F,0x00,0xFF,0xF8,0xF1,0xE3,0xC7,0x8F,0x1F,0x3F,0x7F,
0x7F,0x3F,0x1F,0x8F,0xC7,0xE3,0xF1,0xF8,0xFF,0x00,0x00,0xFF,0xFF,0x0F,0xF0,0xFF,
0xFE,0xFD,0xFB,0xF7,0xEF,0xDF,0xBF,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7E,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,0xEE,
0xFE,0xFC,0xF8,0xF0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xE0,0xF0,0xFB,0xFC,0xFE,
0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xEF
};
uc code Pattern_P2[]=
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFC,0xF9,0xF3,0xE7,0xCF,0x9F,0x3F,0xFF,
0xE7,0xD8,0xBD,0x7E,0xBD,0xDB,0xE7,0xFF,0xE7,0xC3,0x81,0x00,0x81,0xC3,0xE7,0xFF,
0xAA,0x55,0x18,0xFF,0xF0,0x0F,0x00,0xFF,0xF8,0xF1,0xE3,0xC7,0x8F,0x1F,0x3F,0x7F,
0x7F,0x3F,0x1F,0x8F,0xC7,0xE3,0xF1,0xF8,0xFF,0x00,0x00,0xFF,0xFF,0x0F,0xF0,0xFF,
0xFE,0xFD,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFD,0xFB,0xF7,0xEF,0xDF,0xBF,0x7F,
0x7F,0xBF,0xDE,0xEF,0xF7,0xFB,0xFD,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFC,0xF8,0xF0,0xE0,0xC0,0x80,0x00,
0x00,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF
};

uc time1,temp1,time;
int T; 
   
sbit d0=P1^5;




void BEE()	                //花式结束时蜂鸣
{
   Bee=1;
   delay_ms(300);
   Bee=0;
   delay_ms(1000);
   Bee=1;
   delay_ms(300);
   Bee=0;

}


 //*****************温度显示函数*****************//
void Datapros(int T)	             
{
	 float tp;
	 if(T<0)
	 {
	 	 data0[0]=duan[0x40];
//		 disp[1]=0x40;
		 T=T-1;
		 T=~T;
		 tp=T;
		 T=tp*10*0.0625+0.5;
	 }
	 else
	 {
	 	 data0[0]=duan[0x00];
//		 disp[1]=0x00;
		 tp=T;
		 T=tp*10*0.0625+0.5;
	 }
	 data0[1]=duan[T%1000/100];
	 data0[2]=duan[T%100/10]|0x80;
	 data0[3]=duan[T%10];
//	 disp[2]=smgduan[T%1000/100];
//	 disp[3]=smgduan[T%100/10]|0x80;
//	 disp[4]=smgduan[T%10];
}







void led1()		                      //花式1
{
    int i,j,k,l;
  	for(k=2;k>0;k--)
    {
	 Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
     temp1=0xfe;
     P1=temp1;
	  Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
    for(j=7;j>0;j--)
    {
       Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确 
       temp1=_crol_(temp1,1) ; //移位函数
       delay();
      Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
       P1=temp1;
       };
        temp1=0x7f;
        P1=temp1;
        for(l=7;l>0;l--)
            {
           	 Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
             temp1=_cror_(temp1,1); //移位
             delay();
             P1=temp1; //移位后的数据输出到P1口
			 Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
             }
			
	       for(i=5;i>0;i--) //重复5次
         {
		Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
        P1=0xff;  //灯全灭
        delay();  //等待一段时间
		Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
        P1=0x00;  //灯全亮
        delay();  //等待一段时间
		Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
         } 
      }
}


void led2()	                           //花式2
{
    int i;
  	for(i=0;i<136;i++)
    {
      P1=Pattern_P0[i];
	  Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
      Delayms(150);
    }
}



void led3()	                           //花式3
{	 
    int i;	  	
    for(i=0;i<136;i++)
    {
         P1=Pattern_P2[i];
		 Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
		 Delayms(150);
    }
}

//缺点：供电不足，特别暗
//void display1()	                                    //单独扫描数码管温度显示函数
//{
//  Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
//  display();
//}


void key_control()
{
      Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
    	 if(0==key1)
		{
			delay_ms(8);
			if(0==key1)
			{
			    Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
			  	led1();
				P1=0xff;
				Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
				BEE();
			}
			
			while(!key1);
		}
		Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
       if(0==key2)
		{
			delay_ms(8);
			if(0==key2)
			{
			  	led2();
				P1=0xff;
				Datapros(Ds18b20ReadTemp());
				BEE();	         
			}
			
			while(!key2);
		}
		Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
		if(0==key3)
		{
			delay_ms(8);
			if(0==key3)
			{
			  	led3();
				P1=0xff;
				Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
				BEE(); 
			}
			
			while(!key3);
		}
}


void Init_Timer(void)
{
 TMOD = 0x11;	  //使用模式1，16位定时器，使用"|"符号可以在使用多个定时器时不受影响		     
 TH1=(65535-5000)/256;	      //给定初值，这里使用定时器最大值从0开始计数一直到65535溢出
 TL1=(65535-5000)%256;
 EA=1;            //总中断打开
 ET0=1;           //定时器中断打开
// TR0=1;           //定时器开关打开
 ET1=1;           //定时器中断打开
 TR1=1;           //定时器开关打开
// EX0=1;         //外部中断1开
// IT0=1;         //IT1=1表示边沿触发
}





void main()    
{
    Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
    Init_Timer();
    while(1)
	{		 
	Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
//	display1();
	key_control();
//	display1();
	Datapros(Ds18b20ReadTemp());	           //使温度读取显示更快速准确
//	display1();
	}	
}




void delay() //延时函数
   {

     ui x,y;
   for (x=900;x>0;x--)
    for(y=99;y>0;y--);
   }





void Timer1_isr(void) interrupt 3 using 1			//定时刷新  输出PWM
{
 TH1=(65535-5000)/256;		  //重新赋值
 TL1=(65535-5000)%256;
 P3=wei[time1];
 delay_ms(10);
 P0=data0[time1];//取显示数据
 delay_ms(10);
 time1++;
 if(time1==4)
 {
 time1=0;
 }
//  pwm++;
//  pwm_kongzhi(); 
} 