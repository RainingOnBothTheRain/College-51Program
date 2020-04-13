#include <reg52.H>              //包含头文件reg52.H
#include <absacc.h>             //包含头文件absacc.h
#define DA     XBYTE [0xEFFF]   //DA转换芯片U8数据输入端口地址
unsigned char i=0;              //波形数据计数器定义
/***********T0T1初始化***********/
void  T0T1_init(void)           //8个数码管,每个显示时间为扫描显示周期的1/8
{ TMOD=0x22;                    //T0为8位定时自动重装定时、T1为8位自动重装定时
  TH0=0xb8;                     //高字节,DA刷新周期0.078125ms的时间常数b8H
  TL0=0xb8;                     //低字节,(256-b8H)/(11.0592/12)=78.125微秒
  EA=1;                         //允许中断
  ET0=1;                        //允许T0中断
  TR0=1;                        //启动T0定时
}
/*************主程序*************/
void  main(void)
{ T0T1_init();                  //T0、T1初始化程序(11.0592MHz)
  do
  {
  }while(1);
}
/*********T0中断服务程序*********/
void t0(void) interrupt 1 using 1//0.078125ms中断1次
{ 
  i++;                           //8位波形计数器自动递增
  DA=i;
}