#include <STC12C5A60S2.H>       //包含头文件STC12C5A60S2.H
unsigned char i;                //扫描显示位计数和波形计数变量定义
/*************PWM初始化*************/
void  PWM_init(void)            //硬件二阶RC低通Tao=1ms>>0.0463ms=0.5*(1/10.8KHz)
{ CMOD=0x8a;                    //Fosc/4,fPWM=Fosc/4/256=10.8KHz,不准中断
  CCON=0x00;                    //00000000B控制,关闭PCA计数,CF\CCF1\CCF0均清0
  CL=0x00;                      //PCA计数器低8位清0
  CCAPM0=0x42;                  //8位PWM无中断,允许比较器,P1.3做PWMO输出,禁止中断
  PCA_PWM0=0x00;                //最低2位EPC0H和EPC0L均设置为0
  CCAP0H=0x80;                  //预送PWM为50%的占空比
  CCAP0L=0x80;                  //预送PWM为50%的占空比
  CR=1;                         //允许PCA计数器工作
}
/*************主程序*************/
void  main(void)
{ PWM_init();                   //PWM初始化程序
  TMOD=0x22;                    //T0为8位定时自动重装定时
  TH0=0xb8;                     //DA刷新周期0.078125ms的时间常数b8H
  TL0=0xb8;                     //低字节,(256-b8H)/(11.0592/12)=78.125微秒
  EA=1;                         //允许中断
  ET0=1;                        //允许T0中断
  TR0=1;                        //启动T0定时
  do
  { 
  }while(1);
}
/*********T0中断服务程序*********/
void t0(void) interrupt 1 using 1//0.078125ms中断1次
{ i++;                           //8位波形计数器自动递增
  CCAP0H=i;                      //
}