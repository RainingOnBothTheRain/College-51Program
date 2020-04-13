//8位模数转换,ADC0809的0通道中断式A/D转换,T8接T3,测T8、T6间的电压
#include <reg52.h>              //包含头文件reg52.h
#include <absacc.h>             //包含头文件absacc.h
#define D_port XBYTE [0x7FFF]   //数码管段码锁存器U2端口地址,A15为线选地址
#define B_port XBYTE [0xBFFF]   //数码管位码锁存器U3端口地址,A14为线选地址
#define AD0    XBYTE [0xF0FF]   //U5通道0的端口地址,A11为线选地址,A10A9A8选0通道
#define AD1    XBYTE [0xF1FF]   //U5通道1的端口地址,A11为线选地址,A10A9A8选1通道
typedef unsigned char u8;       //无符号字符型变量新表示方法定义
typedef unsigned int u16;       //无符号整型变量新表示方法定义
bit     new_cycle_flag=0;       //有新采样数据位标志定义(1是有新采样数据)及初始化
u8      r_kT,Ax;                //AD转换结果和中间变量定义
char    buf[8],m_kT;            //显示缓冲数组和输入理论电压的10倍整数部分定义
u16     temp1;                  //中间量定义
float   VT8;                    //0通道AD输入理论电压的10倍定义-11.0V～11.0V
u8      M=0;                    //扫描显示位计数变量定义及初始化
u8 code Stab[16]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,
                  0x7c,0x39,0x5e,0x79,0x71};//0～F的共阴段码表 
u8 code Btab[8]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};//共阴位码表
/***********T0T1初始化***********/
void  T0T1_init(void)           //8个数码管,每个显示时间为采样周期的1/8
{ TMOD=0x21;                    //T0为16位定时、T1为8位自动重装定时
  SCON=0x40;                    //8位UART,波特率可变(2^1*T1溢出率/32)
  PCON=0x80;                    //使能波特率倍速位SMOD
  TH0=0xfb;                     //高字节,AD采样周期10ms的时间常数fb80H
  TL0=0x80;                     //低字节,(65536-fb80H)/(11.0592/12)=1250微秒
  TH1=0xfd;                     //T1定时值,(2^SMOD)*307200/32=19200bps
  TL1=0xfd;                     //波特率=2*307200/32=19200bps
  EA=1;                         //允许中断
  ET0=1;                        //允许T0中断
  TR0=1;                        //启动T0定时
  TR1=1;                        //启动T1定时
}
/****AD数据的显示段码信息更新****/
void  disp_g(void)              //AD输入电压理论值的显示段码信息更新
{ if(m_kT<0)                    //如果是负数
  { Ax=-m_kT;                   //取绝对值
    buf[7]=0x40;                //符号位送负号段码
  }
  else                          //否则就是正数
  { Ax=m_kT;                    //取要显示的物理量
    buf[7]=0x00;                //符号位送正号段码
  }
  buf[4]=Stab[Ax%10];           //电压小数位段码信息更新
  buf[5]=Stab[Ax/10%10]|0x80;   //电压个位段码携小数点信息更新
  buf[6]=Stab[Ax/100];          //电压十位段码信息更新
}
void  disp_f(void)              //AD数据的显示段码信息更新
{ buf[0]=0x76;                  //后缀位送十六进制后缀H的段码
  Ax=r_kT&0x0f;                 //取要显示的物理量
  buf[1]=Stab[Ax];              //8位AD数据的低4位对应十六进制数段码
  Ax=(r_kT&0xf0)>>4;            //取要显示的物理量
  buf[2]=Stab[Ax];              //8位AD数据的高4位对应十六进制数段码
  buf[3]=0x00;                  //符号位送正号段码
}
/*************主程序*************/
void  main(void)
{ T0T1_init();                  //T0、T1初始化程序(11.0592MHz)
  IT0=1;                        //外部中断引脚INT0以下降沿方式触发中断请求
  EX0=1;                        //允许外部INT0申请中断
  do
  { if(new_cycle_flag==1)       //有新数据,Vs1=-(R5/R4)VT8=-(20K/100K)VT8=-0.2VT8
    { VT8=(float)r_kT;          //Vs2=-(R9/R7)Vs1-(R9/R8)(-5)=-(100K/100K)Vs1+(100K/200K)5=2.5+0.2VT8
      VT8*=0.9765625;           //当VT8=-10V时,Vs2=0.5V;当VT8=10V时,Vs2=4.5V,
      VT8-=125;                 //N=(256/Vref)Vs2=(256/5)(2.5+0.2VT8);VT8=(N*Vref)/(0.2*256)-2.5/0.2
      m_kT=(char)VT8;           //理论显示数字量部分m_kT=10*VT8=0.9765625N-125;小数点固定在第5位
      SBUF=r_kT;                //发送转换结果
      disp_g();                 //AD输入电压理论值的显示段码信息更新
      disp_f();                 //AD数据的显示段码信息更新
      new_cycle_flag=0;         //新采样数据标志清0
    }
  }while(1);
}
/***********T0中断服务程序***********/
void t0(void) interrupt 1 using 1//1.25ms中断1次，每次均要进行显示处理
{ TH0=0xfb;                     //T0时间常数高字节重装
  TL0=0x80;                     //T0时间常数低字节重装
  if(M==8)                      //10ms到了吗？因为1.25ms*8=10ms
  { AD0=0x00;                   //到了10ms，启动0通道AD转换
    M=0;                        //M清0
  }
  B_port=0xff;                  //关闭显示
  D_port=buf[M];                //对应数码管的段码值送给段码端口U2
  B_port=Btab[M];               //对应数码管的位码值送给位码端口U3
  M++;                          //修改到下一次要显示的数码管
}
/********INT0中断服务程序********/
void int0(void) interrupt 0 using 2 //U5的EOC变1,U6C取非后变0,申请中断
{ r_kT=AD0;                     //读取AD结果
  new_cycle_flag=1;             //置有新采样数据标志
}