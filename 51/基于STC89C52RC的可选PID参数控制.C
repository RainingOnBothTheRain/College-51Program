//2018/12/27   备注学习

  //SW3_SW2_SW1   Kp      Ki       Kd
  // on  on  on   0.10    0.00167  0.833   Kp=0.1,Ti=0.6,Td=0.08333
  // on  on off   0.20    0.00334  1.67    Kp=0.2,Ti=0.6,Td=0.08333
  // on off  on   0.50    0.00835  4.17    Kp=0.5,Ti=0.6,Td=0.08333
  // on off off   1.00    0.0167   8.33    Kp=1.0,Ti=0.6,Td=0.08333
  //off  on  on   0.10    0.00625  0.900   Kp=0.1,Ti=0.016,Td=0.09
  //off  on off   0.20    0.0133   2.00    Kp=0.2,Ti=0.1504,Td=0.1
  //off off  on   0.50    0.0357   5.50    Kp=0.5,Ti=0.1401,Td=0.11
  //off off off   1.00    0.0769   12.0    Kp=1.0,Ti=0.13,Td=0.12
#include <reg52.h>              //包含头文件reg52.h
#include <absacc.h>             //包含头文件absacc.h
#define D_port XBYTE [0x7FFF]   //数码管段码锁存器U2端口地址,A15为线选地址
#define B_port XBYTE [0xBFFF]   //数码管位码锁存器U3端口地址,A14为线选地址
#define AD0    XBYTE [0xF0FF]   //U5通道0的端口地址,A11为线选地址,A10A9A8选0通道
#define AD1    XBYTE [0xF1FF]   //U5通道1的端口地址,A11为线选地址,A10A9A8选1通道
#define DA     XBYTE [0xEFFF]   //外部DA转换数据输入端口地址
typedef unsigned char u8;       //无符号字符型变量新表示方法定义
#define A0r  -9.37              //25#实验板的实验4的0通道
#define Amr  9.81               //25#实验板的实验4的0通道
#define N0r  0x20               //25#实验板的实验4的0通道
#define Nmr  0xe0               //25#实验板的实验4的0通道
#define A0m  -9.59              //25#实验板的实验4的1通道
#define Amm  9.65               //25#实验板的实验4的1通道
#define N0m  0x20               //25#实验板的实验4的1通道
#define Nmm  0xe0               //25#实验板的实验4的1通道
#define Any -8.29               //25#实验板的实验2数据
#define Apy 7.84                //25#实验板的实验2数据
#define Nny 0x20                //25#实验板的实验2数据
#define Npy 0xe0                //25#实验板的实验2数据
  sbit    SW8 = P1^7;             //ON为PID控制，OFF为不做任何控制算法，修正后误差送DA
bit     new_cycle_flag=0;       //有新采样数据位标志定义(1是有新采样数据)及初始化
u8      rkT,mkT;                //两个通道AD转换结果定义
char    buf[8],Ax;              //显示缓冲数组和输入理论电压的10倍整数部分定义
float   Axr,a1r,b1r;            //给定量电压及其标度变换的斜率和截距定义
float   Axm,a1m,b1m;            //反馈量采样电压及其标度变换的斜率和截距定义
float   ekT,ekT_T;              //当前电压误差、上次电压误差浮点数定义(伏特)
float   pPkT,pIkT,pDkT,pIkT_T,pkT;//当前各分量、上次积分分量及总控制量定义
float   Kp,Ki,Kd;               //PID校正系数定义
float   a2,n0;                  //输出电压及其标度变换的斜率和截距定义
u8      DAkT;                   //实际DA操作的数字量
u8      M=0,No_ch=0;            //扫描显示位计数、当前A/D通道变量定义及初始化
u8 code Stab[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};//0～9的共阴段码表
u8 code Btab[8]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};//共阴位码表
  //利用拨码选择参数
  float code  Kptable[8]={0.1,0.2,0.5,1.0,0.1,0.2,0.5,1.0};  
  float code  Kitable[8]={0.00167,0.00334,0.00835,0.0167,0.00625,0.0133,0.0357,0.0769};
  float code  Kdtable[8]={0.833,1.67,4.17,8.33,0.900,2.00,5.50,12.0};
/**********标度变换初始化**********/
void Scaleconversion_value_init(void)              //给定量、反馈量、输出控制量标度变换及反变换
{ Axr = Amr ,Axm = Nmr ;
  Axr -= A0r , Axm -= N0r ;
  a1r = Axr  / Axm ;
  b1r = -a1r ;
  b1r *= N0r ;
  b1r += A0r ;  //教材中的式(6-18)的斜率a1和截距b1
  Axr = Amm ,Axm = Nmm ;
  Axr -= A0m , Axm -= N0m ;
  a1m = Axr  / Axm ;
  b1m = -a1m ;
  b1m *= N0m ;
  b1m += A0m ;  //教材中的式(6-18)的斜率a1和截距b1
  Axm = Npy , Axr = Apy ;
  Axm -= Nny , Axr -= Any ;
  a2 = Axm / Axr ;
  n0 = -a2 ;
  n0 *= Any ;
  n0 += Nny ;
}
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
  ES=0;                         //禁止串行发送结束中断
  ET0=1;                        //允许T0中断
  TR0=1;                        //启动T0定时
  TR1=1;                        //启动T1定时
}
/*********位置式PID控制计算********/
void  PID(void)                 //采用与教材中的式(8-15)相同的PID算法
{ pPkT=Kp*ekT;                  //控制量的比例部分计算
  pIkT=pIkT_T;                  //控制量的积分部分计算
  pIkT+=Ki*ekT;                 //累加当前积分部分
  pDkT=Kd*(ekT-ekT_T);          //控制量的微分部分计算
  pkT=pPkT+pIkT+pDkT;           //教材中的式(8-15)相同
}
/****AD数据的显示段码信息更新****/
void  disp_g(void)              //AD输入给定值的显示段码信息更新
{ Ax=(u8)(Axr*10);              //给定电压放大10倍以便显示
  buf[7]=0x00;                  //符号位送正号段码
  if(Ax<0)                      //如果是负数
  { Ax=-Ax;                     //取绝对值
    buf[7]=0x40;                //符号位送负号段码
  }
  buf[4]=Stab[Ax%10];           //给定电压小数位段码信息更新
  buf[5]=Stab[Ax/10%10]|0x80;   //给定电压个位段码携小数点信息更新
  buf[6]=Stab[Ax/100];          //给定电压十位段码信息更新
}
void  disp_f(void)              //反馈值显示段码信息更新
{ Ax=(u8)(Axm*10);              //反馈电压放大10倍以便显示
  buf[3]=0x00;                  //符号位送正号段码
  if(Ax<0)                      //如果是负数
  { Ax=-Ax;                     //取绝对值
    buf[3]=0x40;                //符号位送负号段码
  }
  buf[0]=Stab[Ax%10];           //反馈电压小数位段码信息更新
  buf[1]=Stab[Ax/10%10]|0x80;   //反馈电压个位段码携小数点信息更新
  buf[2]=Stab[Ax/100];          //反馈电压十位段码信息更新
}
/*************主程序*************/
void  main(void)
{ //Kp=1.00;                      //设置PID参数Kp
  //Ki=0.0769;                    //设置PID参数Ki
  //Kd=12.0;                      //设置PID参数Kd
  
    Kp=Kptable[P1&0x07];          //利用网络标号SW3~SW1设置PID参数Kp
    Ki=Kitable[P1&0x07];          //利用网络标号SW3~SW1设置PID参数Ki
    Kd=Kdtable[P1&0x07];          //利用网络标号SW3~SW1设置PID参数Kd
	
  Scaleconversion_value_init(); //标度变换初始化，计算采样0通道的斜率a1和截距b1
  
  ekT_T=0.0,pIkT_T=0.0;         //上次误差和上次控制量积分部分必须初始化为0.0
  
  T0T1_init();                  //T0、T1初始化程序(11.0592MHz)
  IT0=1;                        //外部中断引脚INT0以下降沿方式触发中断请求
  EX0=1;                        //允许外部INT0申请中断
  do
  { if(new_cycle_flag==1)       //有新数据,Vs1=-(R5/R4)VT8=-(20K/100K)VT8=-0.2VT8
    { Axr=(float)rkT;           //
      Axr*=a1r;                 //给定电压标度变换的乘法
      Axr+=b1r;                 //给定电压标度变换的加法
      Axm=(float)mkT;           //
      Axm*=a1m;                 //反馈电压标度变换的乘法
      Axm+=b1m;                 //反馈电压标度变换的加法
	  
      ekT=Axr-Axm;              //计算当前误差电压，以伏特为单位
      ekT*=a2;                  //先做标度反变换的乘法                 先乘后加？
        pkT=ekT;                  //假设SW4置在OFF状态，不做任何控制
        if(SW8==0) PID();         //如果SW4实际设置在ON状态，做PID控制
      pkT+=n0;                  //标度反变换的加法
	  
      if(pkT>254.5)  DAkT=255;  //DA输出的修正值超大处理，考虑四舍五入
      else if(pkT<0.5) DAkT=0;  //DA输出的修正值为负处理，考虑四舍五入
      else            DAkT=(u8)(pkT+0.5);//DA输出的修正值适合DA输出的处理，考虑四舍五入
      DA=DAkT ;                 //AD值修正后DA输出
	  
      TI=0;                     //清除发送结束状态标志
      ES=1;                     //允许串行发送结束中断
      SBUF=rkT;                 //发送给定量A/D转换结果
      
	  ekT_T=ekT,pIkT_T=pIkT;    //迭代移位，为下一次控制做准备
      disp_g();                 //0通道AD输入电压显示段码信息更新
      disp_f();                 //1通道AD输入电压显示段码信息更新
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
    No_ch=0;                    //目前是启动的0通道A/D
    M=0;                        //M清0
  }
  B_port=0xff;                  //关闭显示
  D_port=buf[M];                //对应数码管的段码值送给段码端口U2
  B_port=Btab[M];               //对应数码管的位码值送给位码端口U3
  M++;                          //修改到下一次要显示的数码管
}
/********INT0中断服务程序********/
void int0(void) interrupt 0 using 2 //U5的EOC变1,U6C取非后变0,申请中断
{ if(No_ch==0)                  //是启动的0通道A/D
  { rkT=AD0;                    //读取0通道AD结果
    No_ch=1;
    AD1=0x00;                   //启动1通道A/D
  }
  else
  { mkT=AD1;                    //读取1通道AD结果
    new_cycle_flag=1;           //置有新采样数据标志
  }
}
/*********TXD中断服务程序*********/
void txd(void) interrupt 4 using 3 //每10ms需发送2字节，第2字节靠串行中断完成
{ TI=0;                         //清除发送结束状态标志
  ES=0;                         //发送到最后1个字节，禁止发送结束中断
  SBUF=mkT;                     //发送报文第2字节，反馈量A/D值
}