//2018/12/27  备注学习   

//SW3_SW2_SW1      Kp
// on  on  on      1.64
// on  on off      1.66
// on off  on      1.68
// on off off      1.685
//off  on  on      1.69
//off  on off      1.695
//off off  on      1.70
//off off off      1.71
#include  <reg52.h>
#include  <absacc.h>
#define uchar unsigned char       //无符号字符型变量新表示方法定义
#define AD0   XBYTE [0xF0FF]      //给定量外部AD通道0的端口地址
#define AD1   XBYTE [0xF1FF]      //反馈量外部AD通道1的端口地址
#define DA    XBYTE [0xEFFF]      //外部DA转换数据输入端口地址
#define Displaydata XBYTE [0x7FFF]//数码管段码锁存器端口地址
#define Displaybit  XBYTE [0xBFFF]//数码管位码锁存器端口地址
#define A0r ______                //你的实验板的实验4的0通道数据
#define Amr ______                //你的实验板的实验4的0通道数据
#define N0r 0x20                  //你的实验板的实验4的0通道数据
#define Nmr 0xe0                  //你的实验板的实验4的0通道数据
#define A0m ______                //你的实验板的实验4的1通道数据
#define Amm ______                //你的实验板的实验4的1通道数据
#define N0m 0x20                  //你的实验板的实验4的1通道数据
#define Nmm 0xe0                  //你的实验板的实验4的1通道数据
#define Any ______                //你的实验板的实验2数据
#define Apy ______                //你的实验板的实验2数据
#define Nny 0x20                  //你的实验板的实验2数据
#define Npy 0xe0                  //你的实验板的实验2数据
sbit    SW8 = P1^7;               //ON为PID控制，OFF为不做任何控制算法，修正后误差送DA
bit     new_cycle_flag=0;         //有新采样数据的位标志定义(1是有新数据)
uchar   rkT,mkT;                  //给定数据源和反馈数据源变量定义
char    Ax;                       //显示用中间变量定义
char    dispbuf[8];               //显示缓冲区字符型数组定义
float   Axr,a1r,b1r;              //给定采样电压及其标度变换的斜率和截距定义
float   Axm,a1m,b1m;              //反馈采样电压及其标度变换的斜率和截距定义
float   ekT,ekT_T;                //当前电压误差、上次电压误差浮点数定义(伏特)
float   pPkT,pIkT,pDkT,pIkT_T,pkT;//当前各分量、上次积分分量及总控制量定义
float   Kp,Ki,Kd;                 //PID校正系数定义
float   a2,n0;                    //输出电压及其标度变换的斜率和截距定义
uchar   M=0,No_ch=0;              //扫描显示位计数、AD通道初始化变量定义
uchar   N=0,disp_gengxin=0;       //采样周期计时、显示更新计数字符变量定义
uchar code Disptab[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f}; //共阴0-9段码表
uchar code  Dispbit[8]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};           //共阴位码表
//float code  Kptable[8]={2.22,2.42,2.52,2.62,2.63,2.73,2.83,3.03};//理论值2.625,01#板调到最小还是等幅振荡
//float code  Kptable[8]={1.42,1.52,1.62,1.72,1.82,1.92,2.02,2.12};//换数做,01#板1.72等幅振荡,1.62衰减振荡
float code  Kptable[8]={1.64,1.66,1.68,1.685,1.69,1.695,1.70,1.71};//再换数做,01#板1.69等幅振荡,1.685极慢速衰减振荡
/**********标度变换初始化**********/
void Scaleconversion_value_init(void)
{ Axr = Nmr , Axm = Amr ;
  Axr -= N0r , Axm -= A0r ;
  a1r = Axm / Axr ;                 //教材中的式(6-18)的a1=(Am-A0)/(Nm-N0)
  b1r=-a1r;
  b1r*=N0r;
  b1r+=A0r;                         //教材中的式(6-18)的b1=A0-a1*N0
  Axr = Nmm , Axm = Amm ;
  Axr -= N0m , Axm -= A0m ;
  a1m = Axm / Axr ;                 //教材中的式(6-18)的a1=(Am-A0)/(Nm-N0)
  b1m=-a1m;
  b1m*=N0m;
  b1m+=A0m;                         //教材中的式(6-18)的b1=A0-a1*N0
  Axr = Npy , Axm = Apy ;
  Axr -= Nny , Axm -= Any ;
  a2 = Axr / Axm ;                  //教材中的式(6-18)的a1=(Am-A0)/(Nm-N0)
  n0 = -a2 ;
  n0 *= Any ;
  n0 += Nny ;                       //教材中的式(6-18)的b1=A0-a1*N0
}
/*************T0T1初始化*************/
void  T0T1_init(void)               //T0、T1初始化程序(11.0592MHz)
{ TMOD=0x21;                        //T0方式1定时、T1方式2定时(8位自动重装)
  SCON=0x40;                        //8位UART波特率可变(2^1*T1溢出率/32)
  PCON=0x80;                        //第8位SMOD=1,波特率加倍
  TH0=0xf3;                         //T0频率=(11059200/12)/(65536-0xf366)=285.679Hz
  TL0=0x66;                         //T0方式1定时周期=1/285.679=0.0035s=3.5ms
  TH1=0xfd;                         //T1溢出率=(11059200/12)/(256-253)=307200Hz
  TL1=0xfd;                         //波特率=2*307200/32=19200bps
  EA=1;                             //允许中断
  ES=0;                             //禁止串口中断
  ET0=1;                            //允许T0中断
  TR0=1;                            //启动T0定时
  TR1=1;                            //启动T1定时
}
/***********位置式PID控制计算**********/
void  PID(void)                     //采用与教材中的式(8-15)相同的PID算法
{ pPkT=Kp*ekT;                      //控制量的比例部分计算
  pIkT=pIkT_T;                      //控制量的积分部分计算
  pIkT+=Ki*ekT;                     //累加当前积分部分
  pDkT=Kd*(ekT-ekT_T);              //控制量的微分部分计算
  pkT=pPkT+pIkT+pDkT;               //教材中的式(8-15)相同
}
/*************DA输出函数*************/
void  DAC(void)                     //
{ pkT+=n0;                          //标度反变换的加法
  if(pkT>254.5) DA = 255 ;          //DA输出的修正值超大处理
  else if(pkT<0.5) DA = 0 ;         //DA输出的修正值为负处理
  else DA = (uchar)(pkT+0.5) ;      //DA输出的修正值适合DA输出的处理
}
/*******位置控制的显示数据更新*******/
void disp_g(void)                   //AD模拟电压显示缓冲区的更新
{ Ax=(char)(Axr*10);                //加快运算速度，给定量放大10倍取整数处理
  dispbuf[7]=0x00;                  //给定符号位预更新为正
  if(Ax<0)
  { Ax=-Ax;                         //如果给定电压是负则取绝对值
    dispbuf[7]=0x40;                //符号位送负号段码
  }
  dispbuf[4]=Disptab[Ax%10];        //电压小数位更新
  dispbuf[5]=Disptab[Ax/10%10]|0x80;//电压个位更新
  dispbuf[6]=Disptab[Ax/100];       //电压十位更新
}
void  disp_f(void)                  //给定量显示缓冲区的更新
{ Ax=(char)(Axm*10);                //加快运算速度，反馈量放大10倍取整数处理
  dispbuf[3]=0x00;                  //反馈符号位预更新为正
  if(Ax<0)
  { Ax=-Ax;                         //如果反馈电压是负则取绝对值
    dispbuf[3]=0x40;                //反馈符号位修改为负号
  }
  dispbuf[0]=Disptab[Ax%10];        //反馈电压小数位更新
  dispbuf[1]=Disptab[Ax/10%10]|0x80;//反馈电压个位更新
  dispbuf[2]=Disptab[Ax/100];       //反馈电压十位更新
}
/***************主程序***************/
void  main(void)                    //01#板Ks=1.69,Ts=0.358秒,控制度1.05时,T=0.014*Ts=0.005012秒,取5ms
  //手机秒表测满10屏的时间，根据截屏分析每屏多少周期，Ts=满10屏的时间/（10*每屏多少周期）
{ Kp=Kptable[P1&0x07];              //做纯比例控制用，做PID控制要注释掉，利用网络标号SW3~SW1设置PID参数Kp
  //Kp=1.0647;                        //做PID控制用，做纯比例控制要注释掉，控制度1.05时,Kp=0.63*Ks=1.0647
  Ki=0.03035;                       //控制度1.05时,Ti=0.49*Ts=0.17542,Ki=Kp*T/Ti=0.03034717
  Kd=10.6726;                       //控制度1.05时,Td=0.14*Ts=0.05012,Kd=Kp*Td/T=10.6725528
  Scaleconversion_value_init();     //标度变换初始化
  ekT_T=0.0,pIkT_T=0.0;             //上次误差和上次控制量积分部分必须初始化为0.0
  T0T1_init();                      //T0、T1初始化程序(11.0592MHz)
  IT0=1;                            //外部中断引脚INT0以下降沿方式触发中断请求
  EX0=1;                            //允许外部INT0申请中断
  do
  { if(new_cycle_flag==1)           //有新采样数据位标志
    { Axr=a1r*(float)rkT+b1r;       //给定量线性标度变换算出当前采样值对应的给定模拟电压
      Axm=a1m*(float)mkT+b1m;       //反馈量线性标度变换算出当前采样值对应的反馈模拟电压
      ekT=Axr-Axm;                  //计算当前误差电压，以伏特为单位
      ekT*=a2;                      //先做标度反变换的乘法修正，把电压误差(V)放大得到DA数字量误差    加法位于DA输出函数中
      if(SW8==0) PID();             //如果SW8实际设置在ON状态，做PID控制
      else pkT=Kp*ekT;              //否则SW8置在OFF状态，仅做P控制
      DAC();                        //做标度反变换的加法修正，即先+n0后控制量8位D/A输出
      TI=0;                         //清除发送结束状态标志
      ES=1;                         //允许串行发送结束中断
      SBUF =rkT;                    //发送报文第2字节，即给定量的A/D值
	  
      ekT_T=ekT,pIkT_T=pIkT;        //迭代移位，为下一次控制做准备       因为要赶紧把调整量输出，所以不在一PID运算完就移位，节省输出时间
	  
	  //处理后刚好28ms扫描 所以每七秒更新数据缓存区足矣
      disp_gengxin+=1;              //1,2,3,4(0),1,2,3,4(0)
      if(disp_gengxin==2) disp_g(); //是2,给定部分显示缓冲区中段码信息的更新
      else if(disp_gengxin==4) 
      { disp_f();                   //是4,反馈部分显示缓冲区中段码信息的更新
        disp_gengxin=0;             //清0
      }
      new_cycle_flag=0;             //新采样周期标志清0
    }
  }while(1);                        //固定循环
}
/***********T0中断服务程序***********/
void t0(void) interrupt 1 using 1   //3.5ms中断1次，每次均要进行显示处理
{ TH0=0xf3;                         //T0时间常数高字节重装f366
  TL0=0x66;                         //T0时间常数低字节重装
  if(N==2)                          //7ms到了吗？因为3.5ms*2=7ms
  { AD0=0x00;                       //到了7ms，启动AD0809的IN0通道——给定量A/D
    No_ch=0;                        //改成IN0通道输入的模拟量的AD转换的标识
    N=0;                            //N清0
  }
  if(M==8) M=0;                     //扫描显示一周了吗？扫描显示周期28.00ms    不需要每次的数据都显示，抽样显示
  Displaybit=0xff;                  //关显示
  Displaydata=dispbuf[M];           //查段码表送数显的段端口
  Displaybit=Dispbit[M];            //查位码表送数显的位端口
  N++;                              //采样周期计时
  M++;                              //修改下一次的显示位
}
/**********INT0中断服务程序**********/
void int0(void) interrupt 0 using 2 //每个采样周期中断2次
{ if(No_ch==0)                      //是IN0通道输入的模拟量的A/D转换的标识
  { rkT=AD0;                        //读取IN0通道给定量的A/D结果
    No_ch=1;                        //改成IN1通道输入的模拟量的A/D转换的标识
    AD1=0x00;                       //启动AD0809的IN1通道——反馈量A/D
  }
  else                              //否则就是IN1通道输入的模拟量的A/D转换标识
  { mkT=AD1;                        //读取IN1通道反馈量A/D结果
    new_cycle_flag=1;               //置2个通道均有采样数据的标志
  }
}
/**********TXD中断服务程序***********/
void  txd(void) interrupt 4 using 3 //每个采样周期需要发送2个字节，其中第2字节靠串行发送中断完成
{ 
  TI=0;                             //清除发送结束状态标志
  ES=0;                             //发送到最后1个字节，禁止发送结束中断
  SBUF=mkT;                         //发送报文第2字节，即反馈量的A/D值
}