 //SW3_SW2_SW1      Kp——纯比例控制的系数有拨码来取
// on  on  on      1.84
// on  on off      1.85
// on off  on      1.86
// on off off      1.87
//off  on  on      1.88
//off  on off      1.89
//off off  on      1.90
//off off off      1.91   //10屏45.13秒；45.40秒；45.35秒；45.14秒；45.34秒，取10屏45.3秒，每屏4.53秒
//看截屏幕，12周对应155小格，则13.9355周对应180小格，Ts=0.3251秒
#include <reg52.h>
#include <absacc.h>
#define uchar unsigned char       //无符号字符型变量新表示方法定义
#define uint  unsigned int        //无符号整型变量新表示方法定义
#define START_AD  XBYTE [0x7BFF]  //启动12位A/D转换。
#define ADHigh8   XBYTE [0x7BFF]  //12位A/D转换的高8位数据，读入D7～D0
#define ADLow4    XBYTE [0x7FFF]  //12位A/D转换的低4位数据，读入D7～D4
#define DALow8    XBYTE [0xBDFF]  //写D/A低8位数据，必须先写低8位,A14线选,1011110111111111
#define DAHigh4   XBYTE [0xBBFF]  //写D/A高4位数据，必须后写，写完就D/A,A14线选,1011101111111111
#define Displaydata XBYTE [0xDFFF]//数码管段码锁存器端口地址,A13线选,1101111111111111
#define Displaybit  XBYTE [0xEFFF]//数码管位码锁存器端口地址,A12线选,1110111111111111
#define A0_r  -8.961              //2上#实验板的实验2的0通道数据
#define Am_r  9.028               //2上#实验板的实验2的0通道数据
#define N0_r  0x202               //2上#实验板的实验2的0通道数据
#define Nm_r  0xe03               //2上#实验板的实验2的0通道数据
#define A0_m  -8.919              //2上#实验板的实验2的1通道数据
#define Am_m  9.013               //2上#实验板的实验2的1通道数据
#define N0_m  0x206               //2上#实验板的实验2的1通道数据
#define Nm_m  0xe02               //2上#实验板的实验2的1通道数据
#define Any -9.016                //2上#实验板的实验1数据
#define Apy 9.018                 //2上#实验板的实验1数据
#define Nny 0x200                 //2上#实验板的实验1数据
#define Npy 0xe00                 //2上#实验板的实验1数据
sbit    SW4=P1^3;                 //AD转换结束检测输入位定义(转换结束低电平)
sbit    SIGN_R=P1^5;              //左边给定的符号位
sbit    SIGN_M=P1^4;              //右边反馈的符号位
sbit    EOC=P1^6;                 //AD转换结束检测输入位定义(转换结束低电平)
sbit    CHANNEL_A=P3^2;           //AD转换的信号选择A(LSB)
sbit    CHANNEL_B=P3^3;           //AD转换的信号选择B(MSB)
sbit    CHANNEL_C=P3^4;           //AD转换的信号选择B(MSB)
sbit    S_H=P3^5;                 //采样保持信号，1采样；0保持
bit     new_cycle_flag=0;         //有新采样数据位标志定义(1是有新数据)
uchar   r_kT_H8,r_kT_L4,Ax;       //给定数据和显示中间变量定义
uchar   m_kT_H8,m_kT_L4;          //反馈数据定义
char    dispbuf[8];               //显示缓冲区字符型数组定义
uint    temp1;                    //计算电压的临时变量
int     r_kT,m_kT;                //计算的模拟电压放大100倍的整数部分
float   Ax_r,a1_r,b1_r;           //采样0通道电压及其标度变换的斜率和截距定义
float   Ax_m,a1_m,b1_m;           //采样1通道电压及其标度变换的斜率和截距定义
float   a2,n0;                    //输出电压及其标度变换的斜率和截距定义
uint    DA_y;                     //12位的DA数字量
float   e_kT=0.0,e_kT_T,e_KT_T2=0.0,A,C,D;         //当前电压修正误差、上次修正误差浮点变量定义(伏)
float   p_P_kT,p_I_kT,p_D_kT,p_I_kT_T=0.0,p_kT,p_kl=0.0;//当前各分量、上次积分分量及总控制量定义
float   Kp,Ki,Kd;                 //PID校正系数定义
uchar   M=0;                      //扫描显示位计数变量定义 
uchar code Disptab[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};//共阴0-9段码表
uchar code Dispbit[8]={0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f};           //共阴位码表
//float code Kptable[8]={2.22,2.42,2.52,2.62,2.63,2.73,2.83,3.03};//第1次选最小2.22还等幅振荡,说明还要取更小一些数据；
//float code Kptable[8]={1.42,1.52,1.62,1.72,1.82,1.92,2.02,2.12};//换第2组，1.82衰减振荡，1.92等幅振荡,Ks介于1.82~1.92；
float code Kptable[8]={1.84,1.85,1.86,1.87,1.88,1.89,1.90,1.91};//1.89慢速衰减振荡，1.90极慢速衰减振荡，1.91等幅振荡
void Scaleconversion_value_init(void)
{ Ax_r = Am_r , Ax_m=Nm_r ;
  Ax_r -= A0_r , Ax_m -= N0_r ;
  a1_r = Ax_r  / Ax_m ;
  b1_r = -a1_r ;
  b1_r *= N0_r ;
  b1_r += A0_r ;                  //教材中的式(6-18)的斜率a1和截距b1
  Ax_r = Am_m , Ax_m=Nm_m ;
  Ax_r -= A0_m , Ax_m -= N0_m ;
  a1_m = Ax_r  / Ax_m ;
  b1_m = -a1_m ;
  b1_m *= N0_m ;
  b1_m += A0_m ;                  //教材中的式(6-18)的斜率a1和截距b1
  Ax_m = Npy , Ax_r = Apy ;
  Ax_m -= Nny , Ax_r -= Any ;
  a2 = Ax_m / Ax_r ;
  n0 = -a2 ;
  n0 *= Any ;
  n0 += Nny ;
}
void  T0T1_init(void)             //T0、T1初始化程序(内部用的是24.0MHz)
{ TMOD=0x21;                      //T0方式1定时、T1方式2定时(8位自动重装)
  SCON=0x40;                      //8位UART波特率可变(2^1*T1溢出率/32)
  PCON=0x80;                      //第8位SMOD=1,
  TH0=0xf6;                       //T0频率=(24000000/6)/(65536-0xf63c)=1600Hz
  TL0=0x3c;                       //T0方式1定时周期=1/1600=0.000625s=0.625ms
  TH1=0xf3;                       //T1溢出率=(24000000/6)/(256-243)=307692.3Hz
  TL1=0xf3;                       //波特率=2*307692.3/32=19230.77bps，PC机用19200bps
  EA=1;                           //允许中断
  ES=0;                           //禁止串行发送结束中断
  ET0=1;                          //允许T0中断
  TR0=1;                          //启动T0定时
  TR1=1;                          //启动T1定时
}
/**********数字P控制计算**********/
void  P_C(void)                   //
{ p_I_kT=0;                       //为了切换控制算法需要
  p_kT=Kptable[P1&0x07];          //做P控制时用，用控制盒背面SW3~SW1设置P控制参数Kp
  p_kT*=e_kT;                     //P控制计算
}
/**********数字PID控制计算**********/
void  PID(void)                   //采用与教材中的式(8-15)相同的PID算法
{ p_P_kT=Kp*e_kT;                 //控制量的比例部分
  p_I_kT=p_I_kT_T;                //控制量的积分部分
  p_I_kT+=Ki*e_kT;                //累加积分项
  p_D_kT=Kd*(e_kT-e_kT_T);        //控制量的微分部分
  p_kT=p_P_kT+p_I_kT+p_D_kT;      //教材中的式(8-15)相同
}
void  JFFL_PID(void)              //采用与教材中的式(8-15)相同的PID算法及式(8-24)
{ p_P_kT=Kp*e_kT;                 //控制量的比例部分
  p_I_kT=p_I_kT_T;                //控制量的积分部分
  if(e_kT>a2);                    //误差正1.0V超标，不积分
  else if(e_kT<-a2);              //误差负1.0V超标，也不积分
  else p_I_kT+=Ki*e_kT;           //误差不超标，累加积分项
  p_D_kT=Kd*(e_kT-e_kT_T);        //控制量的微分部分
  p_kT=p_P_kT+p_I_kT+p_D_kT;      //教材中的式(8-15)相同
}

void  YXXR_PID(void)              //采用与教材中的式(8-15)相同的PID算法及式(8-24)
{ p_P_kT=Kp*e_kT;                 //控制量的比例部分
  p_I_kT=p_I_kT_T;                //控制量的积分部分
        
  if(p_kl>=2047)
    {
	  if(e_kT>0);
	  else
		 p_I_kT+=Ki*e_kT;
	}
  else if(p_kl<=-2047)  
    {
	  if(e_kT<0);
	  	  else
		 p_I_kT+=Ki*e_kT;
	}
	else   p_I_kT+=Ki*e_kT;
	 p_D_kT=Kd*(e_kT-e_kT_T);		   //控制量的微分部分   

	p_kT=p_P_kT+p_I_kT+p_D_kT;
}


void  disp_g(void)                //给定模拟电压显示缓冲区的更新
{ if(r_kT<0)                      //
  { r_kT=-r_kT;                   //取绝对值
    SIGN_R=0;                     //符号位送负号
  }
  else SIGN_R=1;                  //符号位送正号
  dispbuf[4]=Disptab[r_kT%10];    //电压小数点后2位更新
  dispbuf[5]=Disptab[r_kT/10%10]; //电压小数点后1位更新
  dispbuf[6]=Disptab[r_kT/100%10]|0x80;//电压个位更新
  dispbuf[7]=Disptab[r_kT/1000];  //电压十位更新
}
void  disp_f(void)                //反馈模拟电压显示缓冲区的更新
{ if(m_kT<0)                      //
  { m_kT=-m_kT;                   //取绝对值
    SIGN_M=0;                     //符号位送负号
  }
  else SIGN_M=1;                  //符号位送正号
  dispbuf[0]=Disptab[m_kT%10];    //电压小数点后2位更新
  dispbuf[1]=Disptab[m_kT/10%10]; //电压小数点后1位更新
  dispbuf[2]=Disptab[m_kT/100%10]|0x80;//电压个位更新
  dispbuf[3]=Disptab[m_kT/1000];  //电压十位更新
}
void  main(void)                  //2上#Ks=1.91,Ts=0.3251秒,控制度1.05时,T=0.014*Ts=0.00455秒,还取0.005s
{ Kp=10;                      //做PID控制时用，做纯比例控制要注释掉，控制度1.05,Kp=0.63*Ks=1.2033
  Ki=0.769;                     //做PID控制时用，控制度1.05,Ti=0.49*Ts=0.1593,Ki=Kp*T/Ti=0.03777
  Kd=120;                       //做PID控制时用，控制度1.05,Td=0.14*Ts=0.0455,Kd=Kp*Td/T=10.95
  CHANNEL_A=0;                    //选择0通道，对0通道进行AD转换
  CHANNEL_B=0;                    //
  CHANNEL_C=0;                    //
  Scaleconversion_value_init();   //标度变换初始化，分别计算采样输入通道和DA输出通道的斜率和截距
  T0T1_init();                    //T0、T1初始化程序(24.0MHz)
  do
  { if(new_cycle_flag==1)         //有新采样数据位标志,Vs1=-(R6/R5)*Vx1=-(20K/100K)*Vx1=-0.2Vx1
    { Ax_r=(float)temp1;          //准备给定量计算
      Ax_r*=a1_r;                 //标度变换的乘法
      Ax_r+=b1_r;                 //标度变换的加法
      temp1=m_kT_H8;
      temp1<<=4;                  //AD转换结果的高8位暂存到速度给定
      m_kT_L4=m_kT_L4>>4;
      temp1+=m_kT_L4;             //合并低4位后得到完整的12位
      Ax_m=(float)temp1;          //准备反馈量计算
      Ax_m*=a1_m;                 //标度变换的乘法
      Ax_m+=b1_m;                 //标度变换的加法
      e_kT=Ax_r-Ax_m;             //计算误差
      e_kT = a2*e_kT;             //标度反变换的乘法
      if(SW4==0) P_C();           //如果SW4设在ON，P控制函数调用
      //else PID();                 //SW4设在OFF，普通PID控制函数调用
      else JFFL_PID();            //SW4设在OFF，JFFL_PID控制函数调用
	   //YXXR_PID();
	  p_kl=p_kT;
      p_kT += n0;                 //标度反变换的加法
      if(p_kT>4094.5)   DA_y = 4095;//DA输出的修正值超大处理，考虑四舍五入
      else if(p_kT<0.5) DA_y = 0 ;  //DA输出的修正值为负处理，考虑四舍五入
      else              DA_y = (uint)(p_kT+0.5) ;//DA输出的修正值适合DA输出的处理，考虑四舍五入
      DALow8=(uchar)DA_y;         //先DA输出低8位
      DAHigh4=(uchar)(DA_y>>8);   //后DA输出高4位
      r_kT=(int)(100.0*Ax_r);     //理论显示数字量部分r_kT=100*Ax_r
      m_kT=(int)(100.0*Ax_m);     //理论显示数字量部分m_kT=100*Ax_m
      TI=0;                       //清除发送结束状态标志
      ES=1;                       //允许串行发送结束中断
      SBUF = r_kT_H8;             //发送12位二进制数据的高8位
      disp_g();                   //是1反馈部分显示缓冲区中段码数据的更新
      disp_f();                   //是3给定部分显示缓冲区中段码数据的更新
      e_KT_T2=e_kT_T;
      e_kT_T=e_kT;p_I_kT_T=p_I_kT;//迭代移位，为下一次控制作准备
      new_cycle_flag=0;           //新采样周期标志清0
    }
  }while(1);
}
void  t0(void) interrupt 1 using 1//0.625ms中断1次，每次均要进行显示处理
{ TH0=0xf6;                       //T0时间常数高字节重装0xf63c
  TL0=0x3c;                       //T0时间常数低字节重装
  S_H=1;                          //采样
  if(M==8)                        //5ms到了吗？因为0.625ms*8=5ms
  { S_H=0;                        //保持给定量
    START_AD=0x00;                //启动12位A/D转换，写0x7BFF:/WR=0,CE=1晚;A15=0,/CS=0早;/RD=1,A9=1得R/C=0早;A10=0,A0=0早
    CHANNEL_A=1;                  //预送反馈量到采样保持器输入端，待采样
    M=0;                          //到了5ms，M清0
    new_cycle_flag=1;             //置有新采样数据标志
    while(EOC==1);                //如果还是高电平，转换还没结束，循环等待
    S_H=1;                        //采样反馈量送A/D，需一定时间采样才达到稳定，理论上3~4倍的时间常数，与采样电容的容量有关。
    r_kT_H8=ADHigh8;              //读入高8位,读0x7BFF:/RD=0,CE=1晚;A15=0,/CS=0早;/RD=0,A9=1得R/C=1晚;A10=0,A0=0早
    r_kT_L4=ADLow4;               //同理读入低4位
    temp1=r_kT_H8;                //12位AD转换结果的高8位
    temp1<<=4;                    //变成16位变量的相应位置
    r_kT_L4=r_kT_L4>>4;           //12位AD转换结果的低4位变成真正的低4位
    temp1+=r_kT_L4;               //合并低4位后得到完整的12位
    S_H=0;                        //保持反馈量
    START_AD=0x00;                //启动12位A/D转换，写0x7BFF:/WR=0,CE=1晚;A15=0,/CS=0早;/RD=1,A9=1得R/C=0早;A10=0,A0=0早
    CHANNEL_A=0;                  //预先送给定到采样保持器输入端，待采样
    while(EOC==1);                //如果还是高电平，循环等待
    m_kT_H8=ADHigh8;              //读入高8位,读0x7BFF:/RD=0,CE=1晚;A15=0,/CS=0早;/RD=0,A9=1得R/C=1晚;A10=0,A0=0早
    m_kT_L4=ADLow4;               //同理读入低4位
  }
  Displaybit=0xff;                //关闭位端口即关闭显示
  Displaydata=dispbuf[M];         //查段码表送数显的段端口
  Displaybit=Dispbit[M];          //查位码表送数显的位端口
  M++;                            //修改下一次的显示位
}
void txd(void) interrupt 4 using 2//每5ms需要发送2个字节，其中第2字节靠串行发送中断完成
{ TI=0;                           //清除发送结束状态标志
  ES=0;                           //发送到最后1个字节，禁止发送结束中断
  SBUF=m_kT_H8;                   //发送报文第2字节，即DA值
}
