#include "STC8xxxx.H"
#define ui unsigned int
#define uc unsigned char
sbit lcdrs=P6^4;
sbit lcdrw=P6^5;
sbit lcden=P6^6;
sbit s1=P3^0;          //功能键
sbit s2=P3^1;          //增大键
sbit s3=P3^2;          //减小键
sbit s4=P3^3;          //查看闹钟键
sbit bee=P2^7;
bit flag1,flag_ri;          //定义两个位变量

uc t1,t2,miao,shi,fen,year,month,day,week;
uc ashi,afen,amiao;
uc s1num,flag;
uc code a[]="Made by FF";      //开机时显示
uc code a1[]="  20  -  -       ";      //固定显示内容：20xx-xx-xx xxx
uc code a2[]="      :  :  ";        //显示时间
void delay(ui s)
{
  ui i,j;
  for(i=s;i>0;i--)
   for(j=110;j>0;j--);
}


void di()
{
  bee=0;
  delay(100);
  bee=1;
}


void write_com(uc c)      //写命令函数
{
  lcdrs=0;
  lcden=0;
  P0=c;
  delay(5);
  lcden=1;
  delay(5);
  lcden=0;

}




void write_data(uc d)      //写数据函数
{
  lcdrs=1;
  lcden=0;
  P0=d;
  delay(5);
  lcden=1;
  delay(5);
  lcden=0;

}



void init()           //lcd初始化函数
{
  lcdrw=0;
  lcden=0;
  write_com(0x38);
  write_com(0x80);
  write_com(0x0c);
  write_com(0x06);
  write_com(0x01);
}



 void write_sfm(uc a,uc b)    //1602刷新时分秒函数，4为时，7为分，10为秒
 {
   uc shi,ge;
	shi=b/10;
	ge=b%10;
	write_com(0x80+0x40+a);
	write_data(0x30+shi);
	write_data(0x30+ge);
 }



 void write_nyr(uc a,uc b)     //刷新年月日函数，3为年，6为月，9为日
 {
   uc shi,ge;
	shi=b/10;
	ge=b%10;
	write_com(0x80+a);
	write_data(0x30+shi);
	write_data(0x30+ge);

 }





void write_week(uc w)        //1602显示星期函数
{
  write_com(0x80+13);
  switch(w)
  {
    case 1:   write_data('M');delay(5);
	           write_data('O');delay(5);
				  write_data('N');
				  break;


	 case 2:   write_data('T');delay(5);
	           write_data('U');delay(5);
				  write_data('E');
				  break;



	 case 3:   write_data('W');delay(5);
	           write_data('E');delay(5);
				  write_data('D');
				  break;


	 case 4:   write_data('T');delay(5);
	           write_data('H');delay(5);
				  write_data('U');
				  break;



	 case 5:   write_data('F');delay(5);
	           write_data('R');delay(5);
				  write_data('I');
				  break;


	 case 6:   write_data('S');delay(5);
	           write_data('A');delay(5);
				  write_data('T');
				  break;



	  case 7:   write_data('S');delay(5);
	           write_data('U');delay(5);
				  write_data('N');
				  break;
  }
}



void keyscan()     
{
  if(flag_ri==1)    //用来取消闹钟蜂鸣
  {
    if(s1==0||s2==0||s3==0||s4==0)      //按任意键取消
 	 {
	   delay(5);
      if((s1==0)||(s2==0)||(s3==0)||(s4==0))
		{
		   while(!(s1&&s2&&s3&&s4));
			di();
			flag_ri=0;                 //取消
		}
	 }
  }

  if(s1==0)
  {
    delay(5);
  if(s1==0)
 { s1num++;                 //记下按下次数
  if(flag1==1)                   //flag1=1即进入调节闹钟
       if(s1num==4)               //闹钟只能调节时分秒
	       s1num=1;
  flag=1;
  while(!s1);
  di();
  switch(s1num)          //定位光标闪烁点
  {
     case 1:  write_com(0x80+0x40+10);
	         write_com(0x0f);
				break;


	  case 2:  write_com(0x80+0x40+7);
	          break;



		
	  case 3:  write_com(0x80+0x40+4);
	          break;


	 
	  case 4:  write_com(0x80+12);
	          break;


	  case 5:  write_com(0x80+9);
	          break;


	  case 6:  write_com(0x80+6);
	          break;




	  case 7:  write_com(0x80+3);
	          break;


	   case 8:  s1num=0;
		         write_com(0x0c);
					flag=0;
					break;
		
		
     } 
    }
  }

if(s1num!=0)                       //只有当S1按下后才检测S2和S3
{
   if(s2==0)
	{
	  delay(5);
	  if(s2==0)
	  {
	    while(!s2);
		 di();
		 switch(s1num)                  //根据功能键次数调节相应数值
		 {
		   case 1:  miao++;
			         if(miao==60)
						 miao=0;
						write_sfm(10,miao);
						write_com(0x80+0x40+10);      //光标显示的位置
						break;



			case 2:  fen++;
			         if(fen==60)
						 fen=0;
						write_sfm(7,fen);
						write_com(0x80+0x40+7);
						break;


			case 3:  shi++;
			         if(shi==24)
						 shi=0;
						write_sfm(4,shi);
						write_com(0x80+0x40+4);
						break;


			case 4:   week++;
			          if(week==8)
						   week=1;
                   write_week(week);
						 write_com(0x80+12);
						 break;




			case 5:   day++;
			          if(day==32)
						   day=1;
                   write_nyr(9,day);
						 write_com(0x80+9);
						 break;



		 	case 6:   month++;
			          if(month==13)
						   month=1;
                   write_nyr(6,month);
						 write_com(0x80+6);
						 break;



			case 7:     year++;
			          if(year==100)
						   year=0;
                   write_nyr(3,year);
						 write_com(0x80+3);
						 break;




		 }
	  }
	}


	if(s3==0)
	{
	  delay(5);
	  if(s3==0)
	  {
	     while(!s3);
		  di();
		  switch(s1num)
		  {
		    case 1:   miao--;
			           if(miao==-1)
						  miao=59;
                    write_sfm(10,miao);
						  write_com(0x80+0x40+10);
						  break;



			 case 2:   fen--;
			           if(fen==-1)
						  miao=59;
                    write_sfm(7,fen);
						  write_com(0x80+0x40+7);
						  break;




			 case 3:   shi--;
			           if(shi==-1)
						  shi=23;
                    write_sfm(4,shi);
						  write_com(0x80+0x40+4);
						  break;



			 case 4:   week--;
			          if(week==0)
						  week=7;
						  write_week(week);
						  write_com(0x80+12);
                    break;



			 case 5:  day--;
			          if(day==0)
						 day=31;
						 write_nyr(9,day);
						 write_com(0x80+9);
						 break;



			  case 6:  month--;
			          if(month==0)
						 month=12;
						 write_nyr(6,month);
						 write_com(0x80+6);
						 break;



			 case 7:  year--;
			          if(year==0)
						 year=100;
						 write_nyr(3,year);
						 write_com(0x80+3);
						 break;



		  }
	  }
	}
 }
  if(s4==0)                       //设置闹钟
     {
       delay(5);
       if(s4==0)
       {
         flag1=~flag1;
         while(!s4);
         di();
         if(flag1==0)           //退出闹钟设置时保存数值
         {
           flag=0;
           write_com(0x80+0x40);
           write_data(' ');
           write_data(' ');
           write_com(0x0c);
			  ashi=shi;
			  afen=fen;
			  amiao=miao;
          }
        else                   //进入闹钟设置
        {
                      
          miao=amiao;             //读取原来的值，重新赋值以按键调节
          fen=afen;
          shi=ashi;
          write_com(0x80+0x40);
          write_data('R');          //显示标志
          write_data('I');
          write_com(0x80+0x40+3);
          write_sfm(4,ashi);        //送液晶显示
          write_sfm(7,afen);
          write_sfm(10,miao);
        }
       } 
     }
}

 void init2()          //定时器初始化函数
{
 TMOD=0X01;
 TH0=(65536-45872)/256;
 TL0=(65536-45872)%256;
 EA=1;
 ET0=1;
 TR0=1;
 
 
}


void T0_time() interrupt 1
{
  TH0=(65536-45872)/256;
  TL0=(65536-45872)%256;
  t1++;
  if(t1==20);
   {
     t1=0;
     miao++;
    if(miao==60)
    {
      miao=0;

     fen++;
     if(fen==60)
     {
       fen=0;
       shi++;
       if(shi==24)
       {
         shi=0;
         day++;
        if(day==32)
         {
          day=1;
          week++;
         if(week==8)
          {
           week=1;
           month++;
           if(month==13)
            {
             month=1;
             year++;
            }

           
          }
         }
        }
      }
    }
   }
    if(ashi==shi&&amiao==miao&&afen==fen)
	  flag_ri=1;
}



void main()
{ int i;
  ashi=0;
  afen=0;
  amiao=0;
  init();
  init2();
  lcden=0;
  s1num=0;
  flag1=0;
  write_com(0x80);
  for(i=0;i<10;i++)
   {
     write_data(a[i]);
     delay(70);
   }
	write_com(0x01);    //显示清零，数据指针清零
	write_com(0x80);       
	for(i=-1;i<17;i++)    //显示固定内容
	{
	 write_data(a1[i]);
	 delay(10);
	}
   write_com(0x80+0x40);
	for(i=0;i<12;i++)
	{
	 write_data(a2[i]);
	 delay(10);
	}
   year=17;            //初始时间2017-6-5-Mon-00-00-00
   month=6;
   week=1;
   day=5;
   shi=0;
   fen=0;
   miao=0;
   while(1)
  {
//   keyscan();
   if(flag_ri==1)          //闹钟中断时进入
   {
    di();
    delay(50);
    di();
    delay(100);
   }
    if(flag==0&&flag1==0)    //正常工作时进入
    {
 //     keyscan();
      write_sfm(10,miao);      //送液晶显示
      write_sfm(7,fen);
      write_sfm(4,shi);
		write_week(week);
		write_nyr(4,year);
		write_nyr(7,month);
		write_nyr(10,day);
    }
  } 
}
