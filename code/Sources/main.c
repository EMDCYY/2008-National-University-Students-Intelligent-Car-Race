#include <hidef.h>      /* common defines and macros */
#include <mc9s12dp512.h>     /* derivative information */
#include "STDINC.h"
//#include "stdlib.h"
#include "sci.h"
#pragma LINK_INFO DERIVATIVE "mc9s12dp512"
//////////////////////////参数设定//////////////////////
//舵机直行的PWM--2400
//马达60%--1200
#define OK 1
#define WRONG 0
int16 RUDKP=33;//50;
int16 RUDKD=5;//5;
int16 MOTKP=1;//=0.005263;0.002631;0.004197;
int16 MOTKI=1;//=0.005263;0.007894;0.008947;
int16 MOTKD=1;//=0.005263;0.002631;

//舵机参数
int16 RErrLast=0;
int16 RErrCurr=0;
int16 RErrPreLast=0;
int16 Ruddelta=0;
int16 RudPWM;
int16 SenFlg=0;
//电机参数
int16 SpeedCurr=0;
int16 SpeedLast=0;
int16 Speed=0;
//double SpeedFreq=0;
int16 Motdelta=0;
int16 MotPWM=1000;
int16 SErrLast=0;
int16 SErrCurr=0;
int16 SErrPreLast=0;
int16 MotDD=0;
/////////////////速度参数///////////////////////////
int16 Pdelta[4]={23,22,21,14};//{0.67,0.70,0.73,0.50};
int16 Ddelta[4]={3,2,2,1};    //{2,1,1,1}
int16 Vline=3135;
int16 Vscor=2660;
int16 Vlcor=2565;
int16 Vbrek=2375;
int16 Vsbrk=2375;
int16 Vstri=2755;
//Global variables
int16 SenCurBacW,SenLasBacW;
int16 TarSpeed,LastTarSpeed;
float Modulus,TarSpeedF;
int16 TarFlg=0;
uint8 sensorA=0,sensorB=0;
uint8 SensorA=0,SensorB=0;
uint8 scan=1;   
int16 Sensor;      
uint16 CounterCurr,CounterLast,Counter;
int16 tmpflg;
int16 de;
////////////////////////start line///////////////////
int16 LapFlg1=0;
int16 LapFlg2=0;
int8 Flagarm=0;
int16 temp=0;
int16 LapNum=0;
int16 LNum=0;
int8 FLAG=0;
int8 LFlg=0;
int16 delay1=0;
int16 StopFlg=0,StopNum=0;
////////////////////sort/////////////////////////
int16 j=0;
int16 tmp[19]={0};
int16 num=19;
int16 s1,s2,s3;
int16 Tar,cnt1,cnt2,cnt3,cnt4,cnt5;                
int16 flag4;

int8  sf=0;
int16 t1=665,t2=95,t3=0;
int8 Wflag=0;

//////////////////////////函数声明//////////////////////
void atd_init(void);
void motor_ctrl(void);
void resensor(void);
void readspeed(void);
void rudder_ctrl(void);
int8 path_recog(void);
void sort(void);
void startline(void);
////////////////////////////////////////////////////////
#pragma MESSAGE DISABLE C12056         /* Disable warning C12056 
                                      //"SP debug info incorrect because of 
                                      //optimization or inline assembler" */
                                      
                                      
 #pragma MESSAGE DISABLE C2705          /*Disable warning C2705 "possible loss of data"*/                            

  //WmsgSdC2705;

////////////////////////中断函数/////////////////
#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt void readsensor(void){
 DisableInterrupts;
 switch(scan) {
  
  case 1: 
          PORTA=0x00;
          PORTA_BIT5=1; 
          scan++;
          sensorA=0;
          sensorB=0; 
          break;
  case 2:
          
          sensorA=sensorA+(PTM & 0x01);            
          sensorB=sensorB+(PORTB & 0x20);
                                 
          PORTA=0x00;
          scan++; 
          break;
  case 3:
          PORTA=0x00;
          PORTA_BIT4=1; 
          scan++; 
          break;
  case 4:
          sensorA=sensorA+((PTM&0x01)*2);        //6look5    
          sensorB=sensorB+(PORTB&0x10);          

          PORTA=0x00;
          scan++; 
          break;
  case 5:
          PORTA=0x00;
          PORTA_BIT3=1; 
          scan++; 
          break;
  case 6:
          sensorA=sensorA+((PTM&0x08)/2);        //3look4    
          sensorB=sensorB+(PORTB&0x08);          
          PORTA=0x00;
          scan++; 
          break;
  case 7:
          PORTA=0x00;
          PORTA_BIT2=1; 
          scan++; 
          break;
  case 8:
          sensorA=sensorA+(PTM&0x08);            
          sensorB=sensorB+(PORTB&0x04);
          PORTA=0x00;
          scan++; 
          break;         
  case 9:
          PORTA=0x00;
          PORTA_BIT1=1; 
          scan++; 
          break;
  case 10:
          
          sensorA=sensorA+(PTM&0x10);            
          sensorB=sensorB+((PORTB&0x01)*2);     //7look8
          PORTA=0x00;
          scan++; 
          break;
  case 11:
          PORTA=0x00;
          PORTA_BIT0=1; 
          scan++; 
          break;
  case 12:
          sensorA=sensorA+(PTM&0x20);            
          sensorB=sensorB+(PORTB&0x01);
          PORTA=0x00;
          scan=1; 
          
 sensorA=sensorA&0x3F;   //clean unwanted bit
 sensorB=sensorB&0x3F;    
 
 Flagarm=1; 
 if(StopFlg==1)
 {
 StopNum++;
 }
 
 if(LFlg==1)
 {
 delay1++;
 if(delay1>=5) 
 {
 LFlg=0;
 delay1=0;
 }
 }
 else startline();
 
 TarFlg++;
 Counter=PACN32;
 PACN32=0;
 if(Counter<30) 
 {
 Counter=30;
 }
 Speed=Counter*38;//Speed=Counter/5*190
 //if(LapNum>2) {
 //PWMDTY45=1000;
 //}
 //PWMDTY01=1500;
  break;
  default: ;
 }
 /*if(de<6000)
 { 
  de++;
 }*/
 MCCNT=2000;         //0.5ms every interrupt
 MCFLG_MCZF=1;
 EnableInterrupts;
 
}
uint16 plsnum=0;
/*////////////////////读取速度传感器////////
#pragma CODE_SEG __NEAR_SEG NON_BANKED
 interrupt void readspeed(void){
  TFLG1_C3F=1;
  //CounterLast=CounterCurr;
  CounterCurr=TC3;
  CounterLast=TC3H;
  switch(TFLG2_TOF) 
  {
    case 0:Counter=CounterCurr-CounterLast;
           break;
    case 1:Counter=65535-CounterLast+CounterCurr;
           TFLG2_TOF=1;
           break;
  }
  if(Counter<16)
  {                                          
     Counter=16;
  }
  //SpeedFreq=800*256;              //范围大概在100-226
  Speed = 1000000/Counter;//190;//((float)Speed*3/4+(float)SpeedFreq/4);          //Speed  110--250
  
  //if(Speed>=385)  FLAG=1;
 //if(LapNum>2) 
 //{
 //PWMDTY45=1000;
 //}
 //else
 if(LapNum==1)
 {
  plsnum++;
 }
  
 if(LapNum>1) 
 {
  StopFlg=1;
  if(StopNum>100) 
  {
    PWMDTY45=1000;
  }
 }
 else 
 motor_ctrl();
 //FLAG++;
 }*/                                                        
 
////////////////////////函数定义//////////////////////////////////

void pwm_init(void){
 PWMPOL = 0xFF;   //polarity,high at the beginning;
 PWMCLK = 0x2A;   //select SA/SB as the pwmclk;SA clock for STEER;SB clock for MOTOR;
 PWMPRCLK = 0x11; //A=bus clock/2=16MHz,B=bus clock/2=16MHz;
 PWMCAE = 0x00;   //PWM flush left
 PWMSCLA = 0x01;  //SA=A/2;PWM clock=bus clock/2/2/1=8MHz
 PWMSCLB = 0x05;  //SB=B/1;PWM clock=bus clock/2/2/5=1.6MHz
 PWMCTL = 0xF0;   //concatenate 0&1,2&3;

 PWMPER45 = 2000;  //output b frequency=SA clock/2000=4KHz;for MOTOR using
 PWMDTY45 = 1000;  //PWM percent is 65%; 

 PWMPER23 = 16000;    //output frequency=SB clock/16000=100Hz;for RUDDER using
 PWMDTY23 = 2460;     //PWM percent is 15%  down--LEFT
 
 PWME=0x2A;
}
void ect_init(void) {

 MCCTL=0xC2;         //MCZI MODMC RDMCL ICLAT FLMC MCEN MCPR1 MCPR0=11000010B  clock=Fbus/8
 MCCNT=2000;         //3ms every interrupt
 MCFLG=0x80;         //set underflow flag
 //////////////////////////////PACA_init//////////////////////////////////////
  PACTL_PAEN=1;       //enable the pulse accumualtor A
  PACTL_PAMOD=0;      //set the accumulator A mode---event counter mode
  PACTL_PEDGE=1;      //the accumulator edge control--rising edges on PT7 pin
  TIOS=0X00;           //input capture selection,channal 6: output compare channal;
  TCTL3=0X01;
  TCTL4=0X40;                 //capture on the rasing edge;
  TIE=0X18;                     //enable interrupt when captured;       
  TSCR2=0X05;                //bit7:overflow interrupt disable,bit[2:0]:timer=bus clock/32=1MHz
  ICOVW=0X00;               //overwrite by the new data;
  ICSYS=0X02;              //queue mode;
  TSCR1=0x80;              //enable the timer;
  MCCTL_MCEN=1; 
}
//////////////////////////////sci_init/////////////////////////////////
void SciInt(void){
   
  SCI0BDL=0xD0; //总线为32M，波特率为9600
  SCI0CR2=0X2C; //允许中断，允许发送，允许接受
       
   }
   
void SCI_Transmit(byte data) {
  while (!SCI0SR1_TDRE);//SC0DR处于忙状态，等待。
  SCI0DRL=data;
}
void systemboot(void) {
 SYNR = 7;      //Fbus=Fosc*(SYNR+1)/(REFDV+1)=32MHz
 REFDV = 3;
 PLLCTL = 0x71;
 while(CRGFLG_LOCK==0){}
 CLKSEL = 0x80;

 DDRA = 0xFF;     //PortA is input
 PORTA=0x00;        //clean PortA
 DDRB = 0x00;     //PortB is input
 PORTB=0x00;        //clean PortB
 PORTK= 0x00;
 DDRK = 0x00;
 PTM  = 0x00;
 DDRM = 0x00;
 pwm_init();
 ect_init();
 SciInt();
 //////////////////////////////////////////
 if(PORTK_BIT0==1&&PORTK_BIT1==0)
 {
  Vline=2945;//2755;
  Vscor=2660;//2565;
  Vlcor=2660;//2470;
  Vbrek=2565;//2470;     //可尝试变更2090-2185
  Vsbrk=2565;//2470;     
  Vstri=2660;//2565;
  Pdelta[0]=23;// 22
  Pdelta[1]=22;//23
  Pdelta[2]=21;//24
  Ddelta[0]=3; //2
  Ddelta[1]=2; //1
  Ddelta[2]=2; //1
 } 
 else if(PORTK_BIT0==0&&PORTK_BIT1==1)
 {
  Vline=3135;//2755;
  Vscor=2660;//2565;
  Vlcor=2660;//2470;
  Vbrek=2470;//2470;     //可尝试变更2090-2185
  Vsbrk=2470;//2470;     
  Vstri=2660;//2565;
  Pdelta[0]=23;
  Pdelta[1]=22;
  Pdelta[2]=21;
  Ddelta[0]=3; //2
  Ddelta[1]=2; //1
  Ddelta[2]=2; //1
 } 
 else if(PORTK_BIT0==1&&PORTK_BIT1==1)
 {
  Vline=3135;
  Vscor=2850;
  Vlcor=2755;
  Vbrek=2660;     //可尝试变更2090-2185
  Vsbrk=2660;     
  Vstri=2850;
  Pdelta[0]=23;
  Pdelta[1]=22;
  Pdelta[2]=21;
  Ddelta[0]=3; //2
  Ddelta[1]=2; //1
  Ddelta[2]=2; //1
 }
 if(PORTK_BIT2==1){Vscor=Vscor+95;Vstri=Vstri+95;}
 if(PORTK_BIT3==1){Vscor=Vscor+190;Vstri=Vstri+190;}
 if(PORTK_BIT4==1){Vlcor=Vlcor+95;}
 if(PORTK_BIT5==1){Vlcor=Vlcor+190;}
 if(PORTK_BIT7==1){Vline=Vline+95;}
 if(PORTB_BIT7==1){Vline=Vline+190;}
}
#pragma CODE_SEG DEFAULT

void main(void) {
  /* put your own code here */
  DisableInterrupts;
  systemboot();
  _asm(nop);
     
  EnableInterrupts;
  //while(de<1000);
  for(;;){  
  if(Flagarm==1) 
  {
    Flagarm=0;
    
    path_recog();
    sort();
    rudder_ctrl();
    
  if(LapNum>1) 
 {
  StopFlg=1;
  PWMDTY45=1400;
  if(StopNum>150) 
  {
    PWMDTY45=1000;
  }
 }
 else 
 motor_ctrl(); 
  }

  }/* wait forever */
  /* please make sure that you never leave this function */
}
/////////////////MAIN END//////////////


/////////////////PATH RECOGNISE////////
int8 path_recog(void){
  int16 i;
  int16 sen_num=-6;   //当前灯序号
  int16 sen_cnt=0;    //识别到黑线的灯的个数
  int16 sen_pos[12];  //识别到黑线的灯的序号   
  uint8 mask;
  tmpflg=0;
  
  SensorA=sensorA;
  SensorB=sensorB;
   
  mask=0x20;			//PortA sensor status
  for(i=-10;i<=0;i+=2) 
  {
    if((mask&SensorA)!=0) 
    {
       tmpflg+=i;
       sen_pos[sen_cnt]=sen_num;
       sen_cnt++;  
    }
    sen_num++;
    mask=mask/2;  //mask>>1;
  }
  
  mask=0x01;			//PortB sensor status
  for(i=0;i<=10;i+=2) 
  {
    if((mask&SensorB)!=0) 
    {
       tmpflg+=i;//tmpflg=tmpflg+i-1;
       sen_pos[sen_cnt]=sen_num;
       sen_cnt++;  
    }
    sen_num++;
    mask=mask*2;  //mask<<1;
  }

 if(sen_cnt==0) 
  {
    if(Sensor>=7)                     //没有检测到黑线的位置
		{	 	    
			Sensor=10; PWMDTY23=2220;
		} 
		else if(Sensor<=-7)
		{
			Sensor=-10; PWMDTY23=2700;
		} return 0;
  }
  if(sen_cnt==1);
  else for(i=1;i<sen_cnt;i++)          //识别到黑线的灯相隔一个以上保持
  {
   if(sen_pos[i]-sen_pos[i-1]>=2) 
   {
    return 0;
   }
  }
  if(3<sen_cnt&&sen_cnt<11)
  {
  return 0;
  }
  
   Sensor=tmpflg/sen_cnt;
   
    if(Wflag==1)
   {
    if(Sensor<-7||Sensor>7)
    {
    Sensor=RErrCurr;
    } 
    else Wflag=0;
   }
   
   if((Sensor-RErrCurr)>=8||(Sensor-RErrCurr)<=-8) 
      {Sensor=RErrCurr;}
  
   
   return 0;
 }
 

   /////////////////////////start line/////////////////////////////////
 void startline(void){

  if(       ((sensorA & 0x30)==0x30||(sensorA & 0x18)==0x18)
         // ((sensorB & 0x30)==0x30||(sensorB & 0x18)==0x18)
         && ((sensorA & 0x03)!=0x00||(sensorB & 0x03)!=0x00)
         && ((sensorA & 0x06)!=0x06&&(sensorB & 0x06)!=0x06)) 
         {
          //LapNum++;
          LapFlg1=1;
          FLAG=1;
          LNum=0;
         }
  if(    // ((sensorA & 0x30)==0x00||(sensorA & 0x18)==0x00)
            ((sensorB & 0x30)==0x30||(sensorB & 0x18)==0x18)
         && ((sensorA & 0x03)!=0x00||(sensorB & 0x03)!=0x00)
         && ((sensorA & 0x06)!=0x06&&(sensorB & 0x06)!=0x06)) 
         {
          LapFlg2=1;
          FLAG=1;
          LNum=0;
         }
         
  if(LapFlg1==1&&FLAG==1)     
   {
    if(LapFlg2==1) 
    {
      LapNum++;
      LapFlg1=0;
      LapFlg2=0;
      FLAG=0;
      LNum=0;
      LFlg=1;
    }
    else 
    {
      LNum++;
      if(LNum>=2) 
      {
      LapFlg1=0;
      FLAG=0;
      LNum=0;
      }
    }
   } 
  else if(LapFlg2==1&&FLAG==1)     
   {
    if(LapFlg1==1) 
    {
      LapNum++;
      LapFlg2=0;
      LapFlg1=0;
      FLAG=0;
      LNum=0;
    }
    else 
    {
      LNum++;
      if(LNum>=2) 
      {
      LapFlg2=0;
      FLAG=0;
      LNum=0;
      }
    }
   }
 }

 ////////////////RUDDER CONTRAL/////////
 void rudder_ctrl(void){
  float Kp,Kd;
  if(Sensor>=8||Sensor<=-8) temp=0;
  else if(Sensor>4||Sensor<-4) temp=1;
       //else if(Sensor>2||Sensor<-2) temp=2;
            else temp=2;
  RErrLast=RErrCurr;
  RErrCurr=Sensor;
  if(sf==1) 
  {
  sf=0;  
  //temp=3;
  }
  //if(RErrCurr-RErrLast>=4||RErrCurr-RErrLast<=-4) 
  Kp=Pdelta[temp];//*(float)RUDKP;
  Kd=Ddelta[temp];//*(float)RUDKD;
  //}
  Ruddelta=Kp*RErrCurr+Kd*(RErrCurr-RErrLast);
  RudPWM=2460-Ruddelta;
  if(RudPWM<2230)
     RudPWM=2230;
  if(RudPWM>2690)
     RudPWM=2690;
  PWMDTY23=RudPWM;
 }
 ////////////////PATH CLASSI///////////////
 void sort(void){
 Tar=Sensor;   
      //TarFlg=0;						 	//标志位清零
 if(TarFlg>4)
 {
    TarFlg=0;
      for(j=0;j<num-1;j++) 
      {
       tmp[j]=tmp[j+1]; 
      }
      
      tmp[num-1]=Tar;
      
      for(j=0;j<num;j++) 							  //S弯识别
      {
       if(tmp[j]<-4)
       {
        s1=1;
        break;
       } 
       else
       {
        s1=0;
       }
      }
        
      for(j=0;j<num;j++) 
      {
       if(tmp[j]>=-4&&tmp[j]<=4)
       {
        s2=1;
        break;
       } 
       else
       {
        s2=0;
       }
      }
      
      for(j=0;j<num;j++) 
      {
       if(tmp[j]>4)
       {
        s3=1;
        break;
       } 
       else
       {
        s3=0;
       }
      }
      //////////////
       if(Tar<=-7)                           //右大弯控制
       {
        cnt1++;
        cnt2++;
        cnt3=0;
        cnt4=0;
        cnt5=0;
        if(cnt1>num) 
        {
         flag4=1;
         TarSpeed=Vlcor;
        } 
        else
        {
         TarSpeed=Vlcor;
        }
       }
      if(Tar<=-4&&Tar>-7)											//右小弯控制
      {
       cnt2++;
       cnt1++;
       cnt3=0;
       cnt4=0;
       cnt5=0;
       if(cnt2>num)
       {
        flag4=1;

        TarSpeed=Vscor;
       } 
       else
       {
        TarSpeed=Vscor;
       }          
      }
      
      if(Tar>-4&&Tar<4)										  //直道控制
      {
       cnt3++;          
       cnt1=0;
       cnt2=0;
       cnt4=0;
       cnt5=0;
       if(cnt3>num-5)
       {
        flag4=3;
        TarSpeed=Vline;							    //直道速度
       }
       else
       {
        TarSpeed=Vstri;
       }
      }
      
      if(Tar>=4&&Tar<7)									    //左小弯控制 
      {
       cnt4++;
       cnt1=0;
       cnt2=0;
       cnt3=0;
       cnt5++;
       if(cnt4>num)
       {
        flag4=5;

        TarSpeed=Vscor;								  
       } 
       else
       {
        TarSpeed=Vscor;
       }
      }
      if(Tar>=7)									           //左大弯控制
      {
       cnt5++;
       cnt1=0;
       cnt2=0;
       cnt3=0;
       cnt4++;
       if(cnt5>num)
       {
        flag4=5;

        TarSpeed=Vlcor;								  	
       } 
       else
       {
        TarSpeed=Vlcor;
       }
      }
      
      if(flag4==3) 						         //直道末端制动减速控制
      {
       if(Tar<-4||Tar>4)
       {
        TarSpeed=Vbrek;
        //flag4=0;
       } 
      }      
      if(s2==1&&s1==1&&s3==1) 			   //S弯间断制动控制
      {
        sf=1; 
        t3++;      
       if(Tar>=7||Tar<=-7) 
       {
        TarSpeed=Vsbrk;
       }       
      }
 }
 }
																											 
 ////////////////MOTOR CONTRAL///////////////
 void motor_ctrl(void){
 
  
  SErrPreLast=SErrLast;
  SErrLast=SErrCurr;
  SErrCurr=TarSpeed-Speed;
  //MotDD=(int16)((600*MotDD+400*MOTKD*(SErrCurr+SErrPreLast-2*SErrLast)/190))/1000;//
  Motdelta=(int16)(MOTKP*(SErrCurr-SErrLast)/50+MOTKI*SErrCurr/95);//+MotDD(SErrCurr-SErrLast);
    
    /*if(Motdelta>15)        //PWM is  72.5%    
     {
      Motdelta=15;
     }
  else if(Motdelta<-15)   //PWM is  65%
     {
      Motdelta=-15;
     }*/
     
  MotPWM=MotPWM+Motdelta;
 
     if(MotPWM>1990)        //PWM is  72.5%    
     {
      MotPWM=1990;
     }
     else if(MotPWM<50)   //PWM is  65%
     {
      MotPWM=50;
     }
     
  PWMDTY45=MotPWM;

 }
 
  ////////////////////////////////////sci receive isr//////////////////////////////////////////////// 
#pragma CODE_SEG __NEAR_SEG NON_BANKED             
interrupt void sci0(void){        //receive interruption
int8 data_rec,digi[7];    
int8 seq=0;//flagSci;
int16 timerSCI,iCoeRec;
int16 dispflag;
//byte iDis,iCol;
//byte ledstate[16],leddata,i ;
//int temp;

 //pointPos=0;
  dispflag=WRONG;
 //DisableInterrupts;
 
for(seq=0;seq<7;seq++){             //receive sequency;
  while(SCI0SR1_RDRF==0){
    timerSCI++;
    if(timerSCI>4000) {
      display('F','E',0);
     // EnableInterrupts; 
      return;
    }
  }         //receive buffer full flag; 
  timerSCI=0;

data_rec=SCI0DRL;                               
digi[seq]=data_rec;


switch(digi[seq]){                //change the ASCII to decimal;
  case 48 : digi[seq]=0;    break;
  case 49 : digi[seq]=1;    break;
  case 50 : digi[seq]=2;    break;
  case 51 : digi[seq]=3;    break;
  case 52 : digi[seq]=4;    break;
  case 53 : digi[seq]=5;    break;
  case 54 : digi[seq]=6;    break;
  case 55 : digi[seq]=7;    break;
  case 56 : digi[seq]=8;    break;
  case 57 : digi[seq]=9;    break;
  //case '.': digi[seq]=0;    pointPos=seq; break;
}

}
  iCoeRec=digi[2]*1000+digi[3]*100+digi[4]*10+digi[5];
  
    if(digi[0]=='r'&&digi[1]=='p') {  //steer_kp    
      if(digi[6]=='w'){ 
           Pdelta[1]=iCoeRec;
        }
           display('r','p',Pdelta[1]);
           dispflag=OK;
    }
    if(digi[0]=='r'&&digi[1]=='d') {  //steer_kd    
      if(digi[6]=='w'){ 
           Ddelta[1]=iCoeRec;
        }
           display('r','d',Ddelta[1]);
           dispflag=OK;
    }
    /////    
    if(digi[0]=='r'&&digi[1]=='q') {  //steer_kp    
      if(digi[6]=='w'){ 
           Pdelta[2]=iCoeRec;
        }
           display('r','p',Pdelta[2]);
           dispflag=OK;
    }
    if(digi[0]=='r'&&digi[1]=='e') {  //steer_kd    
      if(digi[6]=='w'){ 
           Ddelta[2]=iCoeRec;
        }
           display('r','d',Ddelta[2]);
           dispflag=OK;
    }
        if(digi[0]=='m'&&digi[1]=='p') {  //steer_kp    
      if(digi[6]=='w'){ 
           t1=iCoeRec;
        }
           display('m','p',t1);
           dispflag=OK;
    }
    if(digi[0]=='m'&&digi[1]=='i') {  //steer_kd    
      if(digi[6]=='w'){ 
           t2=iCoeRec;
        }
           display('m','i',t2);
           dispflag=OK;
    }
    if(digi[0]=='t'&&digi[1]=='i') {  //steer_kd    
      if(digi[6]=='w'){ 
           t3=iCoeRec;
        }
           display('t','i',t3);
           dispflag=OK;
    }
    if(digi[0]=='t'&&digi[1]=='p') {  //steer_kd    
      if(digi[6]=='w'){ 
           MOTKP=iCoeRec;
        }
           display('t','p',MOTKP);
           dispflag=OK;
    }
    if(digi[0]=='v'&&digi[1]=='s') {  //speed_kp    
      if(digi[6]=='w'){ 
           TarSpeed=iCoeRec;
        }
           display('v','t',TarSpeed);
           display('v','n',Speed);
           dispflag=OK;
    } 
      
    if(digi[0]=='r'&&digi[1]=='n') {  //speed_kd    
      if(digi[6]=='w'){ 
           Speed=iCoeRec;
        }
           display('r','n',RudPWM);
           dispflag=OK;
    }            
    if(digi[0]=='s'&&digi[1]=='e') {  //speed_fast    
      if(digi[6]=='w'){ 
           ;
        }
           display('s','a',SensorA);
           display('s','b',SensorB);
           dispflag=OK;
    }
  

    if(digi[0]=='s'&&digi[1]=='l') {  //speed_slow    
      if(digi[6]=='w'){ 
           //speed_slow=iCoeRec;
        }
           display('s','l',LapNum);
           dispflag=OK;
    }     
    if(digi[0]=='v'&&digi[1]=='b') {  //speed_middle    
      if(digi[6]=='w'){ 
           Vbrek=iCoeRec;
        }
           display('v','b',Vbrek);
           dispflag=OK;
    }
    if(digi[0]=='v'&&digi[1]=='p') {  //speed_max    
      if(digi[6]=='w'){ 
           Vsbrk=iCoeRec;
        }
           display('v','p',Vsbrk);
           dispflag=OK;
    }
    if(digi[0]=='v'&&digi[1]=='d') {  //speed_min    
      if(digi[6]=='w'){ 
           Vlcor=iCoeRec;
        }
           display('v','d',Vlcor);
           dispflag=OK;
    }   
    if(digi[0]=='v'&&digi[1]=='x') {  //speed_chg_max    
      if(digi[6]=='w'){ 
           Vscor=iCoeRec;
        }
           display('v','x',Vscor);
           dispflag=OK;
    }
    if(digi[0]=='v'&&digi[1]=='l') {  //speed_max    
      if(digi[6]=='w'){ 
           Vline=iCoeRec;
        }
           display('v','l',Vline);
           dispflag=OK;
    }
         
      /*     if(digi[0]=='s'&&digi[1]=='2') {  //speed_max    
      if(digi[6]=='w'){ 
           steer_kp2=iCoeRec;
        }
           display('s','2',steer_kp2);
           dispflag=OK;
    }
         
         
         
         
            if(digi[0]=='s'&&digi[1]=='g') {  //led_state    
      
           display('s','g',speed_goal);
           dispflag=OK;
    } 
          if(digi[0]=='s'&&digi[1]=='n') {  //led_state    
      
           display('s','n',speed_now);
           dispflag=OK;
    } 
    
            if(digi[0]=='s'&&digi[1]=='a') {  //led_state    
      
           display('s','a',steer_angle);
           dispflag=OK;
    } */
     if(digi[0]=='a'&&digi[1]=='l') {  //speed_slow 
                                             
        
            display('r','p',Pdelta[1]);
            display('r','d',Ddelta[1]);
            display('v','t',TarSpeed);
            display('v','n',Speed);
            display('s','a',sensorA);
            display('s','b',sensorB);
            display('s','l',LapNum);
            //display('s','p',plsnum);
            //display('v','s',speed_slow);
            //display('v','x',speed_max);
            //display('v','n',speed_min);
            //display('v','c',speed_chg_max);
            
           dispflag=OK;
    }
    
    /*  if(digi[0]=='l'&&digi[1]=='n') {  //led_state    
      
           display('l','n',lapcnt);
           dispflag=OK;
    }    */
    
     if(!dispflag)
     {
       display('*','*',0);
     }
     
//EnableInterrupts;  
}
