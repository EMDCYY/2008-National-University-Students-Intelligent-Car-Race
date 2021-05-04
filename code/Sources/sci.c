#include "sci.h" 
#include <mc9s12dp512.h>     /* derivative information */

 int modify(volatile int* pEeprom,int mv){
ECLKDIV=0X4A; 
 ESTAT_PVIOL=0;           //make sure the error flags are cleared;
 ESTAT_ACCERR=0;
 
while(ESTAT_CBEIF==0){
}
*pEeprom=mv;
 ECMD=0X60;             //FCMD_CMDB6=1;
ESTAT_CBEIF=1;
while(ESTAT_CCIF==0){
}
delay(10);
return *pEeprom;
 
}


void delay(int n){
int t ;
for(t=0;t<n;t++){
}
}



byte commandRec(char cCoe1Img, char cCoe2Img, int* pCoeNam,int iCoeValImg,volatile int*pEEDImg,byte digi6Img){
  int NewEeprom;
  
  switch(digi6Img){
    case 's' : display(cCoe1Img,cCoe2Img,*pCoeNam); break;
    
    case 'r' : *pCoeNam=iCoeValImg; 
               display(cCoe1Img,cCoe2Img,*pCoeNam); break;
    
    case 'e' : NewEeprom=modify(pEEDImg,iCoeValImg);
               *pCoeNam=NewEeprom; 
               display(cCoe1Img,cCoe2Img,*pCoeNam ); break;

    default : return 0;    
 }
 return 1;
}



///////////////////////////////////////display command////////////////////////////////////////
void display(char cFunc1Dis ,char cFunc2Dis, int iCoeDis){      //666666666666666666666
 int i_sci;
 byte b[4];
 byte data_sent;  
 while(SCI0SR1_TDRE==0);            //display func1 id
 data_sent=cFunc1Dis;
 SCI0DRL=data_sent;

 while(SCI0SR1_TDRE==0);             //display func2 id
 data_sent=cFunc2Dis;
 SCI0DRL=data_sent;
 
 b[0]=(char)(iCoeDis/1000);            //disassemble the coefficient to digits;
 b[1]=(char)((iCoeDis%1000)/100);
 b[2]=(char)((iCoeDis%100)/10);
 b[3]=(char)(iCoeDis%10);
 
 for(i_sci=0;i_sci<4;i_sci++){                    //display digits of the coefficient;
 switch(b[i_sci]){
  case 0 :  data_sent=48; break;
  case 1 :  data_sent=49; break;
  case 2 :  data_sent=50; break;
  case 3 :  data_sent=51; break;
  case 4 :  data_sent=52; break;
  case 5 :  data_sent=53; break;
  case 6 :  data_sent=54; break;
  case 7 :  data_sent=55; break;
  case 8 :  data_sent=56; break;
  case 9 :  data_sent=57; break;
  } 
  
  while(SCI0SR1_TDRE==0); 
  SCI0DRL=data_sent;
 }        //end for p;
 
 while(SCI0SR1_TDRE==0);   //tab
 data_sent=0x09 ;
 SCI0DRL=data_sent;         //
 
}
