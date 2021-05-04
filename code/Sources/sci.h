#ifndef _SCI_H
#define _SCI_H

/////////////////////////////////SCI platform functions declare////////////////////
 void Sci0Intl(void);
 void Dis3Digi(int);
 void  display(char,char,int);
 
void delay(int);
int modify(volatile int*,int);
unsigned char commandRec(char , char , int* ,int ,volatile int*,unsigned char);
unsigned char FcommandRec(char , char , float* ,int ,volatile int*,unsigned char);
void CoeIntl(void);
 //////////////////////////////////////////////////////////////////////////////////

#endif
