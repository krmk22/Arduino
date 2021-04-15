#include <avr/io.h>
#include <string.h>
#include <ctype.h>

#include "utils.h"

long MapValue(long MainValue, long StartMin, long StartMax, long EndMin, long EndMax)
{
  return (MainValue - StartMin) * (EndMax - EndMin) / (StartMax - StartMin) + EndMin;
}

int sstrlen(const char *MainString, int SizeOfMainString)
{
  if(SizeOfMainString) 
  while(!MainString[--SizeOfMainString]) continue;
  return SizeOfMainString;
}

char *smemchr(register const char *MainString, int SubString, int Length)
{
  const unsigned char *LocalPointer = (const unsigned char*) MainString;
  while(Length --> 0)
  if(*LocalPointer == SubString) return (char*)LocalPointer;
  else LocalPointer++;
  return Null;
}

char *smemrchr(register const char *MainString, int SubString, int Length)
{
  char *LocalPointer = (char*)MainString;
  while(Length --> 0)
  if(MainString[Length] == SubString)
  return (char*)LocalPointer + Length;
  return Null;
}

void *smemmem(const void *MainString, int LengthOfMainString, const void *SubString, int LengthOfSubString)
{
  const char *cur, *last;
  const char *cl = (char*)MainString;
  const char *cs = (char*)SubString;
  
  if((!LengthOfMainString && !LengthOfSubString) ||
  (LengthOfMainString < LengthOfSubString)) return Null;
  if(LengthOfSubString == 1) return smemchr((char*)MainString, *cs, LengthOfMainString);  
  
  last = cl + LengthOfMainString - LengthOfSubString;
  for(cur = cl; cur <= last; cur++)
  if(cur[0] == cs[0] && memcmp(cur, cs, LengthOfSubString) == 0)
  return (void*)cur;
  return Null;
}

double power(double x, int n)
{
  double result = 1;
  int minus = 1;
  
  if(n < 0)
  {
    minus = -1;
    n = -n;
  }
  
  if(0 == n) return True;
  if(0 == x) return False;
  
  while(n)
  {
    if(n & 1) result *= x;
    x *= x; n /= 2;
  }
  
  if(minus < 0) return 1.0 / result;
  else return result;
}

int dtoa(unsigned long int Value, char *StoreArray, int EndPoint, const char Base)
{
  unsigned int CrX;
  unsigned int TpT = EndPoint;
  StoreArray += EndPoint;
  if(!EndPoint) return False;
  
  do
  {
    CrX = Value % Base; Value /= Base;
    if(CrX >= 10) CrX += 'A' - '0' - 10;
    CrX += '0'; (*(--StoreArray)) = CrX;
  }
  while(--EndPoint);
  
  StoreArray += TpT;
  (*(StoreArray++)) = '\0';
  return True;
}

int ftostra(float Value, int PostFix, int PreFix, char *StoreArray, const char Sym)
{
  if(Sym == Sign)
  {
    (*(StoreArray++)) = (Value < 0 ?'-' :'+');
    Value = (Value < 0 ? -Value :Value);
  }
  
  int ipart = (int)Value;
  float fpart = Value - (float)ipart;
  
  if(!PostFix || !PreFix) return False;
  dtoa(ipart, StoreArray, PostFix, DEC);
  StoreArray += PostFix;
  (*(StoreArray++)) = '.';
  unsigned int fcon = (int)(fpart * power(10, PreFix));
  dtoa(fcon, StoreArray, PreFix, DEC);
  return True;
}

char SplitString(unsigned char *MdA, unsigned char *SdA, const char *SoS, unsigned char EoS)
{
  unsigned char *StS = strstr((const char*)MdA, SoS);
  unsigned char *EdS = strchr((const char*)StS, EoS);
  if(StS == '\0'|| EdS == '\0') return 0;
  for(unsigned char *LsV = StS + strlen(SoS); LsV < EdS; LsV++)
  *SdA++ = (isdigit(LsV[0]) || LsV[0] == '.' ?LsV[0] :'\0'); 
  while(*SdA) *SdA++ = '\0';
  return 1;
}
