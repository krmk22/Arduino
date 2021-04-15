#ifndef _UTILS_H
#define _UTILS_H

#ifndef F_CPU 
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#ifndef ClockConfig
#define ClockConfig
#define OneMachineCycle  0.0625F
#define MachineCycleUS   16
#endif

#define DelayUS(Delay)\
{\
  unsigned int ReqDelay = Delay;\
  while(--ReqDelay) _delay_us(1);\
}

#define DelayMS(Delay)\
{\
  register unsigned char RoutineDelay;\
  register unsigned int RequireDelay = Delay;\
  do\
  {\
    RoutineDelay = 4;\
    do\
    {\
      _delay_us(250);\
    }\
    while(--RoutineDelay);\
  }\
  while(--RequireDelay);\
}

#ifndef NumberSystem
#define NumberSystem
#define BIN     2
#define OCT     8
#define DEC     10
#define HEX     16
#endif

#ifndef LogicalConfig
#define LogicalConfig
#define True    1
#define False   0
#define Null    '\0'
#endif

#ifndef SignSystem
#define SignSystem
#define Sign    True
#define NoSign  False
#endif

extern long MapValue(long, long, long, long, long);
extern int sstrlen(const char*, int);
extern char *smemchr(register const char*, int, int);
extern char *smemrchr(register const char*, int, int);
extern void *smemmem(const void*, int, const void*, int);
extern double power(double, int);
extern int dtoa(unsigned long, char*, int, const char);
extern int ftostra(float, int, int, char*, const char);
extern char SplitString(unsigned char*, unsigned char*, const char*, unsigned char);
#endif
  
