#ifndef _COMMON_H
#define _COMMON_H

typedef unsigned char UINT8;        /*unsigned 8 bit definition */
typedef unsigned short UINT16;      /*unsigned 16 bit definition*/
typedef unsigned long UINT32;       /*unsigned 32 bit definition*/
typedef signed char INT8;           /*signed 8 bit definition */
typedef short INT16;                /*signed 16 bit definition*/
typedef long int INT32;             /*signed 32 bit definition*/

#define TRUE    1
#define FALSE   0

#define F_BUS (24000000UL)

#define min(a, b) (((a) > (b)) ? (b) : (a))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define limit(x,a,b) ((x) > (b) ? (b) : ((x) < (a) ? (a) : (x)))

#endif
