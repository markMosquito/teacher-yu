#ifndef __MYMATH_H_
#define __MYMATH_H_

//计算用宏
#include <math.h>
#define pi              (3.14159265358979323846f)

#define rad2ang(rad)  ((float)(rad)/pi*180.0f)
#define ang2rad(ang)  ((float)(ang)*pi/180.0f)

#define square(x)     ((x)*(x))

#if defined ( __ICCARM__ )
    #define my_sqrt(x)     sqrtf(x)
#else
    #define my_sqrt(x)     _sqrtf(x)
#endif

#define my_sin(x)      sinf(ang2rad(x))
#define my_cos(x)      cosf(ang2rad(x))
#define my_acos(x)     rad2ang(acosf(x))
#define my_max(a,b)    ((a)>(b) ? (a) : (b))
#define my_abs(x)      ((x) > 0 ? (x) : (-(x)))

//浮点数相关
#define f_equal(a,b)   (fabsf((a)-(b))<0.000001f)
#define f_unequal(a,b) (fabsf((a)-(b))>0.000001f)
#define f_greater(a,b) ((a)-(b)>0.000001f)
#define f_smaller(a,b) ((a)-(b)<-0.000001f)

#define Get_Lenth(x,y) my_sqrt(square(x) + square(y))
#endif
