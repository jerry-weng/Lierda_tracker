#ifndef __UNIX_H
#define __UNIX_H
#include <stdbool.h>


#define UTC_BASE_YEAR 1970
#define MONTH_PER_YEAR 12
#define DAY_PER_YEAR 365
#define SEC_PER_DAY 86400
#define SEC_PER_HOUR 3600
#define SEC_PER_MIN 60
/* 自定义的时间结构体 */
typedef struct
{
    unsigned short nYear;
    unsigned char nMonth;
    unsigned char nDay;
    unsigned char nHour;
    unsigned char nMin;
    unsigned char nSec;
    unsigned char DayIndex;
} mytime_struct;

extern mytime_struct my_time;
extern void utc_sec_2_mytime(uint32_t utc_sec, mytime_struct *result, bool daylightSaving);

#endif