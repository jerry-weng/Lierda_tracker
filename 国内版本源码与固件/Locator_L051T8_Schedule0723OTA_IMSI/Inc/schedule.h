/*******************************************************************************
    Copyright:       Lierda WSN BT team
    Filename:        schedule.h
    Description:     schedule
    FileVersion:     V1.0

    ChangeLog: 

    =========================================
    date:18/02/27
    author:chenkl
    log: Create File
 
*******************************************************************************/
#ifndef SCHEDULE_H
#define SCHEDULE_H

#ifdef __cplusplus
 extern "C" {
#endif
     
#include "gps.h"
#include "User_ClockDriver.h"
#include "nb.h"

//Schedule模式     
#define  SCHEDULE_MODE_DAYCYCLE           1    //按天重复    
#define  SCHEDULE_MODE_WEEKCYCLE          2       
#define  SCHEDULE_MODE_WEEKCYCLEONETIME   3    //按周重复一次 

#define  CYCLEDAYS_MAX                    7    //最大以7天重复
#define  HOURS_GROUP_MAX                  24   //组数     

#define  DEFAULT_S_CYCLE                  0xFFFF //默认schedule模式周期
     
//时间同步超时
#define  TIME_SYN_TIMEOUT                 5000
#define  TIME_SYN_RETRY_NUM_MAX           3       //重发次数
#define  TIME_SYN_EVENT                   0x00020000
     
#define SCHEDULE_UPLOAD_ID                0xD3//发送schedule模式数据     
#define SCHEDULE_CONFIG_ID                0xD4//配置回复
#define TIME_SYN_REQ_ID                   0xD5//请求同步时间戳 

#define SERVER_ID_SCHEDULE_CONFIG         0xC2   //服务器下发进入config模式
#define SERVER_ID_TIME_SYN                0xC3   //服务器下发时间戳
#define SERVER_ID_DEVICE_TIME_SYN         0xC4   //服务器下发,使设备下次上报时带上同步时间戳请求

#define PACKAGE_NO_FINISH                 0xAA   //多包数据是否已经接收完成

#pragma  pack(1)
     
//Schedule模式与周期，用于RTC计时
typedef struct
{
    uint8_t  mode;     //Tracking或者Findnow
    uint16_t cycle;   //周期
    uint16_t config_cycle;//该模式下配置的周期
}
Schedule_Mode_t; 

//typedef struct
//{
//    uint16_t  RSPR;
//    uint16_t  SINR;
//    uint8_t   ECL;
//    uint16_t  RSRQ;
//    uint32_t  cellID;                  //小区ID
//}
//Signal_Level_t; 

typedef struct
{
    uint32_t  NCCIDH;
    uint8_t   ECL;
    uint16_t  RSRQ;
    uint32_t  NCCIDL;                  //SIM卡号
}
Signal_Level_t; 


//数据下发
//数据格式 0xD3
//typedef struct{
//    uint16_t  cycle;                   //上报间隔
//    uint8_t   Date;                    //循环的第几天/周几 
//    uint8_t   scheduleID;
//    uint8_t   Mode;                    //0x00 - FindNow 0x01- Tracking
//    uint16_t  RSPR;
//    uint16_t  SINR;
//    uint8_t   ECL;
//    uint16_t  RSRQ;
//    uint32_t  cellID;                  //小区ID
//    uint8_t   battery_Lv;              //电池电量百分比
//    uint16_t  GPS_data_len;
//    uint8_t   GPS_List_num;
//    /*------------------GPS填充数据---------------------------*/    
//}Schedule_Data_upload_t;


typedef struct{
    uint16_t  cycle;                   //上报间隔
    uint8_t   Date;                    //循环的第几天/周几 
    uint8_t   scheduleID;
    uint8_t   Mode;                    //0x00 - FindNow 0x01- Tracking
    uint32_t  NCCIDH;
    uint8_t   ECL;
    uint16_t  RSRQ;
    uint32_t  NCCIDL;                  //SIM卡号
    uint8_t   battery_Lv;              //电池电量百分比
    uint16_t  GPS_data_len;
    uint8_t   GPS_List_num;
    /*------------------GPS填充数据---------------------------*/    
}Schedule_Data_upload_t;

//typedef struct
//{
//    uint8_t Num;                                 //序号  
//    uint8_t ScheduleDateNum;                     //周期号
//    uint16_t Reserve;                            //预留
//    uint8_t State;                               //数据是否有效  01有效，00无效
//    uint8_t Longitude[NB_LONGITUDE_LEN];         //经度dddmm.mmm
//    uint8_t Latitude[NB_LATITUDE_LEN];           //纬度ddmm.mmmm
//    uint8_t EWNS_Indicator;                      //东西南北半球指示
//    uint8_t UTC_Time[NB_UTC_LEN];                //UTC时间
//    uint8_t Date_Value[NB_DATE_LEN];             //日期
//}Schedule_GPS_Data_t;


/*----------------------当天数据内容Calendar_t-------------------------*/
typedef struct{
    uint8_t mode;           //Tracking(FindNow)
    uint8_t start_Time;     //开始时间(0~23点)
    uint8_t end_Time;       //结束时间(1~24点) >开始时间。以1小时为单位
    uint16_t period;        //模式下的上报周期（需小于等于结束时间-开始时间mim）
}Calendar_t;


/*------------------服务器下发数据---------------------------*/ 
//配置schedule模式 0xC2
typedef struct{
    uint8_t reserve;                 //预留
    uint8_t scheduleID;
    uint8_t seqNum;
    uint8_t ModeFlag;                //0x01 – 按天重复 0x02 – 按周重复 0x03 – 按周重复1次
    uint8_t scheduleDays;            //循环范围
    uint16_t scheduleDataLen;        //后续数据长度
    uint8_t scheduleDataGroupNum;    //后续数据组数
}Schedule_CMD_t;


typedef struct{
    uint8_t schedulePeriod;       //天数
    uint8_t calendar_len;         //后续当天数据长度
    uint8_t calendar_num;         //后续当天数据组数        
}Schedule_Data_t;

typedef struct{
    uint8_t mode;         //01-FindNow 02-Tracking
    uint8_t startH;       //开始时间（0-23点）以1小时为单位         
    uint8_t endH;         //结束时间（1-24点）以1小时为单位         
    uint16_t cycle;       //上报周期 以min为单位    
}Schedule_DayConfig_t;

//C2命令执行结果
typedef struct{
    uint8_t cmdResult;
    uint8_t packetNum;
    uint8_t configMode;
    uint8_t dayrange;
}Schedule_CMD_State_t;

//同步时间戳0xC4
typedef struct{
    uint8_t Local_Time[3];             //本地时间
    uint8_t week;                      //本地时间周几
    uint8_t Date_Value[3];             //本地日期 
}Time_syn_Data_t;


//切换至今天
typedef struct{
    uint8_t DaySwitchFlag;
    uint8_t TimeSynFlag;
    uint8_t CurrentDayInTheLoop;
    uint8_t RTC_ERROR;
}switchToToday_t;
#pragma  pack()
     
extern RTC_TimeTypeDef stimestructure;
extern RTC_DateTypeDef sdatestructure;
extern Signal_Level_t Signal_Level;
extern uint8_t zoneTime;
extern uint8_t Time_synFlag;
extern uint8_t Time_synInitFlag;
extern Schedule_CMD_State_t Schedule_CMD_State;
extern uint8_t Schedule_ModeInitFlag;
extern uint8_t Schedule_ModeFlag;
extern Schedule_Mode_t Schedule_Mode_Data;
extern uint8_t CurrentDayInTheLoop;
extern uint8_t Time_Syn_overFlow;
extern Clock_Struct TimeSynclock;
extern Schedule_Data_upload_t Schedule_Data_upload;
extern uint8_t schedule_congif_Flag;
extern uint8_t Time_syning_Flag;

extern uint8_t Schedule_State;
extern uint8_t Schedule_StateStartFlag;
extern uint8_t Schedule_StateSend_counter;
extern uint8_t cycleStartFlag;
extern uint8_t DaySwitchCounter;

extern Calendar_t CalendarList[CYCLEDAYS_MAX][HOURS_GROUP_MAX];
extern Calendar_t CalendarList_Temp[CYCLEDAYS_MAX][HOURS_GROUP_MAX];
extern Schedule_CMD_State_t Handel_ServerData_Schedule_Config(uint8_t* RxMessage, uint16_t dataLen);
extern void Schedule_Device_Confirm_Report(uint8_t CMD_ID, uint8_t serviceCMD_Process_result, uint8_t packetSeq, uint8_t zoneTime, uint8_t TimeSynFlag);
extern void Schedule_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp, Schedule_Data_upload_t Schedule_Data_upload, GPS_Info_t Schedule_GPS_Info);
extern uint8_t Handel_ServerData_TimeSyn(uint8_t* RxMessage, uint16_t dataLen);
extern switchToToday_t switchToToday(uint8_t schedule_mode);

extern void TLocator_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp, Locator_Data_t Locator_Data_tmp, GPS_Info_t GPS_Info_tmp, Signal_Level_t Signal_Level);
extern Schedule_Mode_t getFollowedMode(uint8_t week);
extern uint8_t endWeekCycle(void);
extern void Time_synchronization(void);
extern void Time_syn_RetryTimerInit(void);
extern uint8_t getRTC_Date_Time(void);
extern uint8_t setRTC_Date_Time(uint8_t yy, uint8_t mm, uint8_t dd, uint8_t hours, uint8_t mins, uint8_t sec, uint8_t week);
extern void Locator_Data_ReportSleep( NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp );

#ifdef __cplusplus
 }
#endif
   
#endif