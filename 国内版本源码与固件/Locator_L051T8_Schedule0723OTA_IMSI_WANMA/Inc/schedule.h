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

//Scheduleģʽ     
#define  SCHEDULE_MODE_DAYCYCLE           1    //�����ظ�    
#define  SCHEDULE_MODE_WEEKCYCLE          2       
#define  SCHEDULE_MODE_WEEKCYCLEONETIME   3    //�����ظ�һ�� 

#define  CYCLEDAYS_MAX                    7    //�����7���ظ�
#define  HOURS_GROUP_MAX                  24   //����     

#define  DEFAULT_S_CYCLE                  0xFFFF //Ĭ��scheduleģʽ����
     
//ʱ��ͬ����ʱ
#define  TIME_SYN_TIMEOUT                 5000
#define  TIME_SYN_RETRY_NUM_MAX           3       //�ط�����
#define  TIME_SYN_EVENT                   0x00020000
     
#define SCHEDULE_UPLOAD_ID                0xD3//����scheduleģʽ����     
#define SCHEDULE_CONFIG_ID                0xD4//���ûظ�
#define TIME_SYN_REQ_ID                   0xD5//����ͬ��ʱ��� 

#define SERVER_ID_SCHEDULE_CONFIG         0xC2   //�������·�����configģʽ
#define SERVER_ID_TIME_SYN                0xC3   //�������·�ʱ���
#define SERVER_ID_DEVICE_TIME_SYN         0xC4   //�������·�,ʹ�豸�´��ϱ�ʱ����ͬ��ʱ�������

#define PACKAGE_NO_FINISH                 0xAA   //��������Ƿ��Ѿ��������

#pragma  pack(1)
     
//Scheduleģʽ�����ڣ�����RTC��ʱ
typedef struct
{
    uint8_t  mode;     //Tracking����Findnow
    uint16_t cycle;   //����
    uint16_t config_cycle;//��ģʽ�����õ�����
}
Schedule_Mode_t; 

//typedef struct
//{
//    uint16_t  RSPR;
//    uint16_t  SINR;
//    uint8_t   ECL;
//    uint16_t  RSRQ;
//    uint32_t  cellID;                  //С��ID
//}
//Signal_Level_t; 

typedef struct
{
    uint32_t  NCCIDH;
    uint8_t   ECL;
    uint16_t  RSRQ;
    uint32_t  NCCIDL;                  //SIM����
}
Signal_Level_t; 


//�����·�
//���ݸ�ʽ 0xD3
//typedef struct{
//    uint16_t  cycle;                   //�ϱ����
//    uint8_t   Date;                    //ѭ���ĵڼ���/�ܼ� 
//    uint8_t   scheduleID;
//    uint8_t   Mode;                    //0x00 - FindNow 0x01- Tracking
//    uint16_t  RSPR;
//    uint16_t  SINR;
//    uint8_t   ECL;
//    uint16_t  RSRQ;
//    uint32_t  cellID;                  //С��ID
//    uint8_t   battery_Lv;              //��ص����ٷֱ�
//    uint16_t  GPS_data_len;
//    uint8_t   GPS_List_num;
//    /*------------------GPS�������---------------------------*/    
//}Schedule_Data_upload_t;


typedef struct{
    uint16_t  cycle;                   //�ϱ����
    uint8_t   Date;                    //ѭ���ĵڼ���/�ܼ� 
    uint8_t   scheduleID;
    uint8_t   Mode;                    //0x00 - FindNow 0x01- Tracking
    uint32_t  NCCIDH;
    uint8_t   ECL;
    uint16_t  RSRQ;
    uint32_t  NCCIDL;                  //SIM����
    uint8_t   battery_Lv;              //��ص����ٷֱ�
    uint16_t  GPS_data_len;
    uint8_t   GPS_List_num;
    /*------------------GPS�������---------------------------*/    
}Schedule_Data_upload_t;

//typedef struct
//{
//    uint8_t Num;                                 //���  
//    uint8_t ScheduleDateNum;                     //���ں�
//    uint16_t Reserve;                            //Ԥ��
//    uint8_t State;                               //�����Ƿ���Ч  01��Ч��00��Ч
//    uint8_t Longitude[NB_LONGITUDE_LEN];         //����dddmm.mmm
//    uint8_t Latitude[NB_LATITUDE_LEN];           //γ��ddmm.mmmm
//    uint8_t EWNS_Indicator;                      //�����ϱ�����ָʾ
//    uint8_t UTC_Time[NB_UTC_LEN];                //UTCʱ��
//    uint8_t Date_Value[NB_DATE_LEN];             //����
//}Schedule_GPS_Data_t;


/*----------------------������������Calendar_t-------------------------*/
typedef struct{
    uint8_t mode;           //Tracking(FindNow)
    uint8_t start_Time;     //��ʼʱ��(0~23��)
    uint8_t end_Time;       //����ʱ��(1~24��) >��ʼʱ�䡣��1СʱΪ��λ
    uint16_t period;        //ģʽ�µ��ϱ����ڣ���С�ڵ��ڽ���ʱ��-��ʼʱ��mim��
}Calendar_t;


/*------------------�������·�����---------------------------*/ 
//����scheduleģʽ 0xC2
typedef struct{
    uint8_t reserve;                 //Ԥ��
    uint8_t scheduleID;
    uint8_t seqNum;
    uint8_t ModeFlag;                //0x01 �C �����ظ� 0x02 �C �����ظ� 0x03 �C �����ظ�1��
    uint8_t scheduleDays;            //ѭ����Χ
    uint16_t scheduleDataLen;        //�������ݳ���
    uint8_t scheduleDataGroupNum;    //������������
}Schedule_CMD_t;


typedef struct{
    uint8_t schedulePeriod;       //����
    uint8_t calendar_len;         //�����������ݳ���
    uint8_t calendar_num;         //����������������        
}Schedule_Data_t;

typedef struct{
    uint8_t mode;         //01-FindNow 02-Tracking
    uint8_t startH;       //��ʼʱ�䣨0-23�㣩��1СʱΪ��λ         
    uint8_t endH;         //����ʱ�䣨1-24�㣩��1СʱΪ��λ         
    uint16_t cycle;       //�ϱ����� ��minΪ��λ    
}Schedule_DayConfig_t;

//C2����ִ�н��
typedef struct{
    uint8_t cmdResult;
    uint8_t packetNum;
    uint8_t configMode;
    uint8_t dayrange;
}Schedule_CMD_State_t;

//ͬ��ʱ���0xC4
typedef struct{
    uint8_t Local_Time[3];             //����ʱ��
    uint8_t week;                      //����ʱ���ܼ�
    uint8_t Date_Value[3];             //�������� 
}Time_syn_Data_t;


//�л�������
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