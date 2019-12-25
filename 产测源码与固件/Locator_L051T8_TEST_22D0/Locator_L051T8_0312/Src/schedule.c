/*******************************************************************************
Copyright:       Lierda WSN BT team
Filename:        schedule.c
Description:     schedule_mode
FileVersion:     V1.0

ChangeLog: 

=========================================
date:18/2/27
author:chenkl
log: Create File

*******************************************************************************/


/*******************************************************************************
* INCLUDES
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_it.h"
#include "User_ClockDriver.h"
#include "rtc.h"
#include "usart.h"
#include "tools.h"
#include "schedule.h"
#include "main.h"
#include "nb.h"
/*********************************************************************
* TYPEDEFS
*/
/*********************************************************************
* MACROS
*/
/*********************************************************************
* PUBLIC VARIABLES 
*/
RTC_TimeTypeDef stimestructure;
RTC_DateTypeDef sdatestructure;
Clock_Struct TimeSynclock;

//��������ʱ�ط�
uint8_t Time_Syn_Retry_Num;

//�����Ƿ�ͬ������־λ
uint8_t Time_synFlag = RESET;

//�����������ͬ��ʱ�����־λ
uint8_t Time_synReq = 0x00;

/*Schedule���ûظ���0xD5)*/
//����ִ�н��
uint8_t ScheduleConfigResult; 
//�ְ����
uint8_t ScheduleConfigSeqNum; 

//ʱ��
uint8_t zoneTime;

uint8_t Schedule_Mode;
//��ǰ����ѭ���еĵڼ���
uint8_t CurrentDayInTheLoop;

Schedule_Data_t Schedule_Data;
Calendar_t Calendar;
//scheduleģʽ���ñ�
Calendar_t CalendarList[CYCLEDAYS_MAX][HOURS_GROUP_MAX];
//��ʱ�洢�ϴε����ñ��Է�����ʧ��
Calendar_t CalendarList_Temp[CYCLEDAYS_MAX][HOURS_GROUP_MAX] = {0};
//scheduleģʽ��Ϣ
Schedule_CMD_t Schedule_CMD;
//�ź�ǿ��
Signal_Level_t Signal_Level;
/*********************************************************************
* User Timer API
*/

/*
* @fn       setRTC_Date_Time
* @brief    �趨RTCʱ�䣬�����գ�ʱ���룬����
* @param   
* @return   
*/
uint8_t setRTC_Date_Time(uint8_t yy, uint8_t mm, uint8_t dd, uint8_t hours, uint8_t mins, uint8_t sec, uint8_t week)
{
    uint8_t dateCorrectFlag = ERROR;
    //��С���ж�
    if( mm==4 || mm==6 )
    {
        if(dd>30)
        {
            return ERROR;
        }
    }
    if( mm==9 || mm==11 )
    {
        if(dd>30)
        {
            return ERROR;
        }
    }
    //������ж� -- ��������������365.24219�졣����һ�򣬰��겻���İ�������
    if( yy%4 != 0 )
    {
        //���������28��Ϊ�쳣
        if( mm==2 && dd>28 )
        {
            return ERROR;
        }
    }
    else
    {
        //�������29��Ϊ�쳣
        if( mm==2 && dd>29 )
        {
            return ERROR;
        }
    }
    //�ж������ʽ
    if( mm>0 && mm<=12 )
    {
        if(dd>0 && dd<=31)
        {
            if( hours<24 )
            {
                if( mins<60 )
                {
                    if( sec<60 )
                    {
                        if( week>0 && week<=7 )
                        {
                            dateCorrectFlag = SUCCESS;
                        }
                    }
                }
            }
        }
    }
    if(dateCorrectFlag == SUCCESS)
    {
        stimestructure.Hours = hours;
        stimestructure.Minutes = mins;
        stimestructure.Seconds = sec;
        stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
        if (HAL_RTC_SetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
        {
            printf("SetTimeError\r\n");
            return ERROR;
        }
        
        sdatestructure.WeekDay = week;
        sdatestructure.Month = mm;
        sdatestructure.Date = dd;
        sdatestructure.Year = yy;
        
        if (HAL_RTC_SetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN) != HAL_OK)
        {
            printf("SetDateError\r\n");
            return ERROR;
        }
    }
    else
    {
        return ERROR;
    }
    return SUCCESS;
}

/*
* @fn       getRTC_Date_Time
* @brief    ��ӡRTCʱ�䣬�����գ�ʱ���룬����
* @param   
* @return   
*/
void getRTC_Date_Time(void)
{
    /* Get the RTC current Time ,must get time first*/
    HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
    /* Get the RTC current Date */
    HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
    
    /* Display date Format : yy/mm/dd */
    printf("%02d/%02d/%02d\r\n",2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date); 
    /* Display time Format : hh:mm:ss */
    printf("%02d:%02d:%02d\r\n",stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
    
    printf("week:%02d\r\n",sdatestructure.WeekDay);
    printf("\r\n");
}


/*
* @fn       Time_synchronization
* @brief    ʱ��ͬ������
* @param   
* @return   
*/
void Time_synchronization(void)
{
    if(Time_synFlag == SET)
    {
        return;
    }
    else
    {
        //������ʱ���Լ����ͼ���
        Time_Syn_Retry_Num = 0;
        User_StartClock(&TimeSynclock);
        
//        Send_Time_synReq();  
    }
}

/*
* @fn       Time_syn_Cb
* @brief    ʱ��ͬ���ص�
* @param    
* @return   
*/
void Time_syn_Cb(void)
{
    //��ʱ�ط�
    if(Time_Syn_Retry_Num < TIME_SYN_RETRY_NUM_MAX-1)
    {
        Time_Syn_Retry_Num++;
        
        //Send_Time_synReq();
        User_StartClock(&TimeSynclock);
    }
    //�ط�������
    else
    {
        Time_Syn_Retry_Num = 0;
        return;
    }
}

/*
* @fn       Time_syn_RetryTimerInit
* @brief    ʱ��ͬ���ط���ʱ
* @param   
* @return   
*/
void Time_syn_RetryTimerInit(void)
{
    TimeSynclock.eventID = TIME_SYN_EVENT;
    TimeSynclock.timeOut = TIME_SYN_TIMEOUT;
    TimeSynclock.TaskHook = Time_syn_Cb;
}

/*
* @fn       Judge_Time_syn_avivilble
* @brief    �ж�ʱ���Ƿ���Ч����ͬ�����±���ʱ��
* @param   
* @return   
*/
void Judge_Time_syn_avivilble(uint8_t yy, uint8_t mm, uint8_t dd, uint8_t hours, uint8_t mins, uint8_t sec, uint8_t week)
{
    getRTC_Date_Time();
    uint8_t t=0xFF;
//#warning ���޸�0309    
    //��ǰʱ�䲻���죬��ʱ���С��3s
    if(dd == sdatestructure.Date)
    {
        if(mins == stimestructure.Minutes)
        {
            if( sec >= stimestructure.Seconds )
            {
                t = sec - stimestructure.Seconds;
            }
            else
            {
                t = stimestructure.Seconds - sec;
            }
            if(t<4)
            {
                if(setRTC_Date_Time(yy, mm, dd, hours, mins, sec, week))
                {
                    Time_synFlag = SET;
                    //����ͬ�������
                    User_StopClock(&TimeSynclock);
                    Time_Syn_Retry_Num = 0;
                }
            }
        }
    }
}


/*
* @fn       switchToToday
* @brief    0��ʱ���ʱ��ͬ��
            �жϲ��л�����һ��
* @param   
* @return   �����ǵڼ���/�ܼ�
*/
uint8_t switchToToday(uint8_t schedule_mode)
{
    getRTC_Date_Time();
    if(stimestructure.Hours == 0)
    {
        if(stimestructure.Minutes == 0)
        {
            if(stimestructure.Seconds >0 && stimestructure.Seconds<=20)
            {
                Time_synFlag = RESET;
                
                switch(schedule_mode)
                {
                case SCHEDULE_MODE_DAYCYCLE:
                    {
                        CurrentDayInTheLoop = (CurrentDayInTheLoop + 1)%Schedule_Data.schedulePeriod;
                        return CurrentDayInTheLoop;
                    }
                    break;
                    
                case SCHEDULE_MODE_WEEKCYCLE:
                    {
                        return sdatestructure.Date;
                    }
                    break;
                case SCHEDULE_MODE_WEEKCYCLEONETIME:
                    {
                        return sdatestructure.Date;
                    }
                    break;    
                    
                default:
                    break;
                }
            }
        }
    }
    return 0xFF;
}

/*
* @fn       getFollowedMode
* @brief    ���ݽ������ܼ�
            ��ѯ������´��ϱ����ڼ�ģʽ
* @param   
* @return   �´��ϱ������ں�ģʽ
*/
Schedule_Mode_t getFollowedMode(uint8_t week)
{
    uint8_t date;
    uint8_t start_h = 47;      //0~23���
    uint8_t end_h = 48;        //1~24��
    
    uint8_t start_l;           //�б��б��
    uint8_t end_l;             //�б��б��
    uint8_t i;
    uint8_t RTC_start_hour = 0xFF;    //��ǰ��RTCʱ���0~47 ��:1 ��ʾ����0:30 С��1:00 ��ʱ���
    uint8_t Schedule_startFlag = RESET;
    uint8_t Schedule_endFlag = RESET;
    
    date = week-1;
    Schedule_Mode_t Schedule_Mode;
    Schedule_Mode.cycle = DEFAULT_S_CYCLE;
    Schedule_Mode.mode =  POSITIONING_MODE;
        
    //getRTC_Date_Time();
    
    //��СʱΪ�洢��λ
    for(i=0; i<24; i++)
    {
        if(i == stimestructure.Hours)
        {
            if(stimestructure.Minutes>=30)
            {
                RTC_start_hour = i*2+1;
            }
            else
            {
                RTC_start_hour = i*2;
            }
            break;
        }
    }
    
    for(i=0; i<HOURS_GROUP_MAX; i++)
    {
        if(CalendarList[date][i].end_Time!=0)
        {
            //��ȡ����Ŀ�ʼʱ��
            if(CalendarList[date][i].start_Time > RTC_start_hour  && CalendarList[date][i].start_Time < start_h)
            {
                Schedule_startFlag = SET;
                start_h = CalendarList[date][i].start_Time;
                start_l = i;
                break;
            }
            
        }
    }
    for(i=0; i<HOURS_GROUP_MAX; i++)
    {
        if(CalendarList[date][i].end_Time!=0)
        {
            //��ȡ����Ľ���ʱ��
            if(CalendarList[date][i].end_Time > RTC_start_hour  && CalendarList[date][i].end_Time <= end_h)
            {
                Schedule_endFlag = SET;
                end_h = CalendarList[date][i].end_Time;
                end_l = i;
                break;
            }
        }
    }
    //if(start_h != end_h)
    //{
        uint16_t Tn;     //��ǰʱ��
        uint16_t Ts;     //schedule�б��д洢�������У�����������Ŀ�ʼʱ��
        Tn = stimestructure.Hours*60 + stimestructure.Minutes;
        
        
        if(Schedule_endFlag == SET)
        {
            if(RTC_start_hour == 47 || RTC_start_hour == 46)
            {
                uint16_t cycle;    //��ʱ��������õ�����
                cycle = BigtoLittle16(CalendarList[date][end_l].period);
                Ts = CalendarList[date][end_l].start_Time*30;
                
                Schedule_Mode.cycle = ((Tn - Ts)/cycle + 1)*cycle - (Tn - Ts);
                Schedule_Mode.mode = CalendarList[date][end_l].mode;
                Schedule_Mode.config_cycle = CalendarList[date][end_l].period;
            }
            else
            {
                //���������Ŀ�ʼʱ��� �´��ϱ�ʱ��Ϊ ��ʼʱ��-����ʱ��
                if(start_h < end_h && Schedule_startFlag == SET)
                {
                    Ts = start_h*30;
                    //ת��Ϊ����
                    Schedule_Mode.cycle = Ts - Tn;
                    Schedule_Mode.mode = CalendarList[date][start_l].mode;
                    Schedule_Mode.config_cycle = 0;//CalendarList[date][start_l].period;
                }
                else
                {
                    uint16_t cycle;    //��ʱ��������õ�����
                    cycle = BigtoLittle16(CalendarList[date][end_l].period);
                    Ts = CalendarList[date][end_l].start_Time*30;
                    
                    Schedule_Mode.cycle = ((Tn - Ts)/cycle + 1)*cycle - (Tn - Ts);
                    Schedule_Mode.mode = CalendarList[date][end_l].mode;
                    Schedule_Mode.config_cycle = CalendarList[date][end_l].period;
                }
            }
        }
        //��һ��Ϊ��scheduleģʽ --Tracking 24h
        else
        {
            Schedule_Mode.mode = POSITIONING_MODE;
            Schedule_Mode.cycle = DEFAULT_T_CYCLE;
            Schedule_Mode.config_cycle = 0;//BigtoLittle16(DEFAULT_T_CYCLE);
        }
    //}
    return Schedule_Mode;
}
//schedule�����ϱ�0xD3
/*
* @fn      Schedule_Data_Report()
* @brief   ��������ϱ�schedule����
* @param   
* @return   
*/
void Schedule_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp, Schedule_Data_upload_t Schedule_Data_upload, GPS_Info_t Schedule_GPS_Info)
{
    uint8_t report_to_ServerDataLen_D;
    uint8_t ServerDataLenSize;
    
    //Э��������䳤��
    uint8_t NB_Report_to_ServerData_Len;
    uint8_t Schedule_Data_upload_Len;
    uint8_t Schedule_GPS_Data_Len;
    
    uint16_t SumLen;
    //        NB_Report_to_ServerData_tmp.cmdlen = BigtoLittle16(NB_Report_to_ServerData_tmp.cmdlen);
    //        Locator_Data_tmp.FindnowCycle = BigtoLittle16(Locator_Data_tmp.FindnowCycle);
    //        Locator_Data_tmp.TrackerCycle = BigtoLittle16(Locator_Data_tmp.TrackerCycle);
    //        Locator_Data_tmp.GPS_data_len = BigtoLittle16(Locator_Data_tmp.GPS_data_len);
    
    
    Schedule_Data_upload.cycle = BigtoLittle16(Schedule_Data_upload.cycle);
    Schedule_Data_upload.cellID = Signal_Level.cellID;
    Schedule_Data_upload.ECL = Signal_Level.ECL;
    Schedule_Data_upload.RSPR = Signal_Level.RSPR;
    Schedule_Data_upload.RSRQ = Signal_Level.RSRQ;
    Schedule_Data_upload.SINR = Signal_Level.SINR;    
    
    NB_Report_to_ServerData_tmp.cmdlen = BigtoLittle16(NB_Report_to_ServerData_tmp.cmdlen);
    
    //�ֶγ���
    NB_Report_to_ServerData_Len = sizeof(NB_Report_to_ServerData_t);
    Schedule_Data_upload_Len = sizeof(Schedule_Data_upload_t);
    Schedule_GPS_Data_Len = sizeof(GPS_List_t);
    
    //�����ݳ���
    report_to_ServerDataLen_D = NB_Report_to_ServerData_Len + Schedule_Data_upload_Len + Schedule_Data_upload.GPS_data_len + 2;
    
    //��С��ת��
    Schedule_Data_upload.GPS_data_len = BigtoLittle16(Schedule_Data_upload.GPS_data_len);
    
    //���ATָ��֡ͷ
    uint8_t dataLen_t;
    uint8_t datadis;
    if(Schedule_Data_upload.GPS_List_num == 0)
    {
        dataLen_t = report_to_ServerDataLen_D - 2;
        datadis = 4;
    }
    else if(Schedule_Data_upload.GPS_List_num > 0)
    {
        dataLen_t = report_to_ServerDataLen_D - 1;
        datadis = 2;
    }
    ServerDataLenSize = Form_AT_Head(dataLen_t);
    SumLen = AT_TEST_PORT_LEN + ServerDataLenSize + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_TAIL_NUM;
    
    //���NB�ֶ��붨λ���ֶ�����
    uint16_t off_set_Len = 0;
    uint8_t at_head_len = 0;
    //NB��ͷ
    off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize;
    at_head_len = AT_TEST_PORT_LEN + ServerDataLenSize;
    
    hex2String((uint8_t*)&NB_Report_to_ServerData_tmp, NB_Report_to_ServerData_Len, (char*)at_cmd_data_p + at_head_len);
    
    off_set_Len = off_set_Len + NB_Report_to_ServerData_Len*2;
    //��λ����ͷ
    hex2String((uint8_t*)&Schedule_Data_upload, Schedule_Data_upload_Len, (char*)at_cmd_data_p + off_set_Len);
    
    off_set_Len = off_set_Len + Schedule_Data_upload_Len*2;
    
    //���У���
    uint8_t crc = 0;
    
    //GPS����
    for(uint8_t i=0; i<Schedule_Data_upload.GPS_List_num; i++)
    {
        hex2String((uint8_t*)&Schedule_GPS_Info.GPS_List[i], Schedule_GPS_Data_Len, (char*)at_cmd_data_p + off_set_Len);           
        off_set_Len = off_set_Len + Schedule_GPS_Data_Len*2;
        crc = crc + checkSum((uint8_t *)&Schedule_GPS_Info.GPS_List[i], Schedule_GPS_Data_Len);
    }
    
    //�����ȥ��ͷ֮�������
    crc = crc + checkSum((uint8_t *)&NB_Report_to_ServerData_tmp+1, NB_Report_to_ServerData_Len-1); //��ȥ0xFA
    crc = crc + checkSum((uint8_t *)&Schedule_Data_upload, Schedule_Data_upload_Len);
    
    hex2String( &crc, CHECK_SUM_LEN, (char*)at_cmd_data_p + off_set_Len);
    
    
    //��ӽ���λ
    uint8_t server_end;
    server_end = SERVER_START;
    hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2);
    off_set_Len = off_set_Len + 2;
    
    //���"\r\n"
    tool_memcpy((char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2, AT_END, AT_TAIL_NUM);
    
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen-datadis);
    //printf("SUM: %d\r\n", SumLen-datadis);
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen-datadis);
}

//schedule�����ϱ�0xD4,D5
/*
* @fn      Schedule_Device_Confirm_Report
* @brief   ��������ϱ��ն�ȷ������
* @param   
* @return   
*/
void Schedule_Device_Confirm_Report(uint8_t CMD_ID, uint8_t serviceCMD_Process_result, uint8_t packetSeq, uint8_t zoneTime, uint8_t TimeSynFlag)
{
    uint8_t report_to_ServerDataLen_D = 0;
    uint16_t SumLen = 0;
    uint8_t ServerDataLenSize;
    uint8_t crc;
    
    //����ն�ȷ������ �ֶ�����
    NB_Report_to_ServerData_t Device_confirm_Data = {0};
    Device_confirm_Data.start = SERVER_START;
    Device_confirm_Data.version = 0x01;
    Device_confirm_Data.softWare_version = 0x02;
    
    Device_confirm_Data.msg_seq_num = BigtoLittle16(msg_seq_num - 1);
    tool_memcpy(Device_confirm_Data.IMEI_num, NB_Report_to_ServerData.IMEI_num, IMEI_NUM_LEN); 
    
    if(CMD_ID == SCHEDULE_CONFIG_ID)
    {
        Device_confirm_Data.cmd = SCHEDULE_CONFIG_ID;
        report_to_ServerDataLen_D = sizeof(NB_Report_to_ServerData_t) + 3 + 1;
    }
    //0xD5
    else if(CMD_ID == TIME_SYN_REQ_ID)
    {
        Device_confirm_Data.cmd = TIME_SYN_REQ_ID;
        report_to_ServerDataLen_D = sizeof(NB_Report_to_ServerData_t) + 3 + 1;
    }
    
    Device_confirm_Data.cmdlen = BigtoLittle16(report_to_ServerDataLen_D);
    //���������ܳ�
    ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D-1);
    SumLen = AT_TEST_PORT_LEN + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_TAIL_NUM;
    
    //�������ִ�н���ֶ�����
    uint8_t off_set_Len = 0;  
    off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize;
    
    crc = checkSum((uint8_t*)&Device_confirm_Data, sizeof(NB_Report_to_ServerData_t));
    crc = crc - SERVER_START;
    
    
    hex2String((uint8_t*)&Device_confirm_Data, sizeof(NB_Report_to_ServerData_t), (char*)at_cmd_data_p+off_set_Len);
    off_set_Len = off_set_Len + sizeof(NB_Report_to_ServerData_t)*2;
    //0xD5
    if(CMD_ID == TIME_SYN_REQ_ID)
    {
        crc = crc + zoneTime;
        //������
        hex2String(&zoneTime, 1, (char*)at_cmd_data_p + off_set_Len);
        off_set_Len = off_set_Len + 2;
        
        //������
        hex2String(&TimeSynFlag, 1, (char*)at_cmd_data_p + off_set_Len);
        off_set_Len = off_set_Len + 2;
    }
    else if(CMD_ID == SCHEDULE_CONFIG_ID)
    {
        crc = crc + serviceCMD_Process_result + packetSeq;
        //�ְ����
        hex2String(&packetSeq, 1, (char*)at_cmd_data_p + off_set_Len);
        off_set_Len = off_set_Len + 2;
        //������
        hex2String(&serviceCMD_Process_result, 1, (char*)at_cmd_data_p + off_set_Len);
        off_set_Len = off_set_Len + 2;
    }
    
    //���У���
    hex2String(&crc, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + CHECK_SUM_LEN*2;
    
    //��ӽ���λ
    uint8_t server_end;
    server_end = SERVER_START;
    hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + 2;
    //���"\r\n"
    tool_memcpy(at_cmd_data_p + off_set_Len, AT_END, AT_TAIL_NUM);
    
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}



//schedule�����·�0xC2
/*
* @fn      Handel_ServerData_Schedule_Config
* @brief   ����������� Scheduleģʽ����
* @param   
* @return  �����Ƿ�ִ�� ���/�ɹ� 
*/
uint8_t Handel_ServerData_Schedule_Config(uint8_t* RxMessage, uint16_t dataLen)
{
    //����ѭ������
    Schedule_Data_t Schedule_Data;
    //���찲��
    Schedule_DayConfig_t Schedule_DayConfig;
    
    uint16_t off_set_Len = 0;
    uint8_t format_judge = ERROR;
    //��������Ƿ��Ѿ��������
    uint8_t packageFinishFlag = RESET;
    
    ascsToHexs((char*)RxMessage, (uint8_t*)&Schedule_CMD, sizeof(Schedule_CMD_t)*2);
    off_set_Len = off_set_Len + sizeof(Schedule_CMD_t)*2;
    
    Schedule_CMD.scheduleDataLen = BigtoLittle16(Schedule_CMD.scheduleDataLen);
    //����β��־λ
    if(Schedule_CMD.seqNum == 0)
    {
        packageFinishFlag = SET;
    }
    
    for(uint8_t i=0; i<Schedule_CMD.scheduleDataGroupNum; i++)
    {
        ascsToHexs((char*)RxMessage+off_set_Len, (uint8_t*)&Schedule_Data, sizeof(Schedule_Data_t)*2);
        off_set_Len = off_set_Len + sizeof(Schedule_Data_t)*2;
        for(uint8_t j=0; j<Schedule_Data.calendar_num; j++)
        {
            ascsToHexs((char*)RxMessage+off_set_Len, (uint8_t*)&Schedule_DayConfig, sizeof(Schedule_DayConfig_t)*2);
            off_set_Len = off_set_Len + sizeof(Schedule_DayConfig_t)*2;
            //��ʽ�ж�
            format_judge = ERROR;
            if(Schedule_DayConfig.startH < Schedule_DayConfig.endH)
            {
                if(Schedule_DayConfig.startH<=47)
                {
                     if(Schedule_DayConfig.endH>0 && Schedule_DayConfig.endH<=48)
                     {
                         if(Schedule_DayConfig.mode <= POSITIONING_MODE)
                         {
                             //if((Schedule_DayConfig.endH - Schedule_DayConfig.startH)*30 >= Schedule_DayConfig.cycle)
                             //{
                                format_judge = SUCCESS;
                             //}
                         }
                     }
                }
            }
            if(format_judge == ERROR)
            {
                return ERROR;
            }
            //���浱�찲��
            tool_memcpy(&CalendarList[Schedule_Data.schedulePeriod-1][j], &Schedule_DayConfig, sizeof(Schedule_DayConfig_t));
        }
    }
    if(packageFinishFlag == SET)
    {
        return SUCCESS;
    }
    else
    {
        return PACKAGE_NO_FINISH;
    }
}
//schedule�����·�0xC3 -- ͬ��ʱ���
/*
* @fn      Handel_ServerData_TimeSyn
* @brief   ���ݷ������·����ݸ���ʱ���
* @param   
* @return   
*/
uint8_t Handel_ServerData_TimeSyn(uint8_t* RxMessage, uint16_t dataLen)
{
    Time_syn_Data_t Time_syn_Data;
    //�����գ�ʱ����
    uint8_t yy, mm, dd, hour, min, sec, week; 
    
    ascsToHexs((char*)RxMessage, (uint8_t*)&Time_syn_Data, sizeof(Time_syn_Data_t)*2);
    
    yy = Time_syn_Data.Date_Value[2];
    mm = Time_syn_Data.Date_Value[1];
    dd = Time_syn_Data.Date_Value[0];
    
    hour = Time_syn_Data.Local_Time[2];
    min = Time_syn_Data.Local_Time[1];
    sec = Time_syn_Data.Local_Time[0];
    
    week = Time_syn_Data.week;
        
    if(SUCCESS != setRTC_Date_Time(yy, mm, dd, hour, min, sec, week))
    {
        return ERROR;
    }
    
    return SUCCESS;
}

/*--------------------------------------������-------------------------------------*/
/*
* @fn      TLocator_Data_Report()
* @brief   ��������ϱ���λ������
* @param   
* @return   
*/
void TLocator_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp, Locator_Data_t Locator_Data_tmp, GPS_Info_t GPS_Info_tmp, Signal_Level_t Signal_Level)
{
        uint8_t report_to_ServerDataLen_D;
        uint8_t ServerDataLenSize;
        uint8_t NB_Report_to_ServerData_Len;
        uint8_t Locator_Data_Len;
        uint8_t GPS_Data_Len;
        uint8_t Signal_Level_Len;
        uint16_t SumLen;
        
        NB_Report_to_ServerData_tmp.cmdlen = BigtoLittle16(NB_Report_to_ServerData_tmp.cmdlen);
        Locator_Data_tmp.FindnowCycle = BigtoLittle16(Locator_Data_tmp.FindnowCycle);
        Locator_Data_tmp.TrackerCycle = BigtoLittle16(Locator_Data_tmp.TrackerCycle);
        Locator_Data_tmp.GPS_data_len = BigtoLittle16(Locator_Data_tmp.GPS_data_len);    
               
        //�ֶγ���
        NB_Report_to_ServerData_Len = sizeof(NB_Report_to_ServerData_t);
        Locator_Data_Len = sizeof(Locator_Data_t);
        GPS_Data_Len = sizeof(GPS_List_t);
        Signal_Level_Len = sizeof(Signal_Level_t);
        
        //�����ݳ���
        report_to_ServerDataLen_D = NB_Report_to_ServerData_Len + Locator_Data_Len + (Locator_Data_tmp.GPS_List_num)*GPS_Data_Len + Signal_Level_Len + 1;
        
        
        //���ATָ��֡ͷ
        ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D);
        SumLen = AT_TEST_PORT_LEN + ServerDataLenSize + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_TAIL_NUM;
          
        //���NB�ֶ��붨λ���ֶ�����
        uint8_t off_set_Len = 0;
        uint8_t at_head_len = 0;
        //NB��ͷ
        off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize;
        at_head_len = AT_TEST_PORT_LEN + ServerDataLenSize;
        
        hex2String((uint8_t*)&NB_Report_to_ServerData_tmp, NB_Report_to_ServerData_Len, (char*)at_cmd_data_p + at_head_len);
        
        off_set_Len = off_set_Len + NB_Report_to_ServerData_Len*2;
        
        //�ź�ǿ�Ȱ�ͷ
        hex2String((uint8_t*)&Signal_Level, Signal_Level_Len, (char*)at_cmd_data_p + off_set_Len);
        off_set_Len = off_set_Len + Signal_Level_Len*2;
        
        //��λ����ͷ
        hex2String((uint8_t*)&Locator_Data_tmp, Locator_Data_Len, (char*)at_cmd_data_p + off_set_Len);
        off_set_Len = off_set_Len + Locator_Data_Len*2;
               
        //���У���
        uint8_t crc = 0;
        
        //GPS����
        for(uint8_t i=0; i<Locator_Data_tmp.GPS_List_num; i++)
        {
            hex2String((uint8_t*)&GPS_Info_tmp.GPS_List[i], GPS_Data_Len, (char*)at_cmd_data_p + off_set_Len);           
            off_set_Len = off_set_Len + GPS_Data_Len*2;
            crc = crc + checkSum((uint8_t *)&GPS_Info_tmp.GPS_List[i], GPS_Data_Len);
        }

        //�����ȥ��ͷ֮�������
        crc = crc + checkSum((uint8_t *)&NB_Report_to_ServerData_tmp+1, NB_Report_to_ServerData_Len-1); 
        crc = crc + checkSum((uint8_t *)&Locator_Data_tmp, Locator_Data_Len);
        crc = crc + checkSum((uint8_t *)&Signal_Level, Signal_Level_Len);
        
        hex2String( &crc, CHECK_SUM_LEN, (char*)at_cmd_data_p + off_set_Len);        
        
        
        //��ӽ���λ
        uint8_t server_end;
        server_end = SERVER_START;
        hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2);
        off_set_Len = off_set_Len + 2;
        
        //���"\r\n"
        tool_memcpy((char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2, AT_END, AT_TAIL_NUM);
        
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}









