/*******************************************************************************
Copyright:       Lierda WSN BT team
Filename:        nb.c
Description:     nb_Driver
FileVersion:     V1.0

ChangeLog: 

=========================================
date:17/8/24
author:chenkl
log: Create File

*******************************************************************************/


/*******************************************************************************
* INCLUDES
*/
//#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_it.h"
#include "User_ClockDriver.h"
#include "usart.h"
#include "tools.h"
#include "nb.h"
#include "schedule.h"
#include "main.h"
#include "flash.h"
#include "UNIX.h"
#include "FOTA.h"
#include "crc16.h"
/*********************************************************************
* TYPEDEFS
*/
/*********************************************************************
* MACROS
*/
/*********************************************************************
* PUBLIC VARIABLES 
*/
char NB_version[30];                        //�汾��
uint8_t NB_version_len = 0;

//char* at_cmd_data_p = NULL;                 //ATָ��

//uint8_t at_cmd_data_p[512];                 //ATָ��

uint8_t *nbRxMessage = NULL;                //NB��������

Device_rsp_t Device_rsp;                    //����ϱ�Ӧ���е�����
Device_confirm_seq_t Device_confirm_seq;    //�ն�ȷ������0xD2D2
uint16_t Device_confirm_seq_len = 0;        //����ִ�н������

Locator_pram_t Locator_pram = {0};          //��λ������

//��λ�������ϱ���ʽ
Locator_Data_t Locator_Data = {0};

//ģʽ--����Ĭ��ע��ģʽ
uint8_t currentMode = REGIST_MODE;

//Sleep
uint8_t SleepStatus = AWAKE;
uint8_t LED_Init_Blink_Finish = RESET;

//���ϻ�վ��־λ
uint8_t NB_Station_Connected_Flag = RESET;
//�Ƿ�ע��ɹ�
uint8_t RegistFlag = RESET;

//��������ע��
//uint8_t RegistConfirmFlag = RESET;

uint8_t DataUpFlag = RESET;
uint8_t data_confirmFlag = RESET;
uint8_t after_registFlag = RESET;
uint8_t RegistStateFlag = RESET;
//���ݷ�����ɱ�־
uint8_t NB_min_counter = 0;
uint8_t NB_StateSend_counter = 0;
uint8_t NB_Confirm_counter = 0;
//NB�ϱ�������������
NB_Report_to_ServerData_t NB_Report_to_ServerData = {0};

//Ĭ��ATָ��
uint8_t curAT_RSP_ID = AT_CFUN0/*AT_CFUN0*/;

//����ִ�н��
uint8_t serviceCMD_Process_result = ERROR; 

//���к�
uint16_t msg_seq_num = 0;

//��ʱ������ֵ
uint32_t wakeupCounter = 0;

//��λ�������л���С��2Сʱ����־λ
uint8_t TrackerCycle_short_Flag = RESET;
//findNow���ϱ�λ������
//uint8_t special_findnowFlag = RESET;

//GPS\NB������־
uint8_t NB_open_Flag = RESET;
uint8_t gps_open_Flag = RESET;

//������־
uint8_t NB_InNetWork_Flag = RESET;

//һ���յ��������ݰ�
uint8_t CMD_packageNum = 0;
//�Ƿ��հ����
uint8_t packageFinishFlag = RESET;
//�Ƿ�ʼ�հ�
uint8_t packageStartFlag = RESET;
//C2����ִ�н��
uint8_t C2_command_FailFlag = RESET;

//�������ݱ�־
uint8_t TxCompleteFlag = RESET;

//�Ƿ��յ���ȷ�Ļظ�
uint8_t Retry_state = ERROR;
//�ط�����
uint8_t AT_Retry_Num = 0;

//����
uint16_t dataCycle = 0;

uint8_t ExtraDataupFlag = RESET;

//�ϱ���ɱ�־λ
//uint8_t upload_OK_Flag = RESET;
//���������־λ
uint8_t cmd_OK_Flag = RESET;
//��ǰ״̬֡
uint8_t state_OK_Flag = RESET;
uint8_t Normal_StateFlag = RESET;
uint8_t RegistFinishFlag = RESET;

uint8_t OTAingFlag = RESET;
uint8_t move_alarm_enable_Flag = RESET;
uint8_t move_alarm_Flag = RESET;
uint8_t move_alarm_Init_Flag = RESET;

uint8_t MoveAlarmPreMode;
//������
uint8_t testFlag = RESET;
uint8_t testFlag2 = RESET;
uint8_t testFlag3 = RESET;

uint8_t FOTA_Cmplt_Flag = 0;

uint8_t moveFlagCounter = 0;

char procotolData[COAP_PROTOCOL_LEN] = "FAB10301020304050607080910111213141564FF1111222201333311223344000000000000";
//coap��������
char coapEnterSleep[32] = "AT+MLWULDATAEX=3,010203,0x0001\r\n";

char* str2 = NULL;

Clock_Struct NBReportclock;
Clock_Struct GPSCollectionclock;
Clock_Struct BoardCloseTimeoutclock;
Clock_Struct LEDclock;
Clock_Struct DataRspclock;
Clock_Struct InNetTimeoutclock;
Clock_Struct DeviceRegistclock;
Clock_Struct ATTimeoutclock;
Clock_Struct GPSTimeOutclock;
Clock_Struct APPUnbindclock;
//�����ط�
Clock_Struct DataRptclock;
Coap_Haed_t Coap_Haed;


void Handel_ServerAT_CMD(uint8_t* RxMessage);
void Handel_ServerData(uint8_t* RxMessage, uint16_t dataLen);
uint8_t Form_AT_Head(uint16_t datalen);
uint8_t Handel_ServerData_Info_RSP(uint8_t* RxMessagen, uint16_t dataLen);
//Device_rsp_t Handel_Server_ctl_cmd( Server_ctl_cmd_t Server_ctl_cmd, uint8_t *Server_ctl_data);
void Locator_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData, Locator_Data_t Locator_Data, GPS_Info_t GPS_Info);
void Device_Confirm_Report(uint8_t serviceCMD_Process_result);
void NBReportclockProcess(void);
void GPSCollectionclockProcess(void);
void ScheduleParmSet(uint8_t CurrentDayInTheLoop);
void Handel_UNIX_Data(uint8_t* RxMessage);
void Coap_EnterSleep(void);
void NB_Recv_Data_Process(uint8_t *buf, uint16_t len);
/*********************************************************************
* User Timer API
*/



/*
* @fn      Open_NB
* @brief   IO�ڿ���NB����
* @param   
* @return   
*/
void Open_NB(void)
{
    HAL_GPIO_WritePin(NB_SW_GPIO_Port, NB_SW_Pin, GPIO_PIN_SET);
}

void Close_NB(void)
{
    //HAL_GPIO_WritePin(NB_SW_GPIO_Port, NB_SW_Pin, GPIO_PIN_RESET);
}


/*
* @fn      NB_AT_CMD_Send
* @brief   ��NBģ�鷢��ATָ��  
* @param   ָ��ID,����...
* @return   
*/
void NB_AT_CMD_Send( uint8_t cmdID_t, ...)
{
    uint8_t cmdID;
    cmdID = cmdID_t;
    //AT+����Э��ջ
    if(cmdID == AT_CFUN0 || cmdID == AT_CFUN1)
    {
        //��Ҫ�ظ�"OK"
        uint8_t SumLen = 0;
        
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        //�ر�Э��ջ
        if(cmdID == AT_CFUN0)
        {
            //AT+CFUN=0\r\n
            tool_strcat(AT_ONE_PRAM_LEN, AT_START, CFUN, AT_PRAM_0, AT_END);   
        }
        //����Э��ջ
        else if(cmdID == AT_CFUN1)
        {
            //AT+CFUN=1\r\n
            tool_strcat(AT_ONE_PRAM_LEN, AT_START, CFUN, AT_PRAM_1, AT_END);
        }
        else
        {
            //error
            return;
        }
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //����IOTƽ̨��ַ
    else if(cmdID == AT_NCDP)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_IOT_IP_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, NCDP, AT_IOT_IP, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
    }    
    //��ѯIMEI
    else if(cmdID == AT_CGSN)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CGSN, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //������
    else if(cmdID == AT_NRB)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_3;
        
        tool_strcat(AT_NO_PRAM_LEN, AT_START, NRB, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //����������ʾ
    else if(cmdID == AT_CMEE)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CMEE, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //����APN
    else if(cmdID == AT_CGDCONT)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_7 + AT_CMDID_PRAM_LEN + AT_CGDCONT_CTNET_LEN;
        
        tool_strcat(AT_TWO_PRAM_LEN, AT_START, CGDCONT, AT_PRAM_1, AT_CGDCONT_CTNET, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //������������֪ͨ
    else if(cmdID == AT_NNMI)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, NNMI, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //�Զ�����
    else if(cmdID == AT_CGATT)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_5 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CGATT, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //���û�վ����֪ͨ
    else if(cmdID == AT_CSCON)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_5 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CSCON, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //��ѯ�����������IP��ַ
    else if(cmdID == AT_CGPADDR)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_7;
        
        tool_strcat(AT_NO_PRAM_LEN, AT_START, CGPADDR, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //��ѯ����״̬
    else if(cmdID == AT_NUESTATS)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_8;
        
        tool_strcat(AT_NO_PRAM_LEN, AT_START, NUESTATS, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //����SOCKET (�˿�xxxx)
    else if(cmdID == AT_NSOCR)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_5 + AT_PRAM_SOCKET_LEN + AT_PRAM_POART_LEN;
        
        tool_strcat(AT_TWO_PRAM_LEN, AT_START, NSOCR, AT_PRAM_SOCKET, AT_PRAM_POART, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    } 
    //AT+���������������
    else if(cmdID == AT_NSOST)
    {
        //        int ReportID;
        //        va_list arg_ptr;
        //        va_start(arg_ptr, cmdID_t);
        //        ReportID = va_arg(arg_ptr, int);
        //        
        //        //�����ϱ�0xD1
        //        if(ReportID == LOCATOR_DATA_REPORT)
        //        {
        //            //NB�������ϱ���ʽ
        //            NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp;
        //            //��λ�������ϱ���ʽ
        //            Locator_Data_t Locator_Data_tmp;
        //            //GPS���ݸ�ʽ
        //            GPS_Info_t GPS_Info_tmp;
        //            
        //            //��ȡ����
        //            NB_Report_to_ServerData_tmp = va_arg(arg_ptr, NB_Report_to_ServerData_t);
        //            Locator_Data_tmp = va_arg(arg_ptr, Locator_Data_t);
        //            GPS_Info_tmp = va_arg(arg_ptr, GPS_Info_t);
        //            va_end(arg_ptr);
        //            //������֡ �뷢��
        //#warning ����180326              
        //            //Locator_Data_Report(NB_Report_to_ServerData_tmp, Locator_Data_tmp, GPS_Info_tmp);
        //            TLocator_Data_Report(NB_Report_to_ServerData_tmp, Locator_Data_tmp, GPS_Info_tmp, Signal_Level);
        //        }
        //        //����ȷ��0xD2
        //        else if(ReportID == LOCATOR_CMD_CONFIRM)
        //        {
        //            va_end(arg_ptr);
        //            Device_Confirm_Report(serviceCMD_Process_result);
        //        }
        //        else
        //        {
        //            va_end(arg_ptr);
        //        }
        int ReportID;
        va_list arg_ptr;
        va_start(arg_ptr, cmdID_t);
        ReportID = va_arg(arg_ptr, int);
        
        //�����ϱ�0xD1
        if(ReportID == LOCATOR_DATA_REPORT)
        {
            //NB�������ϱ���ʽ
            NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp;
            //��λ�������ϱ���ʽ
            Locator_Data_t Locator_Data_tmp;
            //GPS���ݸ�ʽ
            GPS_Info_t GPS_Info_tmp;
            
            //��ȡ����
            NB_Report_to_ServerData_tmp = va_arg(arg_ptr, NB_Report_to_ServerData_t);
            Locator_Data_tmp = va_arg(arg_ptr, Locator_Data_t);
            GPS_Info_tmp = va_arg(arg_ptr, GPS_Info_t);
            va_end(arg_ptr);
            //������֡ �뷢��
#warning ����180326              
            TLocator_Data_Report(NB_Report_to_ServerData_tmp, Locator_Data_tmp, GPS_Info_tmp, Signal_Level);
        }
        //0xD2
//        else if(ReportID == LOCATOR_CMD_CONFIRM)
//        {
//            NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp;
//            //��ȡ����
//            NB_Report_to_ServerData_tmp = va_arg(arg_ptr, NB_Report_to_ServerData_t);
//            va_end(arg_ptr);
//            Locator_Data_ReportSleep( NB_Report_to_ServerData_tmp);
//        }
        else
        {
            va_end(arg_ptr);
        }
    }    
    //AT+����������ͻ�ȡ������������
    else if(cmdID == AT_NSORF)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_NSORF_LEN;
        tool_strcat(AT_NSOST_PRAM_LEN, AT_START, NSORF, AT_PRAM_0, AT_NSORF_NUM, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
    }
    else if(cmdID == AT_CGMR)
    {
        //��Ҫ�ȴ��ظ� ���� +"OK" 
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4;
        tool_strcat(AT_NO_PRAM_LEN, AT_START, CGMR, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //else if(cmdID == AT_MLWULDATAEX)
    //{
    //}
}

/*
* @fn      Locator_Data_Report()
* @brief   ��������ϱ���λ������
* @param   
* @return   
*/
void Locator_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp, Locator_Data_t Locator_Data_tmp, GPS_Info_t GPS_Info_tmp)
{
    uint8_t report_to_ServerDataLen_D;
    uint8_t ServerDataLenSize;
    uint8_t NB_Report_to_ServerData_Len;
    uint8_t Locator_Data_Len;
    uint8_t GPS_Data_Len;
    uint16_t SumLen;
    
    NB_Report_to_ServerData_tmp.cmdlen = BigtoLittle16(NB_Report_to_ServerData_tmp.cmdlen);
    Locator_Data_tmp.FindnowCycle = BigtoLittle16(Locator_Data_tmp.FindnowCycle);
    Locator_Data_tmp.TrackerCycle = BigtoLittle16(Locator_Data_tmp.TrackerCycle);
    Locator_Data_tmp.GPS_data_len = BigtoLittle16(Locator_Data_tmp.GPS_data_len);    
    
    //�ֶγ���
    NB_Report_to_ServerData_Len = sizeof(NB_Report_to_ServerData_t);
    Locator_Data_Len = sizeof(Locator_Data_t);
    GPS_Data_Len = sizeof(GPS_List_t);
    
    //�����ݳ���
    report_to_ServerDataLen_D = NB_Report_to_ServerData_Len + Locator_Data_Len + (Locator_Data.GPS_List_num)*GPS_Data_Len + 1;
    
    //���ATָ��֡ͷ
    ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D);
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
    hex2String((uint8_t*)&Locator_Data_tmp, Locator_Data_Len, (char*)at_cmd_data_p + off_set_Len);
    
    off_set_Len = off_set_Len + Locator_Data_Len*2;
    
    //���У���
    uint8_t crc = 0;
    
    //GPS����
    for(uint8_t i=0; i<Locator_Data.GPS_List_num; i++)
    {
        hex2String((uint8_t*)&GPS_Info_tmp.GPS_List[i], GPS_Data_Len, (char*)at_cmd_data_p + off_set_Len);           
        off_set_Len = off_set_Len + GPS_Data_Len*2;
        crc = crc + checkSum((uint8_t *)&GPS_Info_tmp.GPS_List[i], GPS_Data_Len);
    }
    
    //�����ȥ��ͷ֮�������
    crc = crc + checkSum((uint8_t *)&NB_Report_to_ServerData_tmp+1, NB_Report_to_ServerData_Len-1); 
    crc = crc + checkSum((uint8_t *)&Locator_Data_tmp, Locator_Data_Len);
    
    hex2String( &crc, CHECK_SUM_LEN, (char*)at_cmd_data_p + off_set_Len);        
    
    
    //��ӽ���λ
    uint8_t server_end;
    server_end = SERVER_START;
    hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2);
    off_set_Len = off_set_Len + 2;
    
    //���"\r\n"
    tool_memcpy((char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2, AT_END, AT_TAIL_NUM);
    
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}


/*
* @fn      Device_Confirm_Report
* @brief   ��������ϱ��ն�ȷ������D2
* @param   
* @return   
*/
void Device_Confirm_Report(uint8_t serviceCMD_Process_result)
{
    uint8_t report_to_ServerDataLen_D = 0;
    uint16_t SumLen = 0;
    uint8_t ServerDataLenSize;
    uint8_t crc;
    
    report_to_ServerDataLen_D = sizeof(NB_Report_to_ServerData_t) + 3 + COAP_PROTOCOL_HALFLEN;
    
    //����ն�ȷ������ �ֶ�����
    NB_Report_to_ServerData_t Device_confirm_Data = {0};
    Device_confirm_Data.start = SERVER_START;
    Device_confirm_Data.cmd = SERVER_ID_CMD_RSP;
    Device_confirm_Data.version = CMD_VERSION;
    Device_confirm_Data.softWare_version = SW_VERSION;
    Device_confirm_Data.cmdlen = BigtoLittle16(report_to_ServerDataLen_D - COAP_PROTOCOL_HALFLEN);
    Device_confirm_Data.msg_seq_num = BigtoLittle16(msg_seq_num);
    
    tool_memcpy(Device_confirm_Data.IMEI_num, NB_Report_to_ServerData.IMEI_num, IMEI_NUM_LEN);    
    //���������ܳ�
    ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D-1);
    SumLen = AT_TEST_PORT_LEN + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_COAP_TAIL_NUM;
    
    //�������ִ�н���ֶ�����
    uint8_t off_set_Len = 0;  
    off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize + COAP_PROTOCOL_LEN;
    
    crc = checkSum((uint8_t*)&Device_confirm_Data, sizeof(NB_Report_to_ServerData_t));
    crc = crc - SERVER_START;
    crc = crc + serviceCMD_Process_result;
    
    hex2String((uint8_t*)&Device_confirm_Data, sizeof(NB_Report_to_ServerData_t), (char*)at_cmd_data_p+off_set_Len);
    off_set_Len = off_set_Len + sizeof(NB_Report_to_ServerData_t)*2;
    //������
    hex2String(&serviceCMD_Process_result, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + 2;
    //���У���
    hex2String(&crc, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + CHECK_SUM_LEN*2;
    
    //��ӽ���λ
    uint8_t server_end;
    server_end = SERVER_START;
    hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + 2;
    //���"\r\n"
    tool_memcpy(at_cmd_data_p + off_set_Len, AT_COAP_END, AT_COAP_TAIL_NUM);
    
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen+1);
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}


/*
* @fn      Form_AT_Head(uin8_t datalen)
* @brief   ��� at_cmd_data_p �ϱ�ATָ��֡ͷ
* @param   
* @return  λ��  
*/
uint8_t Form_AT_Head(uint16_t datalen)
{
    //uint8_t SumLen = 0;
    uint8_t ServerDataLenSize = 0;
    //uint8_t datalen_t = 0;
    
    //char* at_cmd_data_head = NULL;
    //datalen_t = datalen * 2;
    //�����ݳ���
    
    if(datalen > 100)
    {
        //��λ��+","
        ServerDataLenSize = 4;
    }
    else if(datalen > 10)
    {
        //��λ��+","
        ServerDataLenSize = 3;    
    }
    else if(datalen > 0)
    {
        //һλ��+","
        ServerDataLenSize = 2;  
    }
    //SumLen = AT_TEST_PORT_LEN + ServerDataLenSize + datalen_t + CHECK_SUM_LEN*2 + AT_TAIL_NUM;
    //at_cmd_data_p = malloc(SumLen);
    
    tool_strcat(AT_HEARER_NUM, AT_START, MLWULDATAEX);
    //tool_strcat(AT_HEARER_NUM, AT_START, NSOST, AT_PRAM_0, AT_PRAM_IP, AT_PRAM_PORT);
    
    //tool_memcpy(at_cmd_data_p, at_cmd_data_head, AT_TEST_PORT_LEN);
    //free(at_cmd_data_head);
    
    //�� AT+NSOST=0,218.244.145.31,8000,120,....\r\n
    //�� AT+MLWULDATAEX=41,FAB10338363533353230333030303833383664FF1111222201333311223344000000000003FFAA013E,0x0100\r\n
    datalen = datalen + CHECK_SUM_LEN;
    if(datalen > 100)
    {
        at_cmd_data_p[AT_TEST_PORT_LEN] = datalen/100 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+1] = datalen%100/10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+2] = datalen%100%10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+3] = ',';
        //tool_memcpy(at_cmd_data_p+AT_TEST_PORT_LEN+4, procotolData,COAP_PROTOCOL_LEN);
    }
    else
    {
        at_cmd_data_p[AT_TEST_PORT_LEN] = datalen%100/10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+1] = datalen%100%10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+2] = ',';
        //tool_memcpy(at_cmd_data_p+AT_TEST_PORT_LEN+3, procotolData,COAP_PROTOCOL_LEN);
    }    
    return ServerDataLenSize;    
}

/*
* @fn      Handel_ServerAT_CMD()
* @brief   �����������·�ATָ���ʽ  
* @param   
* @return   
*/
void Handel_ServerAT_CMD(uint8_t* Message)
{
    uint8_t data_pos = 0;//��������λ��
    uint8_t data_start_Flag = RESET;
    uint16_t nb_server_data_len = 0;
    //����0,218.244.145.31,8000,10,0123456789,0
    uint8_t* RxMessage;
    uint16_t rxdatalen = 0;
    RxMessage = Message;
    if(*RxMessage == '0')
    {
        data_start_Flag = SET;
    }
    else
    {
        return;
    }
    if(data_start_Flag == SET)
    {
        data_pos = NB_ID;
        while((*RxMessage != '\r') || (rxdatalen < USER_BUFF_SIZE))
        {
            if(*RxMessage == ',')
            {
                data_pos++;
                //ip��ַ��
                if(data_pos == NB_IP)
                {
                    
                }
                //�˿ںŶ�
                else if(data_pos == NB_PORT)
                {
                    
                }
                //�������·����ݳ��ȶ�
                else if(data_pos == NB_SERVER_DATA_LEN)
                {
                    uint8_t* RxMessageLen_p = NULL;
                    uint8_t Len;
                    char* str;
                    
                    RxMessageLen_p = RxMessage+1;
                    //��ȡ ���ݳ��ȶ� ����
                    str = strtok((char*)RxMessageLen_p, ",");
                    //��ԭ�����ǵ���','��
                    Len = strlen(str);
                    *(RxMessageLen_p + Len) = ',';
                    //ת��Ϊ��ֵ
                    nb_server_data_len = atoi(str);
                }
                //�������ݶ�
                else if(data_pos == NB_SERVER_DATA)
                {
                    Handel_ServerData(RxMessage, nb_server_data_len);
                    return;
                    //�ϱ�Ӧ���
                }
                //�Ƿ����л����ֶ�
                else if(data_pos == NB_DATA_CACHE_FLAG)
                {
                    if(*RxMessage > 0)
                    {
                        //�����ֶ�δ���������� ��ȡ�� ATָ��, ����ĿǰЭ�飨94�ֽڣ������ᳬ
                        //����ָ��ƴ��
                    }
                }
            }
            RxMessage++;
            rxdatalen++;
        }
    }   
}

/*
* @fn      Handel_ServerData()
* @brief   �������������  ���� ������-->�ն�
* @param   
* @return   
*/
void Handel_ServerData(uint8_t* RxMessage, uint16_t dataLen)
{
    //�ж�У���
    //У��� -- �������ֵ�У��λ֮ǰ�������ֽ��ۼӺ�ȡ��λ�ֽڡ�
    
    uint8_t crcData = 0;
    uint8_t RxcrcData = 0;
    
    if(dataLen<5)
    {
        return;
    }
    RxcrcData = charsToHex((char*)RxMessage+(dataLen-2)*2);
    crcData = checkSumChar((char*)RxMessage+2, (dataLen-4)*2);
    
    if(crcData != RxcrcData)
    {
        printf("CS1:%d\r\n", crcData);
        printf("CS2:%d\r\n", RxcrcData);
        return;
    }
    
    ServerData_t Server_CMD_Data;
    uint8_t Server_CMD_Data_Len;
    Server_CMD_Data_Len = sizeof(ServerData_t);
    
    ascsToHexs((char*)RxMessage, (uint8_t*)&Server_CMD_Data, Server_CMD_Data_Len*2);
    //��ʼλFA
    if(Server_CMD_Data.start != SERVER_START)
    {
        //printf("DATA ERROR!");
        return;
    }
    Server_CMD_Data.packetSeqNum = BigtoLittle16(Server_CMD_Data.packetSeqNum);
    msg_seq_num = Server_CMD_Data.packetSeqNum;
    NB_Report_to_ServerData.msg_seq_num = BigtoLittle16(msg_seq_num);
    
//    //�ϱ�Ӧ��C0
//    if(Server_CMD_Data.cmd == SERVER_ID_DATA_RSP)
//    {
//        printf("C0\r\n");
//        
//        //���ճɹ����++
//        msg_seq_num++;
//        NB_Report_to_ServerData.msg_seq_num = BigtoLittle16(msg_seq_num);
//        
//        //������
//        if(RegistFlag == RESET)
//        {
//            NB_StateSend_counter = 0;
//            if(data_confirmFlag == SET)
//            {
//                if(Schedule_State == SET)
//                {
//                    Schedule_State = RESET;
//                    if(Schedule_StateStartFlag == RESET)
//                    {
//                        Schedule_StateStartFlag = SET;
//                            
//                        curAT_RSP_ID = AT_DATA_RSP;
//                        
//                        //cycleStartFlag = SET;
//                        //ScheduleModeProcess();
//                        ScheduleParmSet(CurrentDayInTheLoop);
//                        
//                        NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
//                        NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
//                        Schedule_Data_upload.GPS_List_num = 0;
//                        Schedule_Data_upload.GPS_data_len = 1;  
//                        Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
//                        //printf("Schedule_DATA_UP: %d", NB_Report_to_ServerData.cmdlen);
//                        
//                        Schedule_StateSend_counter = 0;
//                        DataRspclock.timeOut = 5000;
//                        User_StartClock(&DataRspclock);
//                    }
//                }
//                else
//                {
//                    Schedule_StateStartFlag = RESET;
//                    data_confirmFlag = RESET;
//                    RegistFlag = SET;
//                    DataUpFlag = SET;
//                    Locator_Data.initFlag = SET;
//                    
//                    TrackerCycle_short_Flag = SET;
//                    //after_registFlag = SET;
//                    printf("registSuccess\r\n");
//                    //testFlag = SET;
//                    curAT_RSP_ID = AT_INIT_SUCCESS;
//                    
//                    User_StopClock(&DataRspclock);
//                    User_StopClock(&BoardCloseTimeoutclock);
//                    User_StopClock(&ATTimeoutclock);
//                    User_StopClock(&InNetTimeoutclock);
//                    //������ɺ����GPS��Ϣ
//                    if(currentMode == POSITIONING_MODE)
//                    {
//                        GPS_Info_Delete();
//                        GPS_Info_schedule.stores_num = 0;
//                    }
//                    NB_min_counter = 0;
//                    Close_NB();
//                    //printf("Sleep4");
//                    //wakeupCounter = 4320;
//                    wakeupCounter = 0;
//                    SleepStatus = SLEEP;
//                }
//            }
//            //ע��ʧ��
//            if(C2_command_FailFlag == SET)
//            {
//                NVIC_SystemReset();
//            }
//        }
//        else if(RegistFlag == SET)
//        {
//            if(Schedule_State == SET)
//            {
//                Schedule_State = RESET;
//                if(Schedule_StateStartFlag == RESET)
//                {
//                    Schedule_StateStartFlag = SET;
//                    curAT_RSP_ID = AT_DATA_RSP;
//                    
//                    //cycleStartFlag = SET;
//                    //ScheduleModeProcess();
//                    ScheduleParmSet(CurrentDayInTheLoop);
//                    
//                    NB_Report_to_ServerData.cmd = SCHEDULE_UPLOAD_ID;
//                    NB_Report_to_ServerData.cmdlen = sizeof(NB_Report_to_ServerData_t)+sizeof(Schedule_Data_upload_t)+ Schedule_Data_upload.GPS_List_num * sizeof(GPS_List_t) + CHECK_SUM_LEN + END_NUM_LEN;
//                    Schedule_Data_upload.GPS_List_num = 0;
//                    Schedule_Data_upload.GPS_data_len = 1;
//                    Schedule_Data_Report(NB_Report_to_ServerData, Schedule_Data_upload, GPS_Info_schedule);
//                    printf("Schedule_DATA_UP3: %d", NB_Report_to_ServerData.cmdlen);
//                    
//                    Schedule_StateSend_counter = 0;
//                    DataRspclock.timeOut = 5000;
//                    User_StartClock(&DataRspclock);
//                }
//            }
//            else
//            {
//                if(Locator_pram.enterSleep == SPECIAL_FINDNOW)
//                {
//                    Locator_pram.enterSleep = RESET;
//                    
//                    Schedule_StateStartFlag = RESET;
//                    User_StopClock(&DataRspclock);
//                    User_StopClock(&BoardCloseTimeoutclock);
//                    
//                    printf("SpecialFN\r\n");
//                    Device_Open_GPS();
//                    //special_findnowFlag = SET;
//                    DataUpFlag = RESET;
//                    NB_InNetWork_Flag = RESET;
//                }
//                else
//                {
//                    Schedule_StateStartFlag = RESET;
//                    User_StopClock(&ATTimeoutclock);
//                    User_StopClock(&InNetTimeoutclock);
//                    User_StopClock(&DataRptclock);
//                    User_StopClock(&DataRspclock);
//                    User_StopClock(&BoardCloseTimeoutclock);
//                    //������ɺ����GPS��Ϣ
//                    //if(currentMode == POSITIONING_MODE)
//                    //{
//                        GPS_Info_Delete();
//                    //}
//                    NB_min_counter = 0;
//                    NB_open_Flag = RESET;
//                    Close_NB();
//                    
//                    wakeupCounter = 0;
//                    SleepStatus = SLEEP;
//                    printf("Sleep0\r\n");
//                }
//            }
//        }
//    }
    //�·�����c1
    if(Server_CMD_Data.cmd == SERVER_ID_CMD_DATA)
    {
        printf("C1\r\n");
        //��ȡ���������      
        if(dataLen > 0)
        {
            if(RegistFlag == RESET)
            {        
                User_StopClock(&DeviceRegistclock);
            }
            serviceCMD_Process_result = Handel_ServerData_Info_RSP( RxMessage+(Server_CMD_Data_Len)*2, sizeof(Locator_pram_t)); 
            
            //����Ϊ���ģʽʱ �˳�schedule
            if(serviceCMD_Process_result == 0)
            {
                if(Locator_pram.newMode != SCHEDULE_MODE) 
                { 
                    printf("ModeChange\r\n");
                    Schedule_ModeFlag = RESET;
                    //Time_synFlag = RESET;
                    //Time_synInitFlag = RESET;
                }
            }
//�ϱ�һ֡��ǰ״̬������
            if(Locator_pram.newMode != SCHEDULE_MODE)
            {
                Normal_StateFlag = SET;
            }
            else
            {
                Schedule_StateStartFlag = SET;
            }
            DataRspclock.timeOut = 3000;
            User_StartClock(&DataRspclock);
        }
    }
#warning ���scheduleģʽע��coap�Ĵ�����������������    
    //�·�����-- ����Scheduleģʽ����C2
    else if(Server_CMD_Data.cmd == SERVER_ID_SCHEDULE_CONFIG)
    {
        printf("C2\r\n");
        if(RegistFlag == RESET)
        {        
            User_StopClock(&DeviceRegistclock);
        }
//        //�ȱ����ֳ����洢֮ǰ��scheduleģʽ
//        //��һ��C2��
//        if(packageFinishFlag == RESET && packageStartFlag == RESET)
//        {
//            printf("cS2\r\n");
//            packageStartFlag = SET;
//            if(Schedule_ModeInitFlag == SET)
//            {
//                //EEPROM_WriteBytes(0, (uint8_t*)CalendarList,sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//                //tool_memcpy(CalendarList_Temp, CalendarList, sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//            }
//            //else
//            //{
//                memset(CalendarList, 0, sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//            //}
//        }
//        else if(packageFinishFlag == SET)
//        {
//            printf("cS\r\n");
//            memset(CalendarList, 0, sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//        }
        
        Schedule_CMD_State = Handel_ServerData_Schedule_Config(RxMessage+(Server_CMD_Data_Len)*2, UsartType.RX_Size);
        
//����C2����������
        if(packageFinishFlag == SET)
        {
            if(Schedule_CMD_State.cmdResult == SUCCESS)
            {
                printf("C2_Success\r\n");
                if(Schedule_CMD_State.configMode == SCHEDULE_MODE_DAYCYCLE)
                {
                    CurrentDayInTheLoop = 0;
                }
                Schedule_ModeFlag = SET;
            }
            //����ִ��ʧ��
            else
            {
                printf("C2_Fail\r\n");
            }
            
            Schedule_StateStartFlag = SET;
            DataRspclock.timeOut = 3000;
            ScheduleParmSet(CurrentDayInTheLoop);
            User_StartClock(&DataRspclock);
        }
//                C2_command_FailFlag = RESET;
//                if(RegistFlag == RESET)
//                {
//                    data_confirmFlag = SET;
//                }
//                
//                if(Schedule_CMD_State.configMode == SCHEDULE_MODE_DAYCYCLE)
//                {
//                    CurrentDayInTheLoop = 0;
//                }
//                //ͬ����ʱ���
//                if(Time_synInitFlag == SET)
//                {
//                    Schedule_ModeFlag = SET;
//                    //��ʾ�����ù�scheduleģʽ
//                    if(Schedule_ModeInitFlag == RESET)
//                    {
//                        Schedule_ModeInitFlag = SET;
//                    }
//                    //���ûظ�D4
//                    curAT_RSP_ID = AT_DATA_RSP;
//                    Schedule_Device_Confirm_Report(SCHEDULE_CONFIG_ID, 0, Schedule_CMD_State.packetNum,0xFF, 0);
//                    Schedule_State = SET;
//                    
//                }
//                else
//                {
//                    schedule_congif_Flag = SET;
//                    //ͬ��ʱ���
//                    printf("Tsyn\r\n");
//                    curAT_RSP_ID = AT_DATA_RSP;
//                    //Time_synchronization();
//                    
//                }                  
//            }

//                C2_command_FailFlag = SET;
//                
//                if(Schedule_ModeInitFlag == SET)
//                {
//                    printf("C2_Recover\r\n");
//                    //���֮ǰ������ -- �ָ�
//                    //EEPROM_ReadBytes(0,(uint8_t*)CalendarList,sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//                    //tool_memcpy(CalendarList, CalendarList_Temp, sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//                }
//                //���ûظ�D4
//                curAT_RSP_ID = AT_DATA_RSP;
//                Schedule_Device_Confirm_Report(SCHEDULE_CONFIG_ID, 1, Schedule_CMD_State.packetNum,0xFF, 0);
//            }
//        }
    }
//    //�·�����-- ����Ҫ���豸ͬ��ʱ���C3
//    else if(Server_CMD_Data.cmd == SERVER_ID_TIME_SYN)
//    {
//        printf("C3\r\n");
//        if(RegistFlag == SET)
//        {
//            //D5
//            printf("D5\r\n");
//            curAT_RSP_ID = AT_DATA_RSP;
//            Schedule_Device_Confirm_Report(TIME_SYN_REQ_ID, 0, Schedule_CMD_State.packetNum, 0xFF, 0);
//        }
//    }
//    //�·�����-- ͬ��ʱ���C4
//    else if(Server_CMD_Data.cmd == SERVER_ID_DEVICE_TIME_SYN)
//    {
//        printf("C4\r\n");
//        Time_synInitFlag = SET;
//        
//        if(Handel_ServerData_TimeSyn(RxMessage+(Server_CMD_Data_Len)*2+1, dataLen))
//        {
//            printf("TsynOK\r\n");
//            Time_synFlag = SET;
//        }
//        
//        if(schedule_congif_Flag == SET)
//        {
//            User_StopClock(&TimeSynclock);
//            schedule_congif_Flag = RESET;
//            //���ûظ�D4
//            curAT_RSP_ID = AT_DATA_RSP;
//            Schedule_Device_Confirm_Report(SCHEDULE_CONFIG_ID, 0, Schedule_CMD_State.packetNum,0xFF, 0);
//            printf("D4\r\n");
//            Schedule_State = SET;
//            
//            currentMode = POSITIONING_MODE;
//            dataCycle = DEFAULT_T_CYCLE;
//                
//            Schedule_ModeFlag = SET;
//        }
//        else if(Time_syning_Flag == SET)
//        {
//            User_StopClock(&BoardCloseTimeoutclock);
//            ScheduleParmSet(CurrentDayInTheLoop);
//            Time_synFlag = SET;
//            
//            wakeupCounter = 0;
//            SleepStatus = SLEEP;
//            printf("D4Sleep0\r\n");
//        }
//    }   
}


/*
* @fn      Handel_ServerData_Info_RSP()
* @brief   
* @param   
* @return 
*/
uint8_t Handel_ServerData_Info_RSP(uint8_t* RxMessage, uint16_t dataLen)
{
    uint8_t hex[100];
    uint16_t GPSCollectionCycle_t;
    uint16_t NB_dataCycle_t;
    
    //hex = malloc(dataLen);
    ascsToHexs((char*)RxMessage, hex, dataLen*2);
    tool_memcpy(&Locator_pram, hex, dataLen);
    //free(hex);
    //    if(Locator_pram.enterSleep == SET)
    //    {
    //        //��������
    //        SleepStatus = SLEEP;
    //    }
    
    //�ƶ�����
    Locator_Data.userAlarmFlag = RESET;
    if(Locator_pram.enterSleep == MOVE_ALARM)
    {
        move_alarm_enable_Flag = SET;
        move_alarm_Flag = RESET;
        Locator_Data.sensorFlag = 0x02;
        //move_alarm_Init_Flag = SET;
        return 1;
    }
    else if(Locator_pram.enterSleep == EXIT_MOVE_ALARM)
    {
        currentMode = Locator_Data.mode;
        if(Schedule_ModeFlag == SET)
        {
            ScheduleParmSet(CurrentDayInTheLoop);
        }
        else
        {
            if(currentMode == NORMAL_MODE)
            {
                dataCycle = Locator_Data.FindnowCycle;
            }
            else if(currentMode == POSITIONING_MODE)
            {
                dataCycle = Locator_Data.TrackerCycle;
            }
        }
        Locator_Data.sensorFlag = 0x00;
        move_alarm_enable_Flag = RESET;
        move_alarm_Flag = RESET;
        move_alarm_Init_Flag = RESET;
        return 1;
    }
    
    //�����������ģʽ
    if(Locator_pram.newMode > POSITIONING_MODE)
    {
        if(Locator_pram.newMode == SCHEDULE_MODE)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        if(Locator_pram.enterSleep != SPECIAL_FINDNOW)
        {
            currentMode = Locator_pram.newMode;
            Locator_Data.mode = currentMode;
        }
        if(move_alarm_enable_Flag == SET)
        {
            MoveAlarmPreMode = Locator_pram.newMode;
        }
    }
    //����
    GPSCollectionCycle_t = BigtoLittle16(Locator_pram.GPSCollectionCycle);
    NB_dataCycle_t = BigtoLittle16(Locator_pram.NB_dataCycle);
    
    if(GPSCollectionCycle_t <= 1440 && NB_dataCycle_t <= 1440 )
    {
        Locator_Data.TrackerCycle = GPSCollectionCycle_t;
        Locator_Data.FindnowCycle = NB_dataCycle_t;
        if(Locator_Data.TrackerCycle <= GPS_KEEP_MIN)
        {
            TrackerCycle_short_Flag = SET;
        }     
#warning �����180328
        if(GPSCollectionCycle_t == 1)
        {
            //Tracking_1minFlag = SET;
        }
        if(NB_dataCycle_t == 1)
        {
            //FindNow_1minFlag = SET;
        }
        
        return 0;
    }
    else
    {
        return 1;
    }
}


/**
* @brief  ScheduleParmSet
* @param  None
* @retval None
*/
void ScheduleParmSet(uint8_t CurrentDayInTheLoop)
{
    uint8_t  date=0;
    getRTC_Date_Time();
//    if(DaySwitchCounter == 0)
//    {
        //����ѭ��
        if(Schedule_CMD_State.configMode == SCHEDULE_MODE_DAYCYCLE)
        {
            date = CurrentDayInTheLoop+1;
            
            Schedule_Mode_Data = getFollowedMode(date);
            dataCycle = Schedule_Mode_Data.cycle;
            currentMode = Schedule_Mode_Data.mode;
        }
        //����ѭ��
        else if(Schedule_CMD_State.configMode == SCHEDULE_MODE_WEEKCYCLE)
        {
            date = sdatestructure.WeekDay;
            
            Schedule_Mode_Data = getFollowedMode(date);
            dataCycle = Schedule_Mode_Data.cycle;
            currentMode = Schedule_Mode_Data.mode;
        }
        //����ѭ��һ��
        else if(Schedule_CMD_State.configMode == SCHEDULE_MODE_WEEKCYCLEONETIME)
        {
            date = sdatestructure.WeekDay;
            //���ܽ����ж�
            if(endWeekCycle())
            {
                //�˳�scheduleģʽ
                Schedule_ModeFlag = RESET;
                currentMode = Locator_Data.mode;
                dataCycle = 0xFFFF;
            }
            else
            {
                Schedule_Mode_Data = getFollowedMode(date);
                dataCycle = Schedule_Mode_Data.cycle;
                currentMode = Schedule_Mode_Data.mode;
            }
        }
        
        if(ExtraDataupFlag == SET)
        {
            currentMode = POSITIONING_MODE;
        }
        //���D3�е�
        Schedule_Data_upload.Date =  date;
        Schedule_Data_upload.cycle = Schedule_Mode_Data.config_cycle;
        Schedule_Data_upload.Mode = currentMode; 
        
        if(currentMode != POSITIONING_MODE)
        {
            if(Locator_pram.enterSleep != SPECIAL_FINDNOW)
            {
                Schedule_Data_upload.GPS_List_num = 0;
                Schedule_Data_upload.GPS_data_len = 1;
            }
        }
        
        if(Schedule_ModeFlag == SET)
        {
            printf("Schedule_cycle: %d\r\n", dataCycle);
            printf("Schedule_Mode: %d\r\n", currentMode);
            printf("Schedule_Date: %d\r\n", CurrentDayInTheLoop + 1);
        }
//    }
}


/*
* @fn      Handel_coap_Data()
* @brief   ����coap���ݷ��͵�����
* @param   
* @return 
*/
void Handel_coap_Data(uint8_t* RxMessage)
{
    char* str = NULL;
    char* str1 = NULL;
    str2 = NULL;
    uint16_t data_Len;
    uint8_t size = 1; //��λ��ʮλ����λ��
    
//    str2 = strstr((char*)RxMessage, AT_COAP_RSP_OK);
//    if(str2 != NULL)
//    {
//        //upload_OK_Flag = SET;
//    }
    
    str2 = mystrstr((char*)RxMessage, AT_FOTA, UsartType.RX_Size);
    str = mystrstr((char*)RxMessage, "NNMI", UsartType.RX_Size);
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)RxMessage, UsartType.RX_Size);
    if(str != NULL || str2 != NULL)
    {
        //str1 = strtok((char*)RxMessage+AT_COAP_RX_HEAD_LEN, ",");
        if(str != NULL)
        {
            str1 = str+5;
        }
        //ת��Ϊ��ֵ
        data_Len = atoi(str1);
        if(data_Len >512)
        {
            return;
        }
        else if(data_Len>99)
        {
           size  = 3;
        }
        else if(data_Len > 9)
        {
            size = 2;
        }
        //FOTA
        printf("data_Len:%d\r\n", data_Len);
        //printf("FOTA_Cmplt_Flag:%d\r\n", FOTA_Cmplt_Flag);
        //HAL_UART_Transmit_DMA(&huart2, RxMessage, 20);
        //if(strncmp((char*)RxMessage+AT_COAP_RX_HEAD_LEN+size+AT_COMMA_LEN, "FFFE", 4) == STR_CMP_TRUE)
        if(str2 != NULL)
        {
            printf("FFFE\r\n");
            uint8_t hex[100];
            
            if(FOTA_Cmplt_Flag == 0)
            {
                OTAingFlag = SET;
            }
            else
            {
                OTAingFlag = RESET;
            }
            ascsToHexs(str2+1, hex, data_Len*2);
            NB_Recv_Data_Process(hex, data_Len);
        }  
        else if(data_Len == 8 && OTAingFlag == RESET)
        {                      
            if(Normal_StateFlag == RESET && Schedule_StateStartFlag == RESET)
            {
                if(strncmp((char*)RxMessage+10, "FAFA", 4) == STR_CMP_TRUE)
                {
                    if(Time_synFlag == RESET)
                    {
                        printf("Time_syn\r\n");
                        Handel_UNIX_Data(RxMessage+AT_COAP_RX_HEAD_LEN+size+AT_COMMA_LEN+4);
                    }
                }
                //upload_OK_Flag = RESET;
                //Coap_EnterSleep();
            }
        }
        else if(data_Len == 2 && OTAingFlag == RESET)
        {
            printf("ServerRsp\r\n");
            if(Normal_StateFlag == RESET && Schedule_StateStartFlag == RESET)
            {
                if(Locator_pram.enterSleep == SET)
                {
                    Locator_pram.enterSleep = RESET;
                    NVIC_SystemReset();
                }
                else if(Locator_pram.enterSleep == SPECIAL_FINDNOW)
                {
                    //Locator_pram.enterSleep = RESET;
                    
                    Schedule_StateStartFlag = RESET;
                    Normal_StateFlag = RESET;
                    User_StopClock(&DataRspclock);
                    User_StopClock(&BoardCloseTimeoutclock);
                    
                    printf("SpecialFN\r\n");
                    Device_Open_GPS();
                    //special_findnowFlag = SET;
                    DataUpFlag = RESET;
                    NB_InNetWork_Flag = RESET;
                }
                else
                {
                    if(RegistFlag == SET ||  RegistFinishFlag == SET)
                    {
                        Coap_EnterSleep();
                    }
                    else
                    {
                        RegistStateFlag = SET;
                        curAT_RSP_ID = AT_DATA_RSP;
                        
                        currentMode = POSITIONING_MODE;
                        TrackerCycle_short_Flag = SET;
                        Locator_Data.initFlag = SET;
                        Locator_Data.mode = currentMode;
                        
//                        NB_Report_to_ServerData.cmdlen = END_NUM_LEN + CHECK_SUM_LEN + sizeof(NB_Report_to_ServerData_t) + sizeof(Locator_Data_t) + Locator_Data.GPS_List_num * sizeof(GPS_List_t)+ sizeof(Signal_Level_t);
//                        printf("RegistState\r\n");
//                        NB_AT_CMD_Send(AT_NSOST, LOCATOR_DATA_REPORT, NB_Report_to_ServerData, Locator_Data, GPS_Info);
                        
                        Retry_state = RESET;
                        AT_Retry_Num = 0;
                        User_StartClock(&DataRptclock);
                    }
                    
                }
            }
        }
        else if(data_Len > 8)
        {
            if(OTAingFlag == RESET)
            {
                cmd_OK_Flag = SET;
                printf("processingData\r\n");
                Handel_ServerData(RxMessage+AT_COAP_RX_HEAD_LEN+size+AT_COMMA_LEN, data_Len);
            }
        }
    }
    else
    {
        printf("Other\r\n");
        //HAL_UART_Transmit_DMA(&huart2, RxMessage, 10);
    }
}


/*
* @fn      Handel_UNIX_Data()
* @brief   ����UNIX����
* @param   
* @return 
*/
void Handel_UNIX_Data(uint8_t* RxMessage)
{
    char str[8];
    uint8_t hex[4];
    uint32_t sec1 = 0;
    uint32_t sec = 0;
    uint32_t sec2 = 0;
    
    strncpy(str, (char*)RxMessage, 8);
    ascsToHexs(str, hex, 8);
    sec =  hex[0]<<24;
    sec1 = hex[1]*65536+hex[2]*256+hex[3];
    sec2 = sec + sec1;
    utc_sec_2_mytime(sec2 + 8 * SEC_PER_HOUR, &my_time, false);
    
    if(setRTC_Date_Time(my_time.nYear, my_time.nMonth, my_time.nDay, my_time.nHour, my_time.nMin, my_time.nSec, my_time.DayIndex) == SUCCESS)
    {
       Time_synFlag = SET;
    }     
}


/*
* @fn      Coap_EnterSleep()
* @brief   ��������
* @param   
* @return 
*/
void Coap_EnterSleep(void)
{
    Schedule_StateStartFlag = RESET;
    Normal_StateFlag = RESET;
    if(move_alarm_enable_Flag == SET)
    {
        if(move_alarm_Init_Flag == RESET)
        {
            move_alarm_Init_Flag = SET;
            MoveAlarmPreMode = currentMode;
        }
        Locator_Data.sensorFlag = 0x02;
    }
    else
    {
        Locator_Data.sensorFlag = 0x00;
    }
    printf("enterSleep\r\n");
    curAT_RSP_ID = AT_COAP_ENTER_SLEEP;
    Retry_state = RESET;
    AT_Retry_Num = 0;
    User_StartClock(&ATTimeoutclock);
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)coapEnterSleep, 32);
}


void NB_Recv_Data_Process(uint8_t *buf, uint16_t len)
{
	uint16_t head = buf[0]*256 + buf[1];
	switch(head)
	{
        case 0xFFFE:
        {
            //У��
            uint16_t crc16_recv = buf[4]*256+buf[5];//�յ���crc16ֵ
            buf[4] = 0;
            buf[5] = 0;
            uint16_t crc16 = 0;
            crc16 = crc_check(buf, len);
            if(crc16 != crc16_recv)
            {
                printf("crc16 error\nrecv:0x%04X,true:0x%04X\n", crc16_recv, crc16);
                break;
                //return;
            }
            
            uint8_t msg_id = buf[3];
            uint8_t *data = buf + 8;
            uint16_t data_len = buf[6]*256+buf[7];
            FOTA_Msg_Process( msg_id, data, data_len);
        }break;
	}
}



