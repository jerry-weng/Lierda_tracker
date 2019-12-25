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
#include <stdio.h>
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

//��������
Product_Test_t Product_Test = {0};

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

//���ݷ�����ɱ�־
uint8_t NB_min_counter = 0;
uint8_t NB_StateSend_counter = 0;
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

Clock_Struct NBReportclock;
Clock_Struct GPSCollectionclock;
Clock_Struct BoardCloseTimeoutclock;
Clock_Struct LEDclock;
Clock_Struct DataRspclock;
Clock_Struct InNetTimeoutclock;
Clock_Struct DeviceRegistclock;

void Handel_ServerAT_CMD(uint8_t* RxMessage);
void Handel_ServerData(uint8_t* RxMessage, uint16_t dataLen);
uint8_t Form_AT_Head(uint8_t datalen);
uint8_t Handel_ServerData_Info_RSP(uint8_t* RxMessagen, uint16_t dataLen);
//Device_rsp_t Handel_Server_ctl_cmd( Server_ctl_cmd_t Server_ctl_cmd, uint8_t *Server_ctl_data);
void Locator_Data_Report(NB_Report_to_ServerData_t NB_Report_to_ServerData, Locator_Data_t Locator_Data, GPS_Info_t GPS_Info);
void Device_Confirm_Report(uint8_t serviceCMD_Process_result);
void NBReportclockProcess(void);
void GPSCollectionclockProcess(void);
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
    HAL_GPIO_WritePin(NB_SW_GPIO_Port, NB_SW_Pin, GPIO_PIN_RESET);
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
    //AT+��������������� -- ����
    else if(cmdID == AT_NSOST)
    {
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
              Locator_Data_Report(NB_Report_to_ServerData_tmp, Locator_Data_tmp, GPS_Info_tmp);
          }
          //����ȷ��0xD2
          else if(ReportID == LOCATOR_CMD_CONFIRM)
          {
              va_end(arg_ptr);
              Device_Confirm_Report(serviceCMD_Process_result);
          }
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
    }
    else if(cmdID == AT_CGMR)
    {
        //��Ҫ�ȴ��ظ� ���� +"OK" 
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4;
        tool_strcat(AT_NO_PRAM_LEN, AT_START, CGMR, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
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
        uint8_t off_set_Len = 0;
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
* @brief   ��������ϱ��ն�ȷ������
* @param   
* @return   
*/
void Device_Confirm_Report(uint8_t serviceCMD_Process_result)
{
    uint8_t report_to_ServerDataLen_D = 0;
    uint16_t SumLen = 0;
    uint8_t ServerDataLenSize;
    uint8_t crc;
     
    report_to_ServerDataLen_D = sizeof(NB_Report_to_ServerData_t) + 3;
    
    //����ն�ȷ������ �ֶ�����
    NB_Report_to_ServerData_t Device_confirm_Data = {0};
    Device_confirm_Data.start = SERVER_START;
    Device_confirm_Data.cmd = SERVER_ID_CMD_RSP;
    Device_confirm_Data.version = CMD_VERSION;
    Device_confirm_Data.softWare_version = SW_VERSION;
    Device_confirm_Data.cmdlen = BigtoLittle16(report_to_ServerDataLen_D);
    Device_confirm_Data.msg_seq_num = BigtoLittle16(msg_seq_num - 1);
    
    tool_memcpy(Device_confirm_Data.IMEI_num, NB_Report_to_ServerData.IMEI_num, IMEI_NUM_LEN);    
    //���������ܳ�
    ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D-1);
    SumLen = AT_TEST_PORT_LEN + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_TAIL_NUM;
    
    //�������ִ�н���ֶ�����
    uint8_t off_set_Len = 0;  
    off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize;
    
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
    tool_memcpy(at_cmd_data_p + off_set_Len, AT_END, AT_TAIL_NUM);
    
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}


/*
* @fn      Form_AT_Head(uin8_t datalen)
* @brief   ��� at_cmd_data_p �ϱ�ATָ��֡ͷ
* @param   
* @return  λ��  
*/
uint8_t Form_AT_Head(uint8_t datalen)
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
    
    tool_strcat(AT_HEARER_NUM, AT_START, NSOSTF, AT_PRAM_0, AT_PRAM_IP, AT_PRAM_PORT, AT_TxFlag_replyRelease);
    
    //tool_strcat(AT_HEARER_NUM, AT_START, NSOST, AT_PRAM_0, AT_PRAM_IP, AT_PRAM_PORT);
    //tool_memcpy(at_cmd_data_p, at_cmd_data_head, AT_TEST_PORT_LEN);
    //free(at_cmd_data_head);
    
    //�� AT+NSOST=0,218.244.145.31,8000,120,....\r\n
    datalen = datalen + CHECK_SUM_LEN;
    if(datalen > 100)
    {
        at_cmd_data_p[AT_TEST_PORT_LEN] = datalen/100 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+1] = datalen%100/10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+2] = datalen%100%10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+3] = ',';
    }
    else
    {
        at_cmd_data_p[AT_TEST_PORT_LEN] = datalen%100/10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+1] = datalen%100%10 + '0';
        at_cmd_data_p[AT_TEST_PORT_LEN+2] = ',';       
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
    uint8_t nb_server_data_len = 0;
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
    crcData = checkSumChar((char*)RxMessage+2, (dataLen-8));
    RxcrcData = charsToHex((char*)RxMessage+(dataLen-4));
    
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
    
    //�ϱ�Ӧ��C0
    if(Server_CMD_Data.cmd == SERVER_ID_DATA_RSP)
    {
        printf("C0\r\n");
        //������
        if(RegistFlag == RESET)
        {
            NB_StateSend_counter = 0;
            if(data_confirmFlag == SET)
            {
                data_confirmFlag = RESET;
                RegistFlag = SET;
                DataUpFlag = SET;
                Locator_Data.initFlag = SET;
                //after_registFlag = SET;
                printf("registSuccess\r\n");
                
                User_StopClock(&DataRspclock);
                User_StopClock(&BoardCloseTimeoutclock);
                //������ɺ����GPS��Ϣ
                if(currentMode == POSITIONING_MODE)
                {
                    GPS_Info_Delete();
                }
                NB_min_counter = 0;
                Close_NB();
                printf("Sleep4");
                wakeupCounter = 4320;
                SleepStatus = SLEEP;
            }
        }
        else if(RegistFlag == SET)
        {
//            User_StopClock(&DataRspclock);
//            User_StopClock(&BoardCloseTimeoutclock);
//            //������ɺ����GPS��Ϣ
//            if(currentMode == POSITIONING_MODE)
//            {
//                GPS_Info_Delete();
//            }
//            NB_min_counter = 0;
//            NB_open_Flag = RESET;
//            Close_NB();
//            
//
//#warning ���0208
//            if(Locator_pram.enterSleep == SPECIAL_FINDNOW)
//            {
//                Locator_pram.enterSleep = RESET;
//                
//                printf("SpecialFN\r\n");
//                Device_Open_GPS();
//                special_findnowFlag = SET;
//                DataUpFlag = RESET;
//                NB_InNetWork_Flag = RESET;
//            }
//            else
//            {
//                wakeupCounter = 0;
//                SleepStatus = SLEEP;
//                printf("Sleep0\r\n");
//            }
         }
    }
    //�·�����c1
    else if(Server_CMD_Data.cmd == SERVER_ID_CMD_DATA)
    {
        //��ȡ���������      
        if(dataLen > 0)
        {
            if(RegistFlag == RESET)
            {        
                User_StopClock(&DeviceRegistclock);
            }
            printf("C1\r\n");
            serviceCMD_Process_result = Handel_ServerData_Info_RSP( RxMessage+(Server_CMD_Data_Len)*2+1 , sizeof(Locator_pram_t));            
            //�豸����Ӧ��������ϱ�һ֡��ǰ״̬������
            DataRspclock.timeOut = 1000;
            User_StartClock(&DataRspclock);
        }
    }
    //�·�����-- ����Scheduleģʽ����C2
    else if(Server_CMD_Data.cmd == SERVER_ID_SCHEDULE_CONFIG)
    {
        if(UsartType2.RX_Size != 0)
        {
            //uint8_t c2Flag;
            //Schedule_Mode_t Schedule_Mode;
            //if ID == 0xC2
            //tool_memcpy(CalendarList_Temp, CalendarList, sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
            Handel_ServerData_Schedule_Config(UsartType2.RX_pData+14, UsartType2.RX_Size);
//            if(c2Flag == SUCCESS)
//            {
//                Schedule_Mode = getFollowedMode( 3 );
//            }
//            else
//            {
//                tool_memcpy(CalendarList, CalendarList_Temp, sizeof(Calendar_t)*CYCLEDAYS_MAX*HOURS_GROUP_MAX);
//            }
        }
    }
    //�·�����-- ����Ҫ���豸ͬ��ʱ���C3
    else if(Server_CMD_Data.cmd == SERVER_ID_TIME_SYN)
    {
        
    }
    //�·�����-- ͬ��ʱ���C4
    else if(Server_CMD_Data.cmd == SERVER_ID_DEVICE_TIME_SYN)
    {
        
    }   
}


/*
* @fn      Handel_ServerData_Info_RSP()
* @brief   
* @param   
* @return 
*/
uint8_t Handel_ServerData_Info_RSP(uint8_t* RxMessage, uint16_t dataLen)
{
    uint8_t hex[512];
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
    //�����������ģʽ
    if(Locator_pram.newMode > POSITIONING_MODE)
    {
        return 1;
    }
    else
    {
        currentMode = Locator_pram.newMode;
        Locator_Data.mode = currentMode;
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
        return 0;
    }
    else
    {
        return 1;
    }
}

///*
//* @fn      Handel_Server_ctl_cmd( Server_ctl_cmd_t Server_ctl_cmd )
//* @brief   ��������� �ϱ�Ӧ�� �е������ֶ� 
//* @param   
//* @return  
//*/
//Device_rsp_t Handel_Server_ctl_cmd( Server_ctl_cmd_t Server_ctl_cmd, uint8_t *Server_ctl_data)
//{      
//    //�����ϱ�ģʽ��0xC001��������
//    if(Server_ctl_cmd.cmdID == SERVER_ID_REPORT_MODE)
//    {
//        Device_rsp.cmd_ID = SERVER_ID_REPORT_MODE;
//    }
//    //��ѯ�汾��(0xC002)
//    else if(Server_ctl_cmd.cmdID ==  SERVER_ID_REPORT_REV)
//    {
//        Device_rsp.cmd_ID = SERVER_ID_REPORT_REV;
//    } 
//    //���ö�λ�����0xC106��
//    else if(Server_ctl_cmd.cmdID == SERVER_ID_LOCATOR_CTL)
//    {
//        Device_rsp.cmd_ID = SERVER_ID_LOCATOR_CTL;
//        //Device_rsp.rsp_data = malloc(Server_ctl_cmd.data_len);
//        //�ж����������
//        if(Server_ctl_cmd.data_len != sizeof(Locator_pram_t))
//        {
//            Device_rsp.cmd_state = CONFIG_ERROR;
//            return Device_rsp;
//        }
//        else
//        {
//            tool_memcpy(&Locator_pram, Server_ctl_data, Server_ctl_cmd.data_len);
//            Locator_pram.GPSCollectionCycle = BigtoLittle16(Locator_pram.GPSCollectionCycle);
//            Locator_pram.NB_dataCycle = BigtoLittle16(Locator_pram.NB_dataCycle);   
//            //�жϲ�����Χ
//            if(Locator_pram.GPSCollectionTime > GPS_COLLECTION_TIME_MAX)
//            {
//                Locator_pram.GPSCollectionTime = GPS_COLLECTION_TIME_MAX;
//            }
//            if(Locator_pram.enterSleep == SET)
//            {
//                //��������
//                LED_Blink_Finish = RESET;
//                SleepStatus = SLEEP;
//            }
//            Device_rsp.cmd_state = CONFIG_SUCCESS;
//            //Device_rsp.rsp_data_len = Server_ctl_cmd.data_len;
//            //tool_memcpy(Device_rsp.rsp_data, &Locator_pram, Server_ctl_cmd.data_len);
//        }
//    }
//    return Device_rsp;
//}