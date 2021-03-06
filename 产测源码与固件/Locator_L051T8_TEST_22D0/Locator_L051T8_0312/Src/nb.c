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
char NB_version[30];                        //版本号
uint8_t NB_version_len = 0;

//char* at_cmd_data_p = NULL;                 //AT指令

//uint8_t at_cmd_data_p[512];                 //AT指令

uint8_t *nbRxMessage = NULL;                //NB串口数据

Device_rsp_t Device_rsp;                    //填充上报应答中的命令
Device_confirm_seq_t Device_confirm_seq;    //终端确认命令0xD2D2
uint16_t Device_confirm_seq_len = 0;        //命令执行结果长度

Locator_pram_t Locator_pram = {0};          //定位器参数

//定位器数据上报格式
Locator_Data_t Locator_Data = {0};

//产测命令
Product_Test_t Product_Test = {0};

//模式--开机默认注册模式
uint8_t currentMode = REGIST_MODE;

//Sleep
uint8_t SleepStatus = AWAKE;
uint8_t LED_Init_Blink_Finish = RESET;

//连上基站标志位
uint8_t NB_Station_Connected_Flag = RESET;
//是否注册成功
uint8_t RegistFlag = RESET;

//按键发送注册
//uint8_t RegistConfirmFlag = RESET;

uint8_t DataUpFlag = RESET;
uint8_t data_confirmFlag = RESET;
uint8_t after_registFlag = RESET;

//数据发送完成标志
uint8_t NB_min_counter = 0;
uint8_t NB_StateSend_counter = 0;
//NB上报至服务器数据
NB_Report_to_ServerData_t NB_Report_to_ServerData = {0};

//默认AT指令
uint8_t curAT_RSP_ID = AT_CFUN0/*AT_CFUN0*/;

//命令执行结果
uint8_t serviceCMD_Process_result = ERROR; 

//序列号
uint16_t msg_seq_num = 0;

//定时器计数值
uint32_t wakeupCounter = 0;

//定位器周期切换至小于2小时，标志位

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
* @brief   IO口控制NB开关
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
* @brief   向NB模组发送AT指令  
* @param   指令ID,参数...
* @return   
*/
void NB_AT_CMD_Send( uint8_t cmdID_t, ...)
{
    uint8_t cmdID;
    cmdID = cmdID_t;
    //AT+开关协议栈
    if(cmdID == AT_CFUN0 || cmdID == AT_CFUN1)
    {
        //需要回复"OK"
        uint8_t SumLen = 0;
        
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        //关闭协议栈
        if(cmdID == AT_CFUN0)
        {
            //AT+CFUN=0\r\n
            tool_strcat(AT_ONE_PRAM_LEN, AT_START, CFUN, AT_PRAM_0, AT_END);   
        }
        //开启协议栈
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
    //设置IOT平台地址
    else if(cmdID == AT_NCDP)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_IOT_IP_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, NCDP, AT_IOT_IP, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
    }    
    //查询IMEI
    else if(cmdID == AT_CGSN)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CGSN, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //软重启
    else if(cmdID == AT_NRB)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_3;
        
        tool_strcat(AT_NO_PRAM_LEN, AT_START, NRB, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //开启错误提示
    else if(cmdID == AT_CMEE)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CMEE, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //设置APN
    else if(cmdID == AT_CGDCONT)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_7 + AT_CMDID_PRAM_LEN + AT_CGDCONT_CTNET_LEN;
        
        tool_strcat(AT_TWO_PRAM_LEN, AT_START, CGDCONT, AT_PRAM_1, AT_CGDCONT_CTNET, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //开启下行数据通知
    else if(cmdID == AT_NNMI)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, NNMI, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //自动搜网
    else if(cmdID == AT_CGATT)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_5 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CGATT, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //设置基站连接通知
    else if(cmdID == AT_CSCON)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_5 + AT_CMDID_PRAM_LEN;
        
        tool_strcat(AT_ONE_PRAM_LEN, AT_START, CSCON, AT_PRAM_1, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //查询核心网分配的IP地址
    else if(cmdID == AT_CGPADDR)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_7;
        
        tool_strcat(AT_NO_PRAM_LEN, AT_START, CGPADDR, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //查询网络状态
    else if(cmdID == AT_NUESTATS)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_8;
        
        tool_strcat(AT_NO_PRAM_LEN, AT_START, NUESTATS, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    //建立SOCKET (端口xxxx)
    else if(cmdID == AT_NSOCR)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_5 + AT_PRAM_SOCKET_LEN + AT_PRAM_POART_LEN;
        
        tool_strcat(AT_TWO_PRAM_LEN, AT_START, NSOCR, AT_PRAM_SOCKET, AT_PRAM_POART, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    } 
    //AT+向服务器发送数据 -- 三种
    else if(cmdID == AT_NSOST)
    {
          int ReportID;
          va_list arg_ptr;
          va_start(arg_ptr, cmdID_t);
          ReportID = va_arg(arg_ptr, int);
          
          //数据上报0xD1
          if(ReportID == LOCATOR_DATA_REPORT)
          {
              //NB端数据上报格式
              NB_Report_to_ServerData_t NB_Report_to_ServerData_tmp;
              //定位器数据上报格式
              Locator_Data_t Locator_Data_tmp;
              //GPS数据格式
              GPS_Info_t GPS_Info_tmp;
              
              //获取数据
              NB_Report_to_ServerData_tmp = va_arg(arg_ptr, NB_Report_to_ServerData_t);
              Locator_Data_tmp = va_arg(arg_ptr, Locator_Data_t);
              GPS_Info_tmp = va_arg(arg_ptr, GPS_Info_t);
              va_end(arg_ptr);
              //数据组帧 与发送
              Locator_Data_Report(NB_Report_to_ServerData_tmp, Locator_Data_tmp, GPS_Info_tmp);
          }
          //命令确认0xD2
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
    //AT+向服务器发送获取接收数据命令
    else if(cmdID == AT_NSORF)
    {
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4 + AT_NSORF_LEN;
        tool_strcat(AT_NSOST_PRAM_LEN, AT_START, NSORF, AT_PRAM_0, AT_NSORF_NUM, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
    else if(cmdID == AT_CGMR)
    {
        //需要等待回复 数据 +"OK" 
        uint8_t SumLen = 0;
        SumLen = AT_START_END_LEN + AT_CMDID_LEN_4;
        tool_strcat(AT_NO_PRAM_LEN, AT_START, CGMR, AT_END);
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    }
}

/*
* @fn      Locator_Data_Report()
* @brief   向服务器上报定位器数据
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
               
        //字段长度
        NB_Report_to_ServerData_Len = sizeof(NB_Report_to_ServerData_t);
        Locator_Data_Len = sizeof(Locator_Data_t);
        GPS_Data_Len = sizeof(GPS_List_t);
        
        //总数据长度
        report_to_ServerDataLen_D = NB_Report_to_ServerData_Len + Locator_Data_Len + (Locator_Data.GPS_List_num)*GPS_Data_Len + 1;
        
        //组成AT指令帧头
        ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D);
        SumLen = AT_TEST_PORT_LEN + ServerDataLenSize + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_TAIL_NUM;
          
        //添加NB字段与定位器字段数据
        uint8_t off_set_Len = 0;
        uint8_t at_head_len = 0;
        //NB包头
        off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize;
        at_head_len = AT_TEST_PORT_LEN + ServerDataLenSize;
        
        hex2String((uint8_t*)&NB_Report_to_ServerData_tmp, NB_Report_to_ServerData_Len, (char*)at_cmd_data_p + at_head_len);
        
        off_set_Len = off_set_Len + NB_Report_to_ServerData_Len*2;
        //定位器包头
        hex2String((uint8_t*)&Locator_Data_tmp, Locator_Data_Len, (char*)at_cmd_data_p + off_set_Len);

        off_set_Len = off_set_Len + Locator_Data_Len*2;
        
        //添加校验和
        uint8_t crc = 0;
        
        //GPS数据
        for(uint8_t i=0; i<Locator_Data.GPS_List_num; i++)
        {
            hex2String((uint8_t*)&GPS_Info_tmp.GPS_List[i], GPS_Data_Len, (char*)at_cmd_data_p + off_set_Len);           
            off_set_Len = off_set_Len + GPS_Data_Len*2;
            crc = crc + checkSum((uint8_t *)&GPS_Info_tmp.GPS_List[i], GPS_Data_Len);
        }

        //计算除去包头之外的数据
        crc = crc + checkSum((uint8_t *)&NB_Report_to_ServerData_tmp+1, NB_Report_to_ServerData_Len-1); 
        crc = crc + checkSum((uint8_t *)&Locator_Data_tmp, Locator_Data_Len);
        
        hex2String( &crc, CHECK_SUM_LEN, (char*)at_cmd_data_p + off_set_Len);        
        
        
        //添加结束位
        uint8_t server_end;
        server_end = SERVER_START;
        hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2);
        off_set_Len = off_set_Len + 2;
        
        //添加"\r\n"
        tool_memcpy((char*)at_cmd_data_p + off_set_Len + CHECK_SUM_LEN*2, AT_END, AT_TAIL_NUM);
        
        HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}


/*
* @fn      Device_Confirm_Report
* @brief   向服务器上报终端确认命令
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
    
    //添加终端确认命令 字段数据
    NB_Report_to_ServerData_t Device_confirm_Data = {0};
    Device_confirm_Data.start = SERVER_START;
    Device_confirm_Data.cmd = SERVER_ID_CMD_RSP;
    Device_confirm_Data.version = CMD_VERSION;
    Device_confirm_Data.softWare_version = SW_VERSION;
    Device_confirm_Data.cmdlen = BigtoLittle16(report_to_ServerDataLen_D);
    Device_confirm_Data.msg_seq_num = BigtoLittle16(msg_seq_num - 1);
    
    tool_memcpy(Device_confirm_Data.IMEI_num, NB_Report_to_ServerData.IMEI_num, IMEI_NUM_LEN);    
    //传递数据总长
    ServerDataLenSize = Form_AT_Head(report_to_ServerDataLen_D-1);
    SumLen = AT_TEST_PORT_LEN + (report_to_ServerDataLen_D + CHECK_SUM_LEN)*2 + AT_TAIL_NUM;
    
    //添加命令执行结果字段数据
    uint8_t off_set_Len = 0;  
    off_set_Len = AT_TEST_PORT_LEN + ServerDataLenSize;
    
    crc = checkSum((uint8_t*)&Device_confirm_Data, sizeof(NB_Report_to_ServerData_t));
    crc = crc - SERVER_START;
    crc = crc + serviceCMD_Process_result;
    
    hex2String((uint8_t*)&Device_confirm_Data, sizeof(NB_Report_to_ServerData_t), (char*)at_cmd_data_p+off_set_Len);
    off_set_Len = off_set_Len + sizeof(NB_Report_to_ServerData_t)*2;
    //命令结果
    hex2String(&serviceCMD_Process_result, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + 2;
    //添加校验和
    hex2String(&crc, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + CHECK_SUM_LEN*2;
    
    //添加结束位
    uint8_t server_end;
    server_end = SERVER_START;
    hex2String(&server_end, 1, (char*)at_cmd_data_p + off_set_Len);
    off_set_Len = off_set_Len + 2;
    //添加"\r\n"
    tool_memcpy(at_cmd_data_p + off_set_Len, AT_END, AT_TAIL_NUM);
    
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)at_cmd_data_p, SumLen);
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)at_cmd_data_p, SumLen);
}


/*
* @fn      Form_AT_Head(uin8_t datalen)
* @brief   组成 at_cmd_data_p 上报AT指令帧头
* @param   
* @return  位数  
*/
uint8_t Form_AT_Head(uint8_t datalen)
{
    //uint8_t SumLen = 0;
    uint8_t ServerDataLenSize = 0;
    //uint8_t datalen_t = 0;
    
    //char* at_cmd_data_head = NULL;
    //datalen_t = datalen * 2;
    //总数据长度
    
    if(datalen > 100)
    {
        //三位数+","
        ServerDataLenSize = 4;
    }
    else if(datalen > 10)
    {
        //二位数+","
        ServerDataLenSize = 3;    
    }
    else if(datalen > 0)
    {
        //一位数+","
        ServerDataLenSize = 2;  
    }
    //SumLen = AT_TEST_PORT_LEN + ServerDataLenSize + datalen_t + CHECK_SUM_LEN*2 + AT_TAIL_NUM;
    //at_cmd_data_p = malloc(SumLen);
    
    tool_strcat(AT_HEARER_NUM, AT_START, NSOSTF, AT_PRAM_0, AT_PRAM_IP, AT_PRAM_PORT, AT_TxFlag_replyRelease);
    
    //tool_strcat(AT_HEARER_NUM, AT_START, NSOST, AT_PRAM_0, AT_PRAM_IP, AT_PRAM_PORT);
    //tool_memcpy(at_cmd_data_p, at_cmd_data_head, AT_TEST_PORT_LEN);
    //free(at_cmd_data_head);
    
    //例 AT+NSOST=0,218.244.145.31,8000,120,....\r\n
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
* @brief   解析服务器下发AT指令格式  
* @param   
* @return   
*/
void Handel_ServerAT_CMD(uint8_t* Message)
{
    uint8_t data_pos = 0;//数据所处位置
    uint8_t data_start_Flag = RESET;
    uint8_t nb_server_data_len = 0;
    //例：0,218.244.145.31,8000,10,0123456789,0
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
                //ip地址段
                if(data_pos == NB_IP)
                {
                    
                }
                //端口号段
                else if(data_pos == NB_PORT)
                {
                    
                }
                //服务器下发数据长度段
                else if(data_pos == NB_SERVER_DATA_LEN)
                {
                    uint8_t* RxMessageLen_p = NULL;
                    uint8_t Len;
                    char* str;
                    
                    RxMessageLen_p = RxMessage+1;
                    //获取 数据长度段 内容
                    str = strtok((char*)RxMessageLen_p, ",");
                    //复原被覆盖掉的','号
                    Len = strlen(str);
                    *(RxMessageLen_p + Len) = ',';
                    //转换为数值
                    nb_server_data_len = atoi(str);
                }
                //数据内容段
                else if(data_pos == NB_SERVER_DATA)
                {
                    Handel_ServerData(RxMessage, nb_server_data_len);
                    return;
                    //上报应答等
                }
                //是否仍有缓存字段
                else if(data_pos == NB_DATA_CACHE_FLAG)
                {
                    if(*RxMessage > 0)
                    {
                        //仍有字段未读出，发送 读取的 AT指令, 根据目前协议（94字节），不会超
                        //不做指令拼接
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
* @brief   处理服务器数据  —— 服务器-->终端
* @param   
* @return   
*/
void Handel_ServerData(uint8_t* RxMessage, uint16_t dataLen)
{
    //判断校验和
    //校验和 -- 从命令字到校验位之前的所有字节累加和取低位字节。

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
    //起始位FA
    if(Server_CMD_Data.start != SERVER_START)
    {
        //printf("DATA ERROR!");
        return;
    }
    
    //上报应答C0
    if(Server_CMD_Data.cmd == SERVER_ID_DATA_RSP)
    {
        printf("C0\r\n");
        //有数据
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
                //发送完成后清除GPS信息
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
//            //发送完成后清除GPS信息
//            if(currentMode == POSITIONING_MODE)
//            {
//                GPS_Info_Delete();
//            }
//            NB_min_counter = 0;
//            NB_open_Flag = RESET;
//            Close_NB();
//            
//
//#warning 添加0208
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
    //下发命令c1
    else if(Server_CMD_Data.cmd == SERVER_ID_CMD_DATA)
    {
        //获取命令段数据      
        if(dataLen > 0)
        {
            if(RegistFlag == RESET)
            {        
                User_StopClock(&DeviceRegistclock);
            }
            printf("C1\r\n");
            serviceCMD_Process_result = Handel_ServerData_Info_RSP( RxMessage+(Server_CMD_Data_Len)*2+1 , sizeof(Locator_pram_t));            
            //设备命令应答后，立即上报一帧当前状态的数据
            DataRspclock.timeOut = 1000;
            User_StartClock(&DataRspclock);
        }
    }
    //下发命令-- 进入Schedule模式配置C2
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
    //下发命令-- 主动要求设备同步时间戳C3
    else if(Server_CMD_Data.cmd == SERVER_ID_TIME_SYN)
    {
        
    }
    //下发命令-- 同步时间戳C4
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
//        //进入休眠
//        SleepStatus = SLEEP;
//    }
    //配置完后进入的模式
    if(Locator_pram.newMode > POSITIONING_MODE)
    {
        return 1;
    }
    else
    {
        currentMode = Locator_pram.newMode;
        Locator_Data.mode = currentMode;
    }
    //周期
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
//* @brief   处理服务器 上报应答 中的命令字段 
//* @param   
//* @return  
//*/
//Device_rsp_t Handel_Server_ctl_cmd( Server_ctl_cmd_t Server_ctl_cmd, uint8_t *Server_ctl_data)
//{      
//    //设置上报模式（0xC001）调试用
//    if(Server_ctl_cmd.cmdID == SERVER_ID_REPORT_MODE)
//    {
//        Device_rsp.cmd_ID = SERVER_ID_REPORT_MODE;
//    }
//    //查询版本号(0xC002)
//    else if(Server_ctl_cmd.cmdID ==  SERVER_ID_REPORT_REV)
//    {
//        Device_rsp.cmd_ID = SERVER_ID_REPORT_REV;
//    } 
//    //配置定位器命令（0xC106）
//    else if(Server_ctl_cmd.cmdID == SERVER_ID_LOCATOR_CTL)
//    {
//        Device_rsp.cmd_ID = SERVER_ID_LOCATOR_CTL;
//        //Device_rsp.rsp_data = malloc(Server_ctl_cmd.data_len);
//        //判断配置命令长度
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
//            //判断参数范围
//            if(Locator_pram.GPSCollectionTime > GPS_COLLECTION_TIME_MAX)
//            {
//                Locator_pram.GPSCollectionTime = GPS_COLLECTION_TIME_MAX;
//            }
//            if(Locator_pram.enterSleep == SET)
//            {
//                //进入休眠
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