/*******************************************************************************
    Copyright:       Lierda WSN BT team
    Filename:        FOTA.c
    Description:     
    FileVersion:     

    ChangeLog: 

    =========================================
    date:2018/7/13
    author:Chenkl
    printf: Create File
 
*******************************************************************************/


/*******************************************************************************
* INCLUDES
*/
//#include "main.h"
//#include "boot.h"
#include "stm32l0xx_hal.h"

//#include "util.h"
//#include "uart_process.h"
//#include "timer_process.h"
//#include "NB_ctrl.h"
#include "User_ClockDriver.h"
#include "FOTA.h"
#include "crc16.h"
#include "flash.h"
#include "usart.h"
#include "nb.h"
/*********************************************************************
* TYPEDEFS
*/



/*********************************************************************
* MACROS
*/



/*********************************************************************
* LOCAL FUNCTIONS
*/
void FOTA_New_Version_Process(uint8_t *buf, uint16_t len);
uint8_t FOTA_Full_Img_Check();
uint8_t FOTA_New_Info_Check();
/*********************************************************************
* PUBLIC VARIABLES 
*/
uint8_t fota_continue = 0;
fota_info_t fota_info = {0};
uint8_t fota_info_full = 0;

uint8_t fota_state = QUERY_VERSION;
uint8_t fota_cmd_buf[50];
uint8_t NB_send_buf[150];
uint16_t NB_send_buf_len;
/*********************************************************************
* LOCAL VARIABLES
*/



/***********************`**********************************************
* PUBLIC FUNCTIONS
*/
void NB_Struct_Upload_Cmd(uint8_t *pVal, uint16_t len)
{
 const char HEX[] = "0123456789ABCDEF";

 memset(NB_send_buf, 0, sizeof(NB_send_buf));
 NB_send_buf_len = 0;
 sprintf((char*)NB_send_buf, "AT+MLWULDATAEX=%d,", len);

 uint16_t index = strlen((const char*)NB_send_buf);
    for( uint16_t i=0; i<len; i++ )
    {
     NB_send_buf[index++] = HEX[pVal[i]>>4];
     NB_send_buf[index++] = HEX[pVal[i]&0x0F];
    }
    sprintf((char*)NB_send_buf+index, ",0x0100\r\n");
    NB_send_buf_len = strlen((char*)NB_send_buf);
    printf("ZZZ\r\n");
    //HAL_UART_Transmit_DMA(&huart2, (uint8_t *)NB_send_buf, NB_send_buf_len);
    HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)NB_send_buf, NB_send_buf_len);
}


void FOTA_Msg_Process(uint8_t msg_id, uint8_t *data, uint16_t data_len)
{
//    uint8_t state = (uint8_t)HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, (GPIO_PinState)!state);
    switch(msg_id)
    {
        case QUERY_VERSION_ID:
        {
            FOTA_Version_Rsp();
        }break;
        case NEW_VERSION_ID:
        {
            FOTA_New_Version_Process(data, data_len);
        }break;
        case FOTA_SUCCESS_ID:
        {
            FOTA_Cmplt_Flag = 0;
            FOTA_Cmplt_Flag_Write(0);
            User_StopClock(&ATTimeoutclock);
            //下一步查询网络状态
            curAT_RSP_ID = AT_NUESTATS;
            NB_AT_CMD_Send(AT_NUESTATS);
            //TODO 停止重发升级已成功的定时器
        }

    }
}
void FOTA_Version_Rsp()
{
    uint8_t idx = 0;
    fota_cmd_buf[idx++] = 0xFF;
    fota_cmd_buf[idx++] = 0xFE;
    fota_cmd_buf[idx++] = 0x01;
    fota_cmd_buf[idx++] = QUERY_VERSION_ID;
    fota_cmd_buf[idx++] = 0x00;//CRC16校验高字节 offset=4
    fota_cmd_buf[idx++] = 0x00;//CRC16校验低字节
    
    fota_cmd_buf[idx++] = 0x00;//数据长度高字节
    fota_cmd_buf[idx++] = 0x11;//数据长度低字节 版本号数据17个字节=1(state)+16(verison string)
    
    fota_cmd_buf[idx++] = 0x00;//state
//    memcpy(fota_cmd_buf+idx , "V0.0", strlen("V0.0"));
    memcpy(fota_cmd_buf+idx , fota_info.ver_str, strlen(fota_info.ver_str));
    idx += 16;
    
    uint16_t crc16 = 0;
    crc16 = crc_check(fota_cmd_buf, idx);
    
    fota_cmd_buf[4] = crc16/256;//CRC16校验高字节 offset=4
    fota_cmd_buf[5] = crc16%256;//CRC16校验低字节
    
    NB_Struct_Upload_Cmd(fota_cmd_buf, idx);
    //NB_Auto_Upload();
    
}
void FOTA_New_Version_Process(uint8_t *buf, uint16_t len)
{
    memcpy(fota_info.new_ver_str , buf, 16);
    
    uint8_t idx = 16;
    fota_info.block_size = buf[idx]*256 + buf[idx+1];
    idx += 2;
    fota_info.block_num = buf[idx]*256 + buf[idx+1];
    idx += 2;
//    fota_info.check = buf[idx]*256 + buf[idx+1];
//    idx += 2;
    
    printf("new version info:%s\n    size:%d\nnum:%d\n   check:0x%04X\n", fota_info.new_ver_str, fota_info.block_size, fota_info.block_num, fota_info.check);
    
    uint8_t ret = FOTA_New_Info_Check();
    if(!ret)
    {
        if(memcmp(fota_info.new_ver_str, APP_VERSION, strlen(APP_VERSION)))
        {
            FOTA_Info_Save();
            FOTA_Continue_Flag_Write(1);
            FOTA_Task_Flag_Write(1);
            NVIC_SystemReset();
        }
        else
        {
            FOTA_New_Version_Rsp(3);//返回已经是最新版本
        }
    }

}
void FOTA_New_Version_Rsp(uint8_t state)
{
    uint8_t idx = 0;
    fota_cmd_buf[idx++] = 0xFF;
    fota_cmd_buf[idx++] = 0xFE;
    fota_cmd_buf[idx++] = 0x01;
    fota_cmd_buf[idx++] = NEW_VERSION_ID;
    fota_cmd_buf[idx++] = 0x00;//CRC16校验高字节 offset=4
    fota_cmd_buf[idx++] = 0x00;//CRC16校验低字节
    
    fota_cmd_buf[idx++] = 0x00;//数据长度高字节
    fota_cmd_buf[idx++] = 0x01;//数据长度低字节 版本号数据17个字节=1(state)
    
    fota_cmd_buf[idx++] = state;//state
    
    uint16_t crc16 = 0;
    crc16 = crc_check(fota_cmd_buf, idx);
    
    fota_cmd_buf[4] = crc16/256;//CRC16校验高字节 offset=4
    fota_cmd_buf[5] = crc16%256;//CRC16校验低字节
    
    NB_Struct_Upload_Cmd(fota_cmd_buf, idx);
    //NB_Auto_Upload();
}


void FOTA_Download_State_Rsp(uint8_t state)
{
    if(state == 6)
    {
        //fota_stop_flag = 1;
    }
    uint8_t idx = 0;
    fota_cmd_buf[idx++] = 0xFF;
    fota_cmd_buf[idx++] = 0xFE;
    fota_cmd_buf[idx++] = 0x01;
    fota_cmd_buf[idx++] = DOWNLOAD_CMPLT_RSP_ID;
    fota_cmd_buf[idx++] = 0x00;//CRC16校验高字节 offset=4
    fota_cmd_buf[idx++] = 0x00;//CRC16校验低字节
    
    fota_cmd_buf[idx++] = 0x00;//数据长度高字节
    fota_cmd_buf[idx++] = 0x01;//数据长度低字节
    
    fota_cmd_buf[idx++] = state;//请求包序低字节
    
    uint16_t crc16 = 0;
    crc16 = crc_check(fota_cmd_buf, idx);
    
    fota_cmd_buf[4] = crc16/256;//CRC16校验高字节 offset=4
    fota_cmd_buf[5] = crc16%256;//CRC16校验低字节
    
    NB_Struct_Upload_Cmd(fota_cmd_buf, idx);
    //NB_Auto_Upload();
}
void FOTA_Success_Rsp(uint8_t state)
{
    uint8_t idx = 0;
    fota_cmd_buf[idx++] = 0xFF;
    fota_cmd_buf[idx++] = 0xFE;
    fota_cmd_buf[idx++] = 0x01;
    fota_cmd_buf[idx++] = FOTA_SUCCESS_ID;
    fota_cmd_buf[idx++] = 0x00;//CRC16校验高字节 offset=4
    fota_cmd_buf[idx++] = 0x00;//CRC16校验低字节
    
    fota_cmd_buf[idx++] = 0x00;//数据长度高字节
    fota_cmd_buf[idx++] = 17;//数据长度低字节
    
    fota_cmd_buf[idx++] = state;//请求包序低字节
    
    memcpy(fota_cmd_buf+idx, fota_info.ver_str, 16);
    idx += 16;
    
    uint16_t crc16 = 0;
    crc16 = crc_check(fota_cmd_buf, idx);
    
    fota_cmd_buf[4] = crc16/256;//CRC16校验高字节 offset=4
    fota_cmd_buf[5] = crc16%256;//CRC16校验低字节
    
    NB_Struct_Upload_Cmd(fota_cmd_buf, idx);
    //NB_Auto_Upload();
}

uint8_t FOTA_New_Info_Check()
{
    uint8_t ret = 0;
    if(fota_info.block_num == 0)
    {
        printf("fota_info.block_num error\n");
        ret = 1;
    }
    if(fota_info.block_size == 0)
    {
        printf("fota_info.block_size error\n");
        ret = 2;
    }
//    if(fota_info.check == 0)
//    {
//        printf("fota_info.check error\n");
//        ret = 3;
//    }
    if(!memcmp(fota_info.new_ver_str, "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0", 16))
    {
        printf("fota_info.new_ver_star error\n");
        ret = 4;
    }
    return ret; 
}
void FOTA_Info_Save()
{
    EEPROM_WriteBytes(EEPROM_FOTA_INFO_OFFSET, (uint8_t*)&fota_info, sizeof(fota_info));
}
void FOTA_Info_Clear()
{
    fota_info.block_num = 0;
    fota_info.block_size = 0;
    fota_info.check = 0;
    fota_info.block_current_seq = 0;
    memset(fota_info.new_ver_str, 0, 16);
    EEPROM_WriteBytes(EEPROM_FOTA_INFO_OFFSET, (uint8_t*)&fota_info, sizeof(fota_info));
}
void FOTA_Info_Read()
{
    EEPROM_ReadBytes(EEPROM_FOTA_INFO_OFFSET, (uint8_t*)&fota_info, sizeof(fota_info));
}
uint8_t FOTA_Task_Flag_Read()
{
    uint8_t fota_flag = 0;
    EEPROM_ReadBytes(EEPROM_FOTA_FLAG_OFFSET, &fota_flag, 1);
    return fota_flag;
}
void FOTA_Task_Flag_Write(uint8_t fota_flag)
{
    EEPROM_WriteBytes(EEPROM_FOTA_FLAG_OFFSET, &fota_flag, 1);
}

uint8_t FOTA_Continue_Flag_Read()
{
    uint8_t flag = 0;
    EEPROM_ReadBytes(EEPROM_FOTA_CONTINUE_FLAG_OFFSET, &flag, 1);
    return flag;
}
void FOTA_Continue_Flag_Write(uint8_t flag)
{
    EEPROM_WriteBytes(EEPROM_FOTA_CONTINUE_FLAG_OFFSET, &flag, 1);
}
void FOTA_Cmplt_Flag_Write(uint8_t flag)
{
    EEPROM_WriteBytes(EEPROM_FOTA_CMPLT_FLAG_OFFSET, &flag, 1);
}

uint8_t FOTA_Cmplt_Flag_Read()
{
    uint8_t flag = 0;
    EEPROM_ReadBytes(EEPROM_FOTA_CMPLT_FLAG_OFFSET, &flag, 1);
    return flag;
}
/*********************************************************************
*********************************************************************/




