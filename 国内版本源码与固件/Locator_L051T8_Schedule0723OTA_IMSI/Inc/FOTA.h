/*******************************************************************************
    Copyright:          Lierda WSN BT team
    Filename:           FOTA.h
    Description:        FOTA_H
    FileVersion:        

    ChangeLog: 

    =========================================
    date:2018/7/13
    author:Chenkl
    log: Create File


*******************************************************************************/

#ifndef FOTA_H
#define FOTA_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */
 
 
 
/*********************************************************************
 * MACROS
 */
#define APP_VERSION                                 "v1.0.180921"
    
#define EEPROM_FOTA_FLAG_OFFSET                     16
#define EEPROM_FOTA_CONTINUE_FLAG_OFFSET            17
#define EEPROM_FOTA_CMPLT_FLAG_OFFSET               18
#define EEPROM_FOTA_INFO_OFFSET                     32
    
/*********************************************************************
 * CONSTANTS
 */
enum
{
    QUERY_VERSION,          //平台下发版本号查询
    NEW_VERSION,            //平台通知有新版本固件
    REQ_BLOCK,              //请求升级包
    DOWNLOAD_CMPLT_RSP,     //下载完成后的状态回复
    JUMP_2_APP,             //跳转到APP
    FOTA_SUCCESS,             //升级成功后返回状态给平台
};
enum
{
    QUERY_VERSION_ID    = 0x13,
    NEW_VERSION_ID,            
    REQ_BLOCK_ID,              
    DOWNLOAD_CMPLT_RSP_ID,     
    JUMP_2_APP_ID,    
    FOTA_SUCCESS_ID,        
};
    
    
/*********************************************************************
 * TYPEDEFS
 */
typedef struct 
{
	uint16_t block_num;
	uint16_t block_size;
	uint16_t check;
    
	uint16_t block_current_seq;
    char ver_str[16];
    char new_ver_str[16];
}fota_info_t;

/*********************************************************************
*  EXTERNAL VARIABLES
*/
extern fota_info_t fota_info;
extern uint8_t fota_info_full;
extern uint8_t fota_continue;
/*********************************************************************
 * API FUNCTIONS
 */
void FOTA_Msg_Process(uint8_t msg_id, uint8_t *data, uint16_t data_len);
void FOTA_Version_Rsp();
void FOTA_Download_State_Rsp(uint8_t state);
void FOTA_Info_Save();
void FOTA_Info_Read();
void FOTA_Info_Clear();
uint8_t FOTA_New_Info_Check();
uint8_t FOTA_Task_Flag_Read();
void FOTA_Task_Flag_Write(uint8_t fota_flag);
uint8_t FOTA_Continue_Flag_Read();
void FOTA_Continue_Flag_Write(uint8_t flag);
void FOTA_Cmplt_Flag_Write(uint8_t flag);
void FOTA_Success_Rsp(uint8_t state);
void FOTA_New_Version_Rsp(uint8_t state);
uint8_t FOTA_Cmplt_Flag_Read();
/*********************************************************************
*********************************************************************/  

#ifdef __cplusplus
}
#endif

#endif /* FOTA_H */
