/*******************************************************************************
    Copyright:       Lierda WSN BT team
    Filename:        nb.h
    Description:     nb_Driver
    FileVersion:     V1.0

    ChangeLog: 

    =========================================
    date:17/8/24
    author:chenkl
    log: Create File
 
*******************************************************************************/
#ifndef NB_H
#define NB_H

#ifdef __cplusplus
 extern "C" {
#endif
     
#include "gps.h"
#include "User_ClockDriver.h"
     
#define SERVER_ID_REPORT_REV_LEN        50                 //软硬件版本号+NB版本号长度
#define SERVER_ID_REPORT_MODE_LEN       2                  //支持的场景
#define SERVER_ID_REPORT_MODE_NUM       1                  //目前该项目仅支持定位器功能
     
#define REV_LEN                         12      
#define SW_REV                          "1.1.20180409" 
#define HW_REV                          "1.1.20180409" 
     
#define UNIX_LEN                        8
#define AT_NO_PRAM_LEN                  3
#define AT_ONE_PRAM_LEN                 4
#define AT_TWO_PRAM_LEN                 5     
#define AT_NSOST_PRAM_LEN               5

#define IMEI_NUM_LEN                    15
#define IMSI_LEN                        15
#define ICCID_LEN                       15     

#define AWAKE                           0
#define SLEEP                           1

     
#define AT_HEARER_NUM                   2//6//5
#define AT_TAIL_NUM                     2
#define AT_COAP_TAIL_NUM                9 
     

#define CHECK_SUM_LEN                   1
#define END_NUM_LEN                     1
#define STR_CMP_TRUE                    0     

#define AT_START         "AT+"              //AT指令开始标志  
#define AT_END           "\r\n"             //AT指令结束标志
#define AT_COAP_END      ",0x0100\r\n"             //AT指令结束标志     
     
#define AT_START_END_LEN                5
#define AT_CMDID_LEN_3                  3
#define AT_CMDID_LEN_4                  4
#define AT_CMDID_LEN_5                  5
#define AT_CMDID_LEN_7                  7
#define AT_CMDID_LEN_8                  8     
#define AT_CMDID_PRAM_LEN               2

//工作模式
//#define CONFIG_MODE                         0x03
#define NORMAL_MODE                         0x00
#define POSITIONING_MODE                    0x01
#define REGIST_MODE                         0x02
#define SCHEDULE_MODE                       0x02

//AT指令ID
#define CFUN             "CFUN"             //开关协议栈命令
#define NCDP             "NCDP"             //IOT平台地址
#define CGSN             "CGSN"             //获取IMEI号
#define CIMI             "CIMI"             //查询SIM卡信息
#define NRB              "NRB"              //软重启
#define CMEE             "CMEE"             //开启错误提示
#define CGDCONT          "CGDCONT"          //设置APN模式--一般为电信
#define NNMI             "NNMI"             //开启下行数据通知
#define CGATT            "CGATT"            //自动搜网
#define CSCON            "CSCON"            //设置基站连接通知
#define CGPADDR          "CGPADDR"          //查询核心网分配的IP地址
#define NUESTATS         "NUESTATS"         //查询网络状态
#define NSOCR            "NSOCR"            //建立SOCKET
#define NSOST            "NSOST"            //向服务器发送数据
     
#define NSOSTF           "NSOSTF"           //向服务器发送数据--新方式     
#define NSORF            "NSORF"            //读取服务器下发数据
#define CGMR             "CGMR"             //读取型号版本号 
#define MLWULDATAEX      "MLWULDATAEX="      //coap发送数据
     
     
//默认上报周期
#define DEFAULT_F_CYCLE    1440
#define DEFAULT_T_CYCLE    1440     
     
//所等待的回复ID
enum
{
    AT_CGMR,         //读取版本号
    AT_NO,           //0无
    AT_WAIT,         //1等待串口回复
    AT_CFUN0,        //2关闭协议栈
    AT_CGSN,         //3查询IMEI号
    AT_NCDP,         //4设置IOT平台地址
    AT_NRB,          //5软重启
    AT_NRB_RESULT,   //6复位结果
    AT_CFUN1,        //7开启协议栈
    AT_CMEE,         //8开启错误提示
    AT_CGDCONT,      //9设置APN为电信模式
    AT_NNMI,         //10开启下行数据通知
    AT_CGATT,        //11自动搜网
    AT_CSCON,        //12设置基站连接通知
    AT_CGPADDR,      //13查询核心网分配的IP地址
    AT_NUESTATS,     //14查询网络状态   
    AT_NSOCR,        //15建立Socket
    AT_NSOST,        //16发送数据
    AT_NSONMI,       //17数据接收
    AT_NSORF,        //18读取服务器数据
    AT_ERROR,        //19重发次数到，发送该命令失败
    AT_INIT_SUCCESS, //20联网成功
    AT_DATA_RSP,     //上报回复
    AT_CLOSE_EDRX,   //关闭EDRX
    AT_OPEN_PSM,     //开启PSM
    AT_CLEAR_RESERVE, //清除缓存
    AT_NUESTATS_NORMAL, //平时查询网络状态
    AT_MLWULDATAEX,     //coap发送数据
    AT_COAP_ENTER_SLEEP, //coap进入休眠
    AT_CLEAR_FREQ,       //清除频点
    AT_CELL_RESELECT,    //小区重选 
    AT_STATE,
    AT_FOTA_REPORT,
    AT_NCCID,            //sim卡号
};     
        
#define AT_COMMA                        ","     
#define AT_PRAM_0                       "=0"
#define AT_PRAM_1                       "=1"

#define AT_IMEI_RSP_LEN                 8                       //IMEI号回复包头长度
#define AT_IMEI_RSP                     "\r\n+CGSN:"            //IMEI号回复包头

#define AT_NRB_RSP_LEN                  8                       //软复位回复长度
#define AT_NRB_RSP                      "\r\nREBOOT"            //软复位回复数据
//#define AT_NRB_RESULT_RSP_LEN         13                     //复位结果长度
#define AT_NRB_RESULT_RSP               "CAUSE"                 //"\r\nNeul \r\nOK\r\n"   //复位结果
#define AT_NETWORK_STATE_LEN            12                      //网络状态查询回复长度
#define AT_NETWORK_STATE                "\r\n+CGPADDR:0"        //网络状态查询回复

#define AT_IOT_IP_LEN                   20                      //IP长度
#define AT_IOT_IP                       "=203.172.115.67,5683"  //IOT平台地址

//#define AT_IOT_IP_LEN                   20                      //IP长度
//#define AT_IOT_IP                       "=117.60.157.137,5683"  //IOT平台地址

#define AT_TEST_PORT_LEN                15     ////37                      //AT + IP+端口号长度   ----coap包头长度
#define AT_PRAM_IP                      ",35.182.40.152"        //服务器IP地址
#define AT_PRAM_PORT                    ",7400,"                //端口号
#define AT_NSORF_LEN                    6                       //命令参数长度（=0,512）
#define AT_NSORF_NUM                    ",512"                  //服务器所发数据长度
#define AT_RSP_OK_LEN                   6                       //串口回复OK长度
#define AT_RSP_OK                       "\r\nOK\r\n"            //串口回复OK
#define AT_TxFlag_replyRelease          "0x400,"                //发送完成之后，收到回复退出PSM
#define AT_RSP_ERROR_LEN                9                       //串口回复ERROR长度
#define AT_RSP_ERROR                    "\r\nERROR\r\n"         //串口回复ERROR
#define AT_COAP_SEND_OK_LEN             22                      //串口回复coap数据发送成功长度
#define AT_COAP_SEND_OK                 "\r\n+MLWULDATASTATUS:4\r\n"  //串口回复coap数据发送成功
#define COAP_PROTOCOL_LEN               74                      //协议额外字段长度
#define COAP_PROTOCOL_HALFLEN           37                      //协议额外字段长度



#if defined  (HK_NB) 
    #define AT_CGDCONT_CTNET_LEN            17                      //字段长度
    #define AT_CGDCONT_CTNET                ",\"IP\",\"nbiot.poc\""     //设置APN为香港电信模式
#else
    #define AT_CGDCONT_CTNET_LEN            13                      //字段长度
    #define AT_CGDCONT_CTNET                ",\"IP\",\"ctnet\""     //设置APN为电信模式
#endif

#define AT_PRAM_SOCKET_LEN              6
#define AT_PRAM_SOCKET                  "=DGRAM"                //建立SOCKET
#define AT_COMMA_LEN                    1                       //逗号长
#define AT_COMMA                        ","                     //逗号


#define AT_PRAM_SOCKET_RSP_LEN          11                      //建立SOCKET回复长度
#define AT_PRAM_SOCKET_RSP              "\r\n0\r\n\r\nOK\r\n"   //建立SOCKET回复
#define AT_PRAM_POART_LEN               10      
#define AT_PRAM_POART                   ",17" AT_PRAM_PORT "1"  //",17,7800,1"            //端口号
#define AT_PRAM_NSONMI_LEN              9                       //数据接收长度
#define AT_PRAM_NSONMI                  "\r\n+NSONMI"           //接收到数据
#define AT_PRAM_NSONMI_RSP_OK           "+NSONMI:0"             //未读取完服务器数据
#define AT_PRAM_NSONMI_RSP_OK2          ",0\r\n\r\nOK\r\n"      //读取服务器数据
#define AT_CME                          "+CME"  

#define AT_PRAM_STATION_CONECTE_LEN     12                      //基站连接状态
#define AT_PRAM_STATION_CONECTED        "\r\n+CSCON:1\r\n"      //基站连接成功
#define AT_PRAM_STATION_DISCONECTED     "\r\n+CSCON:0\r\n"      //基站断开连接
#define AT_PRAM_SERVER_DATA_LEN         23
#define AT_PRAM_SERVER_DATA             "0" AT_PRAM_IP AT_PRAM_PORT//例"0,202.107.200.164,7800," //服务器有数据下发

#define AT_PRAM_CME_159_LEN             19
#define AT_PRAM_CME_159                 "\r\n+CME ERROR: 159\r\n" //命令错误:基站忙

#define AT_PRAM_CME_4_LEN               17
#define AT_PRAM_CME_4                   "\r\n+CME ERROR: 4\r\n" //命令错误

#define AT_PRAM_ERROR_LEN               9
#define AT_PRAM_ERROR                   "\r\nERROR\r\n" //命令错误



#define AT_COAP_RSP_OK_LEN              18                   //coap长度
#define AT_COAP_RSP_OK                  "+NNMI:8"            //coap指令正确

#define AT_COAP_RX_HEAD_LEN             8
#define AT_COAP_HEAD_RX                 "\r\n+NNMI:"                 //coap收到数据

#define AT_TIME_SYN                     "FAFA"
#define AT_FOTA                         ",FFFE"

//网络状态参数
#define NUESTATES_NUM                   5                       //信息字段

#define AT_NUESTATS_RSPR_LEN            13
#define AT_NUESTATS_RSPR                "Signal power:"         //信号强度
#define AT_NUESTATS_CELL_ID_LEN         8          
#define AT_NUESTATS_CELL_ID             "Cell ID:"              //小区号
#define AT_NUESTATS_ECL_LEN             4
#define AT_NUESTATS_ECL                 "ECL:"                  //覆盖等级
#define AT_NUESTATS_SNR_LEN             4
#define AT_NUESTATS_SNR                 "SNR:"                  //信噪比
#define AT_NUESTATS_RSRQ_LEN            5
#define AT_NUESTATS_RSRQ                "RSRQ:"                 //信号接收质量

#define AT_SERVER_DATA_LEN             0x01

#define SERVER_START                   0xFA   //起始、结束位
#define SERVER_ID_DATA_REPOET          0xD1   //终端数据上报
#define SERVER_ID_DATA_RSP             0xC0   //服务器上报应答
#define SERVER_ID_CMD_DATA             0xC1   //服务器下发命令
#define SERVER_ID_CMD_RSP              0xD2   //终端命令确认

#define CMD_VERSION                    0x01  //命令版本
#define SW_VERSION                     0x02   //软件版本

#define SERVER_CMD_VERSION             0x30         
     
#define LOCATOR_ID                     0x07   //定位器ID
          
#define LOCATOR_UNINITIALIZED          0x00     
#define LOCATOR_INITIALIZED            0x01
//是否主动上报
#define LOCATOR_TIMEOUT_REPORT         0x00     
#define LOCATOR_USER_REPORT            0x01

//最大搜星时间     
#define GPS_COLLECTION_TIME_MAX        60
//最大上报周期
#define CYCLE_TIME_MAX                 1440
#define CYCLE_TIME_MIN                 5
#define NB_DATA_MAX                    512
//数据接收状态
#define USER_BUFF_SIZE                 512
//GPS星历保存时间
#define GPS_KEEP_MIN                   120

#define SPECIAL_FINDNOW                2
#define MOVE_ALARM                     3
#define EXIT_MOVE_ALARM                4

enum
{
    NB_ID,
    NB_IP,
    NB_PORT,
    NB_SERVER_DATA_LEN,
    NB_SERVER_DATA,
    NB_DATA_CACHE_FLAG,
};

//发送给服务器的数据类型
enum
{
    LOCATOR_DATA_REPORT,    //定时汇报0xD1
    LOCATOR_CMD_CONFIRM,    //命令确认0xD2
};

#pragma  pack(1)
  
//上报数据格式
typedef struct{
    uint8_t start;
    uint8_t cmd;
    uint8_t version;
    uint8_t softWare_version;
    uint16_t cmdlen;                      //起始位到结束位，包含结束位
    uint8_t  IMEI_num[IMEI_NUM_LEN];
    uint16_t  msg_seq_num;                //消息包序号
    /*------------------填充数据-Locator_Data_t--------------------------*/    
}NB_Report_to_ServerData_t;

//填充数据 - NB定位器数据
typedef struct{
    uint8_t mode;
    uint16_t FindnowCycle;
    uint16_t TrackerCycle;
    uint8_t initFlag;
    uint8_t userAlarmFlag;
    uint8_t batteryLv;
    uint8_t avaliableTime;
    uint8_t sensorFlag;
    uint16_t GPS_data_len;
    uint8_t  GPS_List_num;
    /*------------------填充数据-GPS_List_t--------------------------*/   
}Locator_Data_t;


//定位器参数
typedef struct{
    uint8_t enterSleep;
    uint8_t newMode;
    uint16_t NB_dataCycle;
    uint16_t GPSCollectionCycle;
    //uint8_t GPSCollectionTime;
}Locator_pram_t;


//终端确认 —— 个数+ 序号...内容
typedef struct{
    uint8_t  cmd_num;
    uint8_t* cmd_data;
}
Device_confirm_seq_t;

//终端确认 —— 内容 —— 命令数据长度+命令数据
typedef struct
{
    uint16_t  cmd_ID;
    uint8_t  cmd_state;
    uint16_t  rsp_data_len;
    uint8_t* rsp_data;
}
Device_rsp_t;

//终端请求注册
typedef struct{
    uint16_t start;
    uint16_t cmd;
    uint8_t  version;
    uint16_t cmd_len;
    uint8_t IMEI_len;
    uint8_t IMEI_num[IMEI_NUM_LEN];
    uint8_t IMSI_len;
    //uint8_t IMSI_data[IMSI_LEN];
    uint8_t ICCID_len;
    //uint8_t ICCID_data[ICCID_LEN];                
}
Device_regist_Data_t;

//服务器注册确认
typedef struct{
    uint8_t result_len;
    uint8_t result_data;     
}
Server_regist_confirm_t;

//服务器数据C
typedef struct{
    uint8_t start;
    uint8_t cmd;
    uint16_t cmdlen;                 //起始位到结束位，包含结束位
    uint8_t  Reserve[3];             //预留
    uint16_t packetSeqNum;           //序列号
}ServerData_t;

typedef struct{
    uint16_t  Head;                    //FAB1
    uint8_t   Type;                    //设备类型
    uint8_t   IMEI[15];
    uint8_t   battery_Lv;              //电池电量百分比
    uint8_t   days;                    //可用天数
    uint16_t  RSPR;
    uint16_t  SINR;
    uint8_t   ECL;
    uint16_t  RSRQ;
    uint32_t  cellID;                  //小区ID
    uint16_t  Reserve;                 //扩展字节
    uint8_t   sos;
    uint8_t   state;
    uint16_t  data_len; 
}Coap_Haed_t;

#pragma  pack()    

extern Coap_Haed_t Coap_Haed;
extern uint8_t SleepStatus;
extern uint8_t RegistConfirmFlag;
extern uint8_t LED_Init_Blink_Finish;
extern Locator_pram_t Locator_pram;               //定位器参数
extern uint8_t curAT_RSP_ID;
extern uint16_t msg_seq_num;

extern uint8_t CMD_packageNum;
extern uint8_t packageFinishFlag;
extern uint8_t packageStartFlag;

extern uint8_t DataUpFlag;
extern uint8_t data_confirmFlag;
extern uint8_t after_registFlag;
extern uint8_t NB_min_counter;
extern uint8_t NB_StateSend_counter;
extern uint8_t TrackerCycle_short_Flag;
//extern uint8_t special_findnowFlag;
extern uint8_t NB_InNetWork_Flag;
extern uint8_t RegistFinishFlag;

extern uint8_t NB_open_Flag;
extern uint8_t gps_open_Flag;

extern uint8_t TxCompleteFlag;
extern uint16_t dataCycle;

extern uint8_t testFlag;
extern uint8_t testFlag2;
extern uint8_t testFlag3;
extern uint8_t testFlag4;

extern uint8_t ExtraDataupFlag;

extern uint8_t Retry_state;
extern uint8_t AT_Retry_Num;

extern uint8_t upload_OK_Flag;
extern uint8_t cmd_OK_Flag;
extern uint8_t Normal_StateFlag;
extern uint8_t state_OK_Flag;
extern uint8_t RegistStateFlag;
extern uint8_t FOTA_Cmplt_Flag;
extern uint8_t OTAingFlag;
extern uint8_t move_alarm_enable_Flag;
extern uint8_t move_alarm_Init_Flag;
extern uint8_t move_alarm_Flag;

extern uint8_t MoveAlarmPreMode;
//NB端数据上报格式
extern NB_Report_to_ServerData_t NB_Report_to_ServerData;
//NB型号
extern char NB_version[30];                         //版本号
extern uint8_t NB_version_len;

extern uint32_t wakeupCounter;
extern char coapEnterSleep[32];

extern uint8_t NB_Station_Connected_Flag;
extern uint8_t RegistFlag;
extern uint8_t RegistConfirmFlag;
extern uint8_t NB_Confirm_counter;
extern Clock_Struct NBReportclock;
extern Clock_Struct GPSCollectionclock;
extern Clock_Struct LEDclock;
extern Clock_Struct BoardCloseTimeoutclock;
extern Clock_Struct DataRspclock;
extern Clock_Struct InNetTimeoutclock;
extern Clock_Struct DeviceRegistclock;
extern Clock_Struct GPSTimeOutclock;
extern Clock_Struct ATTimeoutclock;
extern Clock_Struct DataRptclock;
extern Clock_Struct APPUnbindclock;


extern uint8_t *nbRxMessage;
extern Locator_Data_t Locator_Data;
extern uint8_t currentMode;
extern void NBReportclockProcess(void);
extern void GPSCollectionclockProcess(void);
extern uint8_t Form_AT_Head(uint16_t datalen);
extern void Handel_ServerAT_CMD(uint8_t* RxMessage);
extern void ScheduleParmSet(uint8_t CurrentDayInTheLoop);
extern void Handel_ServerAT_CMD(uint8_t* Message);
extern void Handel_coap_Data(uint8_t* RxMessage);
extern void Coap_EnterSleep(void);
extern void NB_Recv_Data_Process(uint8_t *buf, uint16_t len);

extern void Open_NB(void);
extern void Close_NB(void);

//extern char* at_cmd_data_p;                      //AT指令
//extern uint8_t at_cmd_data_p[512];
extern void NB_AT_CMD_Send(uint8_t CmdID, ...);

#ifdef __cplusplus
 }
#endif
   
#endif