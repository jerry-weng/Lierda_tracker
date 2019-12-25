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
     
#define SERVER_ID_REPORT_REV_LEN        50                 //��Ӳ���汾��+NB�汾�ų���
#define SERVER_ID_REPORT_MODE_LEN       2                  //֧�ֵĳ���
#define SERVER_ID_REPORT_MODE_NUM       1                  //Ŀǰ����Ŀ��֧�ֶ�λ������
     
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

#define AT_START         "AT+"              //ATָ�ʼ��־  
#define AT_END           "\r\n"             //ATָ�������־
#define AT_COAP_END      ",0x0100\r\n"             //ATָ�������־     
     
#define AT_START_END_LEN                5
#define AT_CMDID_LEN_3                  3
#define AT_CMDID_LEN_4                  4
#define AT_CMDID_LEN_5                  5
#define AT_CMDID_LEN_7                  7
#define AT_CMDID_LEN_8                  8     
#define AT_CMDID_PRAM_LEN               2

//����ģʽ
//#define CONFIG_MODE                         0x03
#define NORMAL_MODE                         0x00
#define POSITIONING_MODE                    0x01
#define REGIST_MODE                         0x02
#define SCHEDULE_MODE                       0x02

//ATָ��ID
#define CFUN             "CFUN"             //����Э��ջ����
#define NCDP             "NCDP"             //IOTƽ̨��ַ
#define CGSN             "CGSN"             //��ȡIMEI��
#define CIMI             "CIMI"             //��ѯSIM����Ϣ
#define NRB              "NRB"              //������
#define CMEE             "CMEE"             //����������ʾ
#define CGDCONT          "CGDCONT"          //����APNģʽ--һ��Ϊ����
#define NNMI             "NNMI"             //������������֪ͨ
#define CGATT            "CGATT"            //�Զ�����
#define CSCON            "CSCON"            //���û�վ����֪ͨ
#define CGPADDR          "CGPADDR"          //��ѯ�����������IP��ַ
#define NUESTATS         "NUESTATS"         //��ѯ����״̬
#define NSOCR            "NSOCR"            //����SOCKET
#define NSOST            "NSOST"            //���������������
     
#define NSOSTF           "NSOSTF"           //���������������--�·�ʽ     
#define NSORF            "NSORF"            //��ȡ�������·�����
#define CGMR             "CGMR"             //��ȡ�ͺŰ汾�� 
#define MLWULDATAEX      "MLWULDATAEX="      //coap��������
     
     
//Ĭ���ϱ�����
#define DEFAULT_F_CYCLE    1440
#define DEFAULT_T_CYCLE    1440     
     
//���ȴ��Ļظ�ID
enum
{
    AT_CGMR,         //��ȡ�汾��
    AT_NO,           //0��
    AT_WAIT,         //1�ȴ����ڻظ�
    AT_CFUN0,        //2�ر�Э��ջ
    AT_CGSN,         //3��ѯIMEI��
    AT_NCDP,         //4����IOTƽ̨��ַ
    AT_NRB,          //5������
    AT_NRB_RESULT,   //6��λ���
    AT_CFUN1,        //7����Э��ջ
    AT_CMEE,         //8����������ʾ
    AT_CGDCONT,      //9����APNΪ����ģʽ
    AT_NNMI,         //10������������֪ͨ
    AT_CGATT,        //11�Զ�����
    AT_CSCON,        //12���û�վ����֪ͨ
    AT_CGPADDR,      //13��ѯ�����������IP��ַ
    AT_NUESTATS,     //14��ѯ����״̬   
    AT_NSOCR,        //15����Socket
    AT_NSOST,        //16��������
    AT_NSONMI,       //17���ݽ���
    AT_NSORF,        //18��ȡ����������
    AT_ERROR,        //19�ط������������͸�����ʧ��
    AT_INIT_SUCCESS, //20�����ɹ�
    AT_DATA_RSP,     //�ϱ��ظ�
    AT_CLOSE_EDRX,   //�ر�EDRX
    AT_OPEN_PSM,     //����PSM
    AT_CLEAR_RESERVE, //�������
    AT_NUESTATS_NORMAL, //ƽʱ��ѯ����״̬
    AT_MLWULDATAEX,     //coap��������
    AT_COAP_ENTER_SLEEP, //coap��������
    AT_CLEAR_FREQ,       //���Ƶ��
    AT_CELL_RESELECT,    //С����ѡ 
    AT_STATE,
    AT_FOTA_REPORT,
    AT_NCCID,            //sim����
};     
        
#define AT_COMMA                        ","     
#define AT_PRAM_0                       "=0"
#define AT_PRAM_1                       "=1"

#define AT_IMEI_RSP_LEN                 8                       //IMEI�Żظ���ͷ����
#define AT_IMEI_RSP                     "\r\n+CGSN:"            //IMEI�Żظ���ͷ

#define AT_NRB_RSP_LEN                  8                       //��λ�ظ�����
#define AT_NRB_RSP                      "\r\nREBOOT"            //��λ�ظ�����
//#define AT_NRB_RESULT_RSP_LEN         13                     //��λ�������
#define AT_NRB_RESULT_RSP               "CAUSE"                 //"\r\nNeul \r\nOK\r\n"   //��λ���
#define AT_NETWORK_STATE_LEN            12                      //����״̬��ѯ�ظ�����
#define AT_NETWORK_STATE                "\r\n+CGPADDR:0"        //����״̬��ѯ�ظ�

#define AT_IOT_IP_LEN                   20                      //IP����
#define AT_IOT_IP                       "=203.172.115.67,5683"  //IOTƽ̨��ַ

//#define AT_IOT_IP_LEN                   20                      //IP����
//#define AT_IOT_IP                       "=117.60.157.137,5683"  //IOTƽ̨��ַ

#define AT_TEST_PORT_LEN                15     ////37                      //AT + IP+�˿ںų���   ----coap��ͷ����
#define AT_PRAM_IP                      ",35.182.40.152"        //������IP��ַ
#define AT_PRAM_PORT                    ",7400,"                //�˿ں�
#define AT_NSORF_LEN                    6                       //����������ȣ�=0,512��
#define AT_NSORF_NUM                    ",512"                  //�������������ݳ���
#define AT_RSP_OK_LEN                   6                       //���ڻظ�OK����
#define AT_RSP_OK                       "\r\nOK\r\n"            //���ڻظ�OK
#define AT_TxFlag_replyRelease          "0x400,"                //�������֮���յ��ظ��˳�PSM
#define AT_RSP_ERROR_LEN                9                       //���ڻظ�ERROR����
#define AT_RSP_ERROR                    "\r\nERROR\r\n"         //���ڻظ�ERROR
#define AT_COAP_SEND_OK_LEN             22                      //���ڻظ�coap���ݷ��ͳɹ�����
#define AT_COAP_SEND_OK                 "\r\n+MLWULDATASTATUS:4\r\n"  //���ڻظ�coap���ݷ��ͳɹ�
#define COAP_PROTOCOL_LEN               74                      //Э������ֶγ���
#define COAP_PROTOCOL_HALFLEN           37                      //Э������ֶγ���



#if defined  (HK_NB) 
    #define AT_CGDCONT_CTNET_LEN            17                      //�ֶγ���
    #define AT_CGDCONT_CTNET                ",\"IP\",\"nbiot.poc\""     //����APNΪ��۵���ģʽ
#else
    #define AT_CGDCONT_CTNET_LEN            13                      //�ֶγ���
    #define AT_CGDCONT_CTNET                ",\"IP\",\"ctnet\""     //����APNΪ����ģʽ
#endif

#define AT_PRAM_SOCKET_LEN              6
#define AT_PRAM_SOCKET                  "=DGRAM"                //����SOCKET
#define AT_COMMA_LEN                    1                       //���ų�
#define AT_COMMA                        ","                     //����


#define AT_PRAM_SOCKET_RSP_LEN          11                      //����SOCKET�ظ�����
#define AT_PRAM_SOCKET_RSP              "\r\n0\r\n\r\nOK\r\n"   //����SOCKET�ظ�
#define AT_PRAM_POART_LEN               10      
#define AT_PRAM_POART                   ",17" AT_PRAM_PORT "1"  //",17,7800,1"            //�˿ں�
#define AT_PRAM_NSONMI_LEN              9                       //���ݽ��ճ���
#define AT_PRAM_NSONMI                  "\r\n+NSONMI"           //���յ�����
#define AT_PRAM_NSONMI_RSP_OK           "+NSONMI:0"             //δ��ȡ�����������
#define AT_PRAM_NSONMI_RSP_OK2          ",0\r\n\r\nOK\r\n"      //��ȡ����������
#define AT_CME                          "+CME"  

#define AT_PRAM_STATION_CONECTE_LEN     12                      //��վ����״̬
#define AT_PRAM_STATION_CONECTED        "\r\n+CSCON:1\r\n"      //��վ���ӳɹ�
#define AT_PRAM_STATION_DISCONECTED     "\r\n+CSCON:0\r\n"      //��վ�Ͽ�����
#define AT_PRAM_SERVER_DATA_LEN         23
#define AT_PRAM_SERVER_DATA             "0" AT_PRAM_IP AT_PRAM_PORT//��"0,202.107.200.164,7800," //�������������·�

#define AT_PRAM_CME_159_LEN             19
#define AT_PRAM_CME_159                 "\r\n+CME ERROR: 159\r\n" //�������:��վæ

#define AT_PRAM_CME_4_LEN               17
#define AT_PRAM_CME_4                   "\r\n+CME ERROR: 4\r\n" //�������

#define AT_PRAM_ERROR_LEN               9
#define AT_PRAM_ERROR                   "\r\nERROR\r\n" //�������



#define AT_COAP_RSP_OK_LEN              18                   //coap����
#define AT_COAP_RSP_OK                  "+NNMI:8"            //coapָ����ȷ

#define AT_COAP_RX_HEAD_LEN             8
#define AT_COAP_HEAD_RX                 "\r\n+NNMI:"                 //coap�յ�����

#define AT_TIME_SYN                     "FAFA"
#define AT_FOTA                         ",FFFE"

//����״̬����
#define NUESTATES_NUM                   5                       //��Ϣ�ֶ�

#define AT_NUESTATS_RSPR_LEN            13
#define AT_NUESTATS_RSPR                "Signal power:"         //�ź�ǿ��
#define AT_NUESTATS_CELL_ID_LEN         8          
#define AT_NUESTATS_CELL_ID             "Cell ID:"              //С����
#define AT_NUESTATS_ECL_LEN             4
#define AT_NUESTATS_ECL                 "ECL:"                  //���ǵȼ�
#define AT_NUESTATS_SNR_LEN             4
#define AT_NUESTATS_SNR                 "SNR:"                  //�����
#define AT_NUESTATS_RSRQ_LEN            5
#define AT_NUESTATS_RSRQ                "RSRQ:"                 //�źŽ�������

#define AT_SERVER_DATA_LEN             0x01

#define SERVER_START                   0xFA   //��ʼ������λ
#define SERVER_ID_DATA_REPOET          0xD1   //�ն������ϱ�
#define SERVER_ID_DATA_RSP             0xC0   //�������ϱ�Ӧ��
#define SERVER_ID_CMD_DATA             0xC1   //�������·�����
#define SERVER_ID_CMD_RSP              0xD2   //�ն�����ȷ��

#define CMD_VERSION                    0x01  //����汾
#define SW_VERSION                     0x02   //����汾

#define SERVER_CMD_VERSION             0x30         
     
#define LOCATOR_ID                     0x07   //��λ��ID
          
#define LOCATOR_UNINITIALIZED          0x00     
#define LOCATOR_INITIALIZED            0x01
//�Ƿ������ϱ�
#define LOCATOR_TIMEOUT_REPORT         0x00     
#define LOCATOR_USER_REPORT            0x01

//�������ʱ��     
#define GPS_COLLECTION_TIME_MAX        60
//����ϱ�����
#define CYCLE_TIME_MAX                 1440
#define CYCLE_TIME_MIN                 5
#define NB_DATA_MAX                    512
//���ݽ���״̬
#define USER_BUFF_SIZE                 512
//GPS��������ʱ��
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

//���͸�����������������
enum
{
    LOCATOR_DATA_REPORT,    //��ʱ�㱨0xD1
    LOCATOR_CMD_CONFIRM,    //����ȷ��0xD2
};

#pragma  pack(1)
  
//�ϱ����ݸ�ʽ
typedef struct{
    uint8_t start;
    uint8_t cmd;
    uint8_t version;
    uint8_t softWare_version;
    uint16_t cmdlen;                      //��ʼλ������λ����������λ
    uint8_t  IMEI_num[IMEI_NUM_LEN];
    uint16_t  msg_seq_num;                //��Ϣ�����
    /*------------------�������-Locator_Data_t--------------------------*/    
}NB_Report_to_ServerData_t;

//������� - NB��λ������
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
    /*------------------�������-GPS_List_t--------------------------*/   
}Locator_Data_t;


//��λ������
typedef struct{
    uint8_t enterSleep;
    uint8_t newMode;
    uint16_t NB_dataCycle;
    uint16_t GPSCollectionCycle;
    //uint8_t GPSCollectionTime;
}Locator_pram_t;


//�ն�ȷ�� ���� ����+ ���...����
typedef struct{
    uint8_t  cmd_num;
    uint8_t* cmd_data;
}
Device_confirm_seq_t;

//�ն�ȷ�� ���� ���� ���� �������ݳ���+��������
typedef struct
{
    uint16_t  cmd_ID;
    uint8_t  cmd_state;
    uint16_t  rsp_data_len;
    uint8_t* rsp_data;
}
Device_rsp_t;

//�ն�����ע��
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

//������ע��ȷ��
typedef struct{
    uint8_t result_len;
    uint8_t result_data;     
}
Server_regist_confirm_t;

//����������C
typedef struct{
    uint8_t start;
    uint8_t cmd;
    uint16_t cmdlen;                 //��ʼλ������λ����������λ
    uint8_t  Reserve[3];             //Ԥ��
    uint16_t packetSeqNum;           //���к�
}ServerData_t;

typedef struct{
    uint16_t  Head;                    //FAB1
    uint8_t   Type;                    //�豸����
    uint8_t   IMEI[15];
    uint8_t   battery_Lv;              //��ص����ٷֱ�
    uint8_t   days;                    //��������
    uint16_t  RSPR;
    uint16_t  SINR;
    uint8_t   ECL;
    uint16_t  RSRQ;
    uint32_t  cellID;                  //С��ID
    uint16_t  Reserve;                 //��չ�ֽ�
    uint8_t   sos;
    uint8_t   state;
    uint16_t  data_len; 
}Coap_Haed_t;

#pragma  pack()    

extern Coap_Haed_t Coap_Haed;
extern uint8_t SleepStatus;
extern uint8_t RegistConfirmFlag;
extern uint8_t LED_Init_Blink_Finish;
extern Locator_pram_t Locator_pram;               //��λ������
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
//NB�������ϱ���ʽ
extern NB_Report_to_ServerData_t NB_Report_to_ServerData;
//NB�ͺ�
extern char NB_version[30];                         //�汾��
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

//extern char* at_cmd_data_p;                      //ATָ��
//extern uint8_t at_cmd_data_p[512];
extern void NB_AT_CMD_Send(uint8_t CmdID, ...);

#ifdef __cplusplus
 }
#endif
   
#endif