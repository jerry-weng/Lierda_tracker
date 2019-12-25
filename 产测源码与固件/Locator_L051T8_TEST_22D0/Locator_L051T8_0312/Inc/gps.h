/*******************************************************************************
    Copyright:       Lierda WSN BT team
    Filename:        gps.h
    Description:     gps_Driver
    FileVersion:     V1.0

    ChangeLog: 

    =========================================
    date:17/8/21
    author:chenkl
    log: Create File
 
*******************************************************************************/
#ifndef GPS_H
#define GPS_H

#ifdef __cplusplus
 extern "C" {
#endif

#define GPS_NUM_MAX      10          //最大保存10组GPS数据  
     
#define GPS_ID_GPRMC     "GNRMC"     //Recommend Minimum Specific GNSS Data
#define GPS_ID_GPVGT     "GPVGT"     //Course Over Ground and Ground Speed
#define GPS_ID_GPGGA     "GPGGA"     //Global Position System Fixed Data
#define GPS_ID_GPGSA     "GPGSA"     //GNSS DOP and Active Satellites
#define GPS_ID_GPGSV     "GPGSV"     //GNSS Satellite in view
#define GPS_ID_GPGLL     "GPGLL"     //Geographic Position - Latitude/Longitude

//GPS模块上传数据格式长度
#define GPS_DATA_STATE_LEN          1
#define GPS_UTC_LEN                 10
#define GPS_LATITUDE_LEN            9
#define GPS_LATITUDE_LEN_H          4
#define GPS_LATITUDE_LEN_L          4
#define GPS_EWNS_IND_LEN            1
#define GPS_LONGITUDE_LEN           10
#define GPS_LONGITUDE_LEN_H         5
#define GPS_LONGITUDE_LEN_L         4
#define GPS_DATE_LEN                6
    
//搜星超时时间上限
#define GPS_SERCH_TIMEOUT_MAX              60000
     
//NB上报GPS数据格式长度     
#define NB_DATA_STATE_LEN          1
#define NB_LATITUDE_LEN            4
#define NB_LATITUDE_LEN_H          2     
#define NB_LONGITUDE_LEN           5
#define NB_DIR_LEN                 1
#define NB_UTC_LEN                 3
#define NB_DATE_LEN                3

#define DATA_STATE_AVILIABLE       0x41         //"A"
#define DATA_STATE_UNAVILIABLE     0x56         //"V"

#define E_INDECATOR                0x45         //"E"
#define W_INDECATOR                0x57         //"W"
#define S_INDECATOR                0x53         //"S"
#define N_INDECATOR                0x4E         //"N"
 
#define N_INDECATOR_BIT            0x02
#define W_INDECATOR_BIT            0x01
     
     
#define INFO_STATE_AVILIABLE       0x01
#define INFO_STATE_UNAVILIABLE     0x00     
     
enum
{
    GPS_ID,
    GPS_UTC,
    GPS_STATUS,
    GPS_LATI,
    GPS_N_S,
    GPS_LONG,
    GPS_W_E,
    GPS_SPEED,
    GPS_ANGLE,
    GPS_DATE,
    GPS_MAGNETIC,
    GPS_W_E_INS,
    GPS_MODE,
};

#pragma  pack(1)
typedef struct
{
    uint8_t State;                               //数据是否有效 
    uint8_t UTC_Time[GPS_UTC_LEN];               //UTC时间
    uint8_t Latitude[GPS_LATITUDE_LEN];          //纬度ddmm.mmmm
    uint8_t N_S_Indicator;                       //南北半球指示
    uint8_t Longitude[GPS_LONGITUDE_LEN];        //经度dddmm.mmmm
    uint8_t W_E_Indicator;                       //东西半球指示
    uint8_t Date_Value[GPS_DATE_LEN];            //日期
}GPS_Data_t;

typedef struct
{
    uint8_t mode;                                //当前模式0x00 -- Findnow 0x01 -- Track
    uint8_t Reserve[2];                          //预留
    uint8_t State;                               //数据是否有效  01有效，00无效
    uint8_t Longitude[NB_LONGITUDE_LEN];         //经度dddmm.mmm
    uint8_t Latitude[NB_LATITUDE_LEN];           //纬度ddmm.mmmm
    uint8_t EWNS_Indicator;                      //东西南北半球指示
    uint8_t UTC_Time[NB_UTC_LEN];                //UTC时间
    uint8_t Date_Value[NB_DATE_LEN];             //日期
}GPS_TO_NB_Data_t;

typedef struct
{
    uint8_t             serial_num;               //序号
    GPS_TO_NB_Data_t    GPS_TO_NB_Data;           //NB上传，GPS数据
}
GPS_List_t;

typedef struct
{
    uint8_t             overflow_Flag;            //溢出标志位          
    uint8_t             stores_num;               //当前存储数量
    GPS_List_t          GPS_List[GPS_NUM_MAX];
}
GPS_Info_t;
#pragma  pack()

extern void Open_GPS(void);
extern void Close_GPS(void);
extern  uint8_t *gpsRxMessage;
extern  GPS_Info_t GPS_Info;
extern  void GPS_Info_Store(GPS_TO_NB_Data_t* GPS_TO_NB_Data);
extern void GPS_Info_Delete(void);
extern  GPS_TO_NB_Data_t* GPS_Rx_Data_process(uint8_t *aRxMessage);
extern  GPS_TO_NB_Data_t GPS_TO_NB_TimeOut_Data;
extern  GPS_TO_NB_Data_t nb_gps_data;


#ifdef __cplusplus
 }
#endif
   
#endif