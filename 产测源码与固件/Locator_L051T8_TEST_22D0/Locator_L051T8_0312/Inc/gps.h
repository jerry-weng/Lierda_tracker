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

#define GPS_NUM_MAX      10          //��󱣴�10��GPS����  
     
#define GPS_ID_GPRMC     "GNRMC"     //Recommend Minimum Specific GNSS Data
#define GPS_ID_GPVGT     "GPVGT"     //Course Over Ground and Ground Speed
#define GPS_ID_GPGGA     "GPGGA"     //Global Position System Fixed Data
#define GPS_ID_GPGSA     "GPGSA"     //GNSS DOP and Active Satellites
#define GPS_ID_GPGSV     "GPGSV"     //GNSS Satellite in view
#define GPS_ID_GPGLL     "GPGLL"     //Geographic Position - Latitude/Longitude

//GPSģ���ϴ����ݸ�ʽ����
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
    
//���ǳ�ʱʱ������
#define GPS_SERCH_TIMEOUT_MAX              60000
     
//NB�ϱ�GPS���ݸ�ʽ����     
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
    uint8_t State;                               //�����Ƿ���Ч 
    uint8_t UTC_Time[GPS_UTC_LEN];               //UTCʱ��
    uint8_t Latitude[GPS_LATITUDE_LEN];          //γ��ddmm.mmmm
    uint8_t N_S_Indicator;                       //�ϱ�����ָʾ
    uint8_t Longitude[GPS_LONGITUDE_LEN];        //����dddmm.mmmm
    uint8_t W_E_Indicator;                       //��������ָʾ
    uint8_t Date_Value[GPS_DATE_LEN];            //����
}GPS_Data_t;

typedef struct
{
    uint8_t mode;                                //��ǰģʽ0x00 -- Findnow 0x01 -- Track
    uint8_t Reserve[2];                          //Ԥ��
    uint8_t State;                               //�����Ƿ���Ч  01��Ч��00��Ч
    uint8_t Longitude[NB_LONGITUDE_LEN];         //����dddmm.mmm
    uint8_t Latitude[NB_LATITUDE_LEN];           //γ��ddmm.mmmm
    uint8_t EWNS_Indicator;                      //�����ϱ�����ָʾ
    uint8_t UTC_Time[NB_UTC_LEN];                //UTCʱ��
    uint8_t Date_Value[NB_DATE_LEN];             //����
}GPS_TO_NB_Data_t;

typedef struct
{
    uint8_t             serial_num;               //���
    GPS_TO_NB_Data_t    GPS_TO_NB_Data;           //NB�ϴ���GPS����
}
GPS_List_t;

typedef struct
{
    uint8_t             overflow_Flag;            //�����־λ          
    uint8_t             stores_num;               //��ǰ�洢����
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