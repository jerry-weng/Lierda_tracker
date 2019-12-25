/*******************************************************************************
Copyright:       Lierda WSN BT team
Filename:        GPS.c
Description:     GPS_Driver
FileVersion:     V1.0

ChangeLog: 

=========================================
date:17/8/21
author:chenkl
log: Create File

*******************************************************************************/


/*******************************************************************************
* INCLUDES
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "gps.h"
#include "tools.h"
#include "usart.h"
/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* MACROS
*/
/*********************************************************************
* PUBLIC VARIABLES 
*/
GPS_TO_NB_Data_t nb_gps_data = {0};
GPS_TO_NB_Data_t GPS_TO_NB_TimeOut_Data = {0};

//GPS_Info 信息传递至main函数内
GPS_Info_t GPS_Info = {0};
uint8_t *gpsRxMessage = NULL;
/*********************************************************************
* User Timer API
*/

/*
* @fn      Open_GPS
* @brief   IO口控制GPS开
* @param   GPIOxPort, GPIO_Pin
* @return   
*/
void Open_GPS(void)
{
    HAL_GPIO_WritePin(GPS_SW_GPIO_Port, GPS_SW_Pin, GPIO_PIN_RESET);
}

/*
* @fn      Close_GPS
* @brief   GPIOxPort, GPIO_Pin
* @param   
* @return   
*/
void Close_GPS(void)
{
    HAL_GPIO_WritePin(GPS_SW_GPIO_Port, GPS_SW_Pin, GPIO_PIN_SET);
}

/*
* @fn      GPS_Info_Store
* @brief   存储GPS数据
* @param   GPS_TO_NB_Data_t* GPS_TO_NB_Data
* @return  
*/
void GPS_Info_Store(GPS_TO_NB_Data_t* GPS_TO_NB_Data)
{
    uint8_t stores_num = 0;
    
    stores_num = GPS_Info.stores_num;
    //保存GPS数据
    tool_memcpy(&(GPS_Info.GPS_List[stores_num].GPS_TO_NB_Data), GPS_TO_NB_Data, sizeof(GPS_TO_NB_Data_t));
    //保存当前数据序列号
    GPS_Info.GPS_List[stores_num].serial_num = stores_num;
    //判断数据是否溢出
    if(GPS_Info.stores_num + 1 > GPS_NUM_MAX)
    {
        GPS_Info.overflow_Flag = SET;
    }
    //当前列表中存储的GPS数量++，循环覆盖
    GPS_Info.stores_num = (GPS_Info.stores_num + 1)%GPS_NUM_MAX;
}

/*
* @fn      GPS_Info_Delete
* @brief   删除GPS数据
* @param   
* @return  
*/
void GPS_Info_Delete(void)
{
    memset(&GPS_Info, 0, sizeof(GPS_Info_t));
}

/*
* @fn      GPS_Rx_Data_process
* @brief   解析GPS串口数据
* @param   uint8_t *aRxMessage
* @return  NB所要上传的GPS数据信息; 
*/
GPS_TO_NB_Data_t* GPS_Rx_Data_process(uint8_t *aRxMessage)
{
    uint8_t *gps_data_p = NULL;
    //uint8_t *gps_tmp_p = NULL;
    uint8_t *gps_bbc_p = NULL;
    uint8_t data_pos = 0;//数据所处位置   
    GPS_Data_t gps_data = {0};
    
    //判断GPS数据格式。
    //例;$GNRMC,023121.000,A,3016.6686,N,11959.2593,E,0.89,248.51,220817,,,A*69
    //gps_data_p = malloc(150);
    //gps_tmp_p = gps_data_p;
    gps_data_p = (uint8_t *)mystrstr((const char *)aRxMessage,(const char *)GPS_ID_GPRMC, RX_LEN);
    gps_bbc_p = gps_data_p;
    if(gps_data_p != NULL)
    {
        uint8_t hex = 0;
        uint8_t bbc = 0;
        uint16_t rmcLen = 0;
        uint8_t rmcDataCompleteFlag = RESET;
        //判断gps数据合法性
        rmcLen = (RX_LEN-(gps_data_p-aRxMessage));
        for( uint8_t gps_counter = 0; gps_counter < rmcLen-2 ; gps_counter++ )
        {
            if(*gps_bbc_p == '*')
            {
                rmcDataCompleteFlag = SET;
                break;
            }
            else
            {
                bbc = bbc ^ (*gps_bbc_p);
                gps_bbc_p++;
            }
        }
        if(rmcDataCompleteFlag == SET)
        {
            ascsToHexs((char*)gps_bbc_p+1, &hex, 2);
            if(hex != bbc)
            {
                //printf("bbc_ERROR\r\n");
                return &GPS_TO_NB_TimeOut_Data;
            }
        }
        else
        {
            //printf("Len_ERROR\r\n");
            return &GPS_TO_NB_TimeOut_Data;
        }
        
        data_pos = GPS_ID;
        while(*gps_data_p != '*')
        {
            if(*gps_data_p == ',')
            {
                data_pos++;
                //UTC时间
                if(data_pos == GPS_UTC)
                {
                    tool_memcpy(gps_data.UTC_Time, gps_data_p+1, GPS_UTC_LEN);
                    //削减UTC精度
                    ascsToHexs((char*)gps_data.UTC_Time, nb_gps_data.UTC_Time, GPS_UTC_LEN-4);
                }
                //GPS状态
                else if(data_pos == GPS_STATUS)
                {
                    tool_memcpy(&gps_data.State, gps_data_p+1, GPS_DATA_STATE_LEN);
                    //A数据有效，V数据无效
                    if(gps_data.State == DATA_STATE_AVILIABLE)
                    {
                        nb_gps_data.State = INFO_STATE_AVILIABLE;
                    }
                    else if(gps_data.State == DATA_STATE_UNAVILIABLE)
                    {
                        nb_gps_data.State = INFO_STATE_UNAVILIABLE;
                    }
                }
                //纬度
                else if(data_pos == GPS_LATI)
                {
                    tool_memcpy(gps_data.Latitude, gps_data_p+1, GPS_LATITUDE_LEN);
                    //跳过“.”号, ddmm.mmmm
                    ascsToHexs((char*)gps_data.Latitude, nb_gps_data.Latitude, GPS_LATITUDE_LEN_H);
                    ascsToHexs((char*)gps_data.Latitude+GPS_LATITUDE_LEN_H+1, nb_gps_data.Latitude+NB_LATITUDE_LEN_H, GPS_LATITUDE_LEN_L);   
                }
                //南北指示
                else if(data_pos == GPS_N_S)
                {
                    tool_memcpy(&gps_data.N_S_Indicator, gps_data_p+1, GPS_EWNS_IND_LEN);
                    //Bit1=1：北半球；Bit1=0：南半球；
                    if(gps_data.N_S_Indicator == N_INDECATOR)
                    {
                        SET_BIT(nb_gps_data.EWNS_Indicator, N_INDECATOR_BIT); 
                    }
                    else if(gps_data.N_S_Indicator == S_INDECATOR)
                    {
                        CLEAR_BIT(nb_gps_data.EWNS_Indicator, N_INDECATOR_BIT);
                    }
                }
                //经度
                else if(data_pos == GPS_LONG)
                {
                    tool_memcpy(gps_data.Longitude, gps_data_p+1, GPS_LONGITUDE_LEN);
                    //跳过“.”号 dddmm.mmmm
                    tool_memcpy(gps_data.Longitude+GPS_LONGITUDE_LEN_H, gps_data.Longitude+GPS_LONGITUDE_LEN_H+1, GPS_LONGITUDE_LEN_L);
                    //裁剪掉最后一个字节
                    ascsToHexs((char*)gps_data.Longitude, nb_gps_data.Longitude, GPS_LONGITUDE_LEN);
                }
                //东西指示
                else if(data_pos == GPS_W_E)
                {
                    tool_memcpy(&gps_data.W_E_Indicator, gps_data_p+1, GPS_EWNS_IND_LEN);
                     //Bit0=1：西半球；Bit0=0：东半球；
                    if(gps_data.W_E_Indicator == W_INDECATOR)
                    {
                        SET_BIT(nb_gps_data.EWNS_Indicator, W_INDECATOR_BIT); 
                    }
                    else if(gps_data.W_E_Indicator == E_INDECATOR)
                    {
                        CLEAR_BIT(nb_gps_data.EWNS_Indicator, W_INDECATOR_BIT);
                    }
                }
                //対地速度
                else if(data_pos == GPS_SPEED)
                {
                    
                }
                //对地方位角   
                else if(data_pos == GPS_ANGLE)
                {
                    
                }
                //日期
                else if(data_pos == GPS_DATE)
                {
                    tool_memcpy(gps_data.Date_Value, gps_data_p+1, GPS_DATE_LEN);
                    ascsToHexs((char*)gps_data.Date_Value, nb_gps_data.Date_Value, GPS_DATE_LEN);
                }
                //磁偏角    
                else if(data_pos == GPS_MAGNETIC)
                {
                    
                }
                //东西指示
                else if(data_pos == GPS_W_E_INS)
                {
                    
                }
                //模式
                else if(data_pos == GPS_MODE)
                {
                    
                }
            }
            gps_data_p++;            
        }
    }
    //free(gps_tmp_p);
    return &nb_gps_data;
}