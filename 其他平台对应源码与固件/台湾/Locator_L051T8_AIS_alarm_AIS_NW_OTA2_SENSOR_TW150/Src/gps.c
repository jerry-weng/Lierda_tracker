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
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l0xx.h"
#include "stm32l0xx_hal.h"
#include "gps.h"
#include "tools.h"
#include "usart.h"
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
GPS_TO_NB_Data_t nb_gps_data = {0};
GPS_TO_NB_Data_t GPS_TO_NB_TimeOut_Data = {0};

//GPS_Info ��Ϣ������main������
GPS_Info_t GPS_Info = {0};
GPS_Info_t GPS_Info_schedule = {0};

//GSV��Ϣ
GPS_GSV_Data_t GPS_GSV_Data;

GPS_GSV_Data_t* GPS_TO_NB_GSV_Data = NULL;

uint8_t eff_satCounter = 0;

uint8_t *gpsRxMessage = NULL;
/*********************************************************************
* User Timer API
*/

/*
* @fn      Open_GPS
* @brief   IO�ڿ���GPS��
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
* @brief   �洢GPS����
* @param   GPS_TO_NB_Data_t* GPS_TO_NB_Data
* @return  
*/
void GPS_Info_Store(GPS_TO_NB_Data_t* GPS_TO_NB_Data)
{
    uint8_t stores_num = 0;
    //��scheduleģʽ
    if( Schedule_ModeFlag == RESET)
    {
        stores_num = GPS_Info.stores_num;
        //����GPS����
        tool_memcpy(&(GPS_Info.GPS_List[stores_num].GPS_TO_NB_Data), GPS_TO_NB_Data, sizeof(GPS_TO_NB_Data_t));
        //���浱ǰ�������к�
        GPS_Info.GPS_List[stores_num].serial_num = stores_num;

        //��ǰ�б��д洢��GPS����++��ѭ������
        GPS_Info.stores_num = (GPS_Info.stores_num + 1)%GPS_NUM_MAX;
    }
    
    //scheduleģʽ
    else
    {
        stores_num = GPS_Info_schedule.stores_num;
        //����GPS����
        tool_memcpy(&(GPS_Info_schedule.GPS_List[stores_num].GPS_TO_NB_Data), GPS_TO_NB_Data, sizeof(GPS_TO_NB_Data_t));
        //���浱ǰ�������к�
        GPS_Info_schedule.GPS_List[stores_num].serial_num = stores_num;

        //��ǰ�б��д洢��GPS����++��ѭ������
        GPS_Info_schedule.stores_num = (GPS_Info_schedule.stores_num + 1)%GPS_NUM_MAX;
    }
}

/*
* @fn      GPS_Info_Delete
* @brief   ɾ��GPS����
* @param   
* @return  
*/
void GPS_Info_Delete(void)
{
    memset(&GPS_Info, 0, sizeof(GPS_Info_t));
    memset(&GPS_Info_schedule, 0, sizeof(GPS_Info_t));
}

/*
* @fn      GPS_Rx_Data_process
* @brief   ����GPS��������
* @param   uint8_t *aRxMessage
* @return  NB��Ҫ�ϴ���GPS������Ϣ; 
*/
GPS_TO_NB_Data_t* GPS_Rx_Data_process(uint8_t *aRxMessage)
{
    uint8_t *gps_data_p = NULL;
    //uint8_t *gps_tmp_p = NULL;
    uint8_t *gps_bbc_p = NULL;
    uint8_t data_pos = 0;//��������λ��   
    GPS_Data_t gps_data = {0};
    
    //�ж�GPS���ݸ�ʽ��
    //��;$GPRMC,023121.000,A,3016.6686,N,11959.2590,E,0.89,248.51,220817,,,A*69
    //gps_data_p = malloc(150);
    //gps_tmp_p = gps_data_p;
    nb_gps_data.mode = currentMode;
    gps_data_p = (uint8_t *)mystrstr((const char *)aRxMessage,(const char *)GPS_ID_GPRMC, RX_LEN);
    gps_bbc_p = gps_data_p;
    if(gps_data_p != NULL)
    {
        uint8_t hex = 0;
        uint8_t bbc = 0;
        uint16_t rmcLen = 0;
        uint8_t rmcDataCompleteFlag = RESET;
        //�ж�gps���ݺϷ���
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
                printf("bbc_ERROR\r\n");
                return &GPS_TO_NB_TimeOut_Data;
            }
        }
        else
        {
            printf("Len_ERROR\r\n");
            return &GPS_TO_NB_TimeOut_Data;
        }
        
        data_pos = GPS_ID;
        while(*gps_data_p != '*')
        {
            if(*gps_data_p == ',')
            {
                data_pos++;
                //UTCʱ��
                if(data_pos == GPS_UTC)
                {
                    tool_memcpy(gps_data.UTC_Time, gps_data_p+1, GPS_UTC_LEN);
                    //����UTC����
                    ascsToHexs((char*)gps_data.UTC_Time, nb_gps_data.UTC_Time, GPS_UTC_LEN-4);
                }
                //GPS״̬
                else if(data_pos == GPS_STATUS)
                {
                    tool_memcpy(&gps_data.State, gps_data_p+1, GPS_DATA_STATE_LEN);
                    //A������Ч��V������Ч
                    if(gps_data.State == DATA_STATE_AVILIABLE)
                    {
                        nb_gps_data.State = INFO_STATE_AVILIABLE;
                    }
                    else if(gps_data.State == DATA_STATE_UNAVILIABLE)
                    {
                        nb_gps_data.State = INFO_STATE_UNAVILIABLE;
                    }
                }
                //γ��
                else if(data_pos == GPS_LATI)
                {
                    tool_memcpy(gps_data.Latitude, gps_data_p+1, GPS_LATITUDE_LEN);
                    //������.����, ddmm.mmmm
                    ascsToHexs((char*)gps_data.Latitude, nb_gps_data.Latitude, GPS_LATITUDE_LEN_H);
                    ascsToHexs((char*)gps_data.Latitude+GPS_LATITUDE_LEN_H+1, nb_gps_data.Latitude+NB_LATITUDE_LEN_H, GPS_LATITUDE_LEN_L);   
                }
                //�ϱ�ָʾ
                else if(data_pos == GPS_N_S)
                {
                    tool_memcpy(&gps_data.N_S_Indicator, gps_data_p+1, GPS_EWNS_IND_LEN);
                    //Bit1=1��������Bit1=0���ϰ���
                    if(gps_data.N_S_Indicator == N_INDECATOR)
                    {
                        SET_BIT(nb_gps_data.EWNS_Indicator, N_INDECATOR_BIT); 
                    }
                    else if(gps_data.N_S_Indicator == S_INDECATOR)
                    {
                        CLEAR_BIT(nb_gps_data.EWNS_Indicator, N_INDECATOR_BIT);
                    }
                }
                //����
                else if(data_pos == GPS_LONG)
                {
                    tool_memcpy(gps_data.Longitude, gps_data_p+1, GPS_LONGITUDE_LEN);
                    //������.���� dddmm.mmmm
                    tool_memcpy(gps_data.Longitude+GPS_LONGITUDE_LEN_H, gps_data.Longitude+GPS_LONGITUDE_LEN_H+1, GPS_LONGITUDE_LEN_L);
                    //�ü������һ���ֽ�
                    ascsToHexs((char*)gps_data.Longitude, nb_gps_data.Longitude, GPS_LONGITUDE_LEN);
                }
                //����ָʾ
                else if(data_pos == GPS_W_E)
                {
                    tool_memcpy(&gps_data.W_E_Indicator, gps_data_p+1, GPS_EWNS_IND_LEN);
                     //Bit0=1��������Bit0=0��������
                    if(gps_data.W_E_Indicator == W_INDECATOR)
                    {
                        SET_BIT(nb_gps_data.EWNS_Indicator, W_INDECATOR_BIT); 
                    }
                    else if(gps_data.W_E_Indicator == E_INDECATOR)
                    {
                        CLEAR_BIT(nb_gps_data.EWNS_Indicator, W_INDECATOR_BIT);
                    }
                }
                //�����ٶ�
                else if(data_pos == GPS_SPEED)
                {
                    
                }
                //�Եط�λ��   
                else if(data_pos == GPS_ANGLE)
                {
                    
                }
                //����
                else if(data_pos == GPS_DATE)
                {
                    tool_memcpy(gps_data.Date_Value, gps_data_p+1, GPS_DATE_LEN);
                    ascsToHexs((char*)gps_data.Date_Value, nb_gps_data.Date_Value, GPS_DATE_LEN);
                }
                //��ƫ��    
                else if(data_pos == GPS_MAGNETIC)
                {
                    
                }
                //����ָʾ
                else if(data_pos == GPS_W_E_INS)
                {
                    
                }
                //ģʽ
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



/*
* @fn      GPS_GSV_Data_process
* @brief   ����GPS�ź�ǿ������
* @param   uint8_t *aRxMessage
* @return  �ź�ǿ����30���ϵ���������������������Ч���� 
*/
GPS_GSV_Data_t* GPS_GSV_Data_process(uint8_t *aRxMessage)
{
    uint8_t *gps_data_p = NULL;
    uint8_t *gps_bbc_p = NULL;
    uint8_t data_pos = 0;//��������λ��   
    uint8_t sat_pos = 0;//������Ϣ����λ��
    uint8_t sat_num = 0;//������Ϣ�������
    uint8_t gsv_counter = 0;
    uint8_t gsv_end_Flag = RESET; //gsv������־λ
    
    GPS_GSV_Info_t GPS_GSV_Info = {0};
    
    //�ж�GSV���ݸ�ʽ��
    //��;$GPGSV,2,1,05,10,39,230,30,15,43,084,34,16,28,265,27,26,16,237,*72
    //$GPGSV,2,2,05,27,35,308,28*4E
    
    gps_data_p = (uint8_t *)mystrstr((const char *)aRxMessage,(const char *)GPS_ID_GPGSV, RX_LEN);
    gps_bbc_p = gps_data_p;
    if(gps_data_p != NULL)
    {
        uint8_t hex = 0;
        uint8_t bbc = 0;
        uint16_t rmcLen = 0;
        uint8_t rmcDataCompleteFlag = RESET;
        //�ж�gps���ݺϷ���
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
                GPS_GSV_Data.state = ERROR;
                printf("bbc_ERROR\r\n");
                return &GPS_GSV_Data;
            }
        }
        else
        {
            GPS_GSV_Data.state = ERROR;
            printf("Len_ERROR\r\n");
            return &GPS_GSV_Data;
        }
        
        data_pos = GSV_SUM;
        while( gsv_end_Flag == RESET )
        {
            while(*gps_data_p != '*' )
            {
                if(*gps_data_p == ',')
                {
                    //GSV�������
                    if(data_pos == GSV_SUM)
                    {
                        GPS_GSV_Info.gsv_sum = charsToHex1((char*)gps_data_p+1);
                        //ascsToHexs((char*)gps_data_p+1, &GPS_GSV_Info.gsv_sum, 1);
                        data_pos = GSV_NUM;
                    }
                    //GSV������
                    else if(data_pos == GSV_NUM)
                    {
                        GPS_GSV_Info.gsv_num = charsToHex1((char*)gps_data_p+1);
                        //ascsToHexs((char*)gps_data_p+1, &GPS_GSV_Info.gsv_num, 1);
                        data_pos = SAT_NUM;
                    }
                    //��������
                    else if(data_pos == SAT_NUM)
                    {
                        ascsToHexs((char*)gps_data_p+1, &GPS_GSV_Info.sat_num, 1);
                        GPS_GSV_Info.sat_num  = hex_to_dig(GPS_GSV_Info.sat_num);
                        data_pos = SAT_1;
                        
                        if(GPS_GSV_Info.sat_num == 0 || GPS_GSV_Info.sat_num > 19)
                        {
                            GPS_GSV_Data.state = ERROR;
                            return &GPS_GSV_Data;
                        }
                    }
                    //������Ϣ
                    else if(data_pos == SAT_1)
                    {
                        sat_pos++;
                        if(sat_pos == 1)
                        {
                            ascsToHexs((char*)gps_data_p+1, &GPS_GSV_Info.GPS_SAT_Info[sat_num].sat_1, 1);
                        }
                        else if(sat_pos == 2)
                        {
                            if(*(gps_data_p+1) != ',')
                            {
                                ascsToHexs((char*)gps_data_p+1, &GPS_GSV_Info.GPS_SAT_Info[sat_num].sat_2, 1);
                            }
                        }
                        else if(sat_pos == 3)
                        {
                            if(*(gps_data_p+1) != ',')
                            {
                                GPS_GSV_Info.GPS_SAT_Info[sat_num].sat_3[0] = charsToHex1((char*)gps_data_p+1);
                                ascsToHexs((char*)gps_data_p+2, GPS_GSV_Info.GPS_SAT_Info[sat_num].sat_3+1, 1);
                            }
                        }
                        else if(sat_pos == 4)
                        {
                            if(*(gps_data_p+1) != ',' && *(gps_data_p+1) != '*')
                            {
                                ascsToHexs((char*)gps_data_p+1, &GPS_GSV_Info.GPS_SAT_Info[sat_num].sat_snr, 1);
                            }
                            sat_pos = 0;
                            sat_num++;
                            
                            //gsv������
                            if(GPS_GSV_Info.gsv_num == GPS_GSV_Info.gsv_sum)
                            {
                                uint8_t satCounter = 0;
                                gsv_end_Flag = SET;
                                GPS_GSV_Data.state = SUCCESS;
                                for(uint8_t i=0; i<GPS_GSV_Info.gsv_sum ;i++)
                                {
                                    if(GPS_GSV_Info.GPS_SAT_Info[i].sat_snr >= 0x30)
                                    {
                                        satCounter++;
                                    }
                                    GPS_GSV_Data.SNR30up_satellites_Num = satCounter;
                                }
                                return &GPS_GSV_Data;
                            }
                        }
                    }
                }
                gps_data_p++;
            }
            data_pos = GSV_SUM;
            gps_data_p++;
            gsv_counter++;
            //��ֹ����
            if(gsv_counter > 50)
            {
                gsv_counter = 0;
                return &GPS_GSV_Data;
            }
        }
    }
    return &GPS_GSV_Data;
}