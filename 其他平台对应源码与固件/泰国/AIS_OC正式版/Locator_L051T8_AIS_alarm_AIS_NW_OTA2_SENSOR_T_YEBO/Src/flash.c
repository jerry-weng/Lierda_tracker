/*******************************************************************************
** ��Ȩ:		    ������Ƽ����޹�˾
** �ļ���: 		  uart.c
** �汾��  	    V1.00   
** ��������: 	  RealView MDK-ARM V5.11
** ����: 		    ydc
** ����:		    Flash����						
** ����: 	      2014-08-12
** �޸���־��	  2014-01-15
                  1���������ע��
** ��Ȩ����     (C)2014-2015 ������Ƽ����޹�˾
*******************************************************************************/

#include "flash.h"
#include "stm32l0xx_hal.h"

void EEPROM_Init(void);
void EEPROM_WriteOneByte(uint32_t add, uint8_t data);
uint8_t EEPROM_ReadOneByte(uint32_t add);
void EEPROM_WritePage(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte);
void EEPROM_WriteBytes(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte);
void EEPROM_ReadBytes(uint32_t add, uint8_t * p_rbuf, uint32_t nbyte);

/*******************************************************************************
** ����	      FLASH��ʼ��
** ����       ��
** ����ֵ	    ��
** ע��       
** �޸���־
*******************************************************************************/
void FLASH_Init(void)
{

}

/*******************************************************************************
** ����	      EEPROM��ʼ��
** ����       ��
** ����ֵ	    ��
** ע��       �˺���Ϊ��������EEPROM��д��Ϊ�պ���
** �޸���־
*******************************************************************************/
void EEPROM_Init(void)
{
    
}

/*******************************************************************************
** ����	      ��EEPROMдһ�ֽ�����
** ����       add:�洢����ַ,��0��ʼ
              data:Ҫд��������ֽ�
** ����ֵ	    ��
** ע��       
** �޸���־
*******************************************************************************/
void EEPROM_WriteOneByte(uint32_t add, uint8_t data)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();//����EEPROM
    add += EEPROM_BASE_ADDR;//����EEPROMʵ�ʵ�ַ
    if(add > EEPROM_END_ADDR)//��ַ������Χ
    {
        HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM���� 
        return;
    }
    if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, add, data) == HAL_OK)//��EEPROMдһ�ֽ�����
    {
        
    }
    HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM���� 
}

/*******************************************************************************
** ����	      ��EEPROM��һ�ֽ�����
** ����       add:�洢����ַ,��0��ʼ
** ����ֵ	    ����������
** ע��       
** �޸���־
*******************************************************************************/
uint8_t EEPROM_ReadOneByte(uint32_t add)
{
    return *(__IO uint8_t*)(EEPROM_BASE_ADDR+add);
}

/*******************************************************************************
** ����	      ��EEPROMдһҳ����
** ����       add:�洢����ַ,��0��ʼ
              p_wbuf:Ҫд������ݻ���ָ��
              nbyte��Ҫд�������������λ���ֽ�
** ����ֵ	    ��
** ע��       �˺���Ϊ��������EEPRO��д����ҳ��ַ�����ҳ��С����
** �޸���־
*******************************************************************************/
void EEPROM_WritePage(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();//����EEPROM
    add += EEPROM_BASE_ADDR;//����EEPROMʵ�ʵ�ַ
    /*  ���EEPROM */
    while(nbyte>0)
    {
        if(add > EEPROM_END_ADDR)//��ַ������Χ
        {
            HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM����  
            return;
        }
        if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, add, *p_wbuf) == HAL_OK)//��EEPROMдһ�ֽ�����
        {
            
        }  
        add++;
        p_wbuf++;
        nbyte--;
    }
    HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM���� 
}

/*******************************************************************************
** ����	      ��EEPROMд�������
** ����       add:�洢����ַ,��0��ʼ
              p_wbuf:Ҫд������ݻ���ָ��
              nbyte��Ҫд�������������λ���ֽ�
** ����ֵ	    ��
** ע��       
** �޸���־
*******************************************************************************/
void EEPROM_WriteBytes(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte)
{
    HAL_FLASHEx_DATAEEPROM_Unlock();//����EEPROM
    add += EEPROM_BASE_ADDR;//����EEPROMʵ�ʵ�ַ
    /*  ���EEPROM */
    while(nbyte>0)
    {
        if(add > EEPROM_END_ADDR)//��ַ������Χ
        {
            HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM����  
            return;
        }
        if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, add, *p_wbuf) == HAL_OK)//��EEPROMдһ�ֽ�����
        {
            
        }  
        add++;
        p_wbuf++;
        nbyte--;
    }
    HAL_FLASHEx_DATAEEPROM_Lock();//EEPROM���� 
}

/*******************************************************************************
** ����	      ��EEPROM��ȡ�������
** ����       add:�洢����ַ,��0��ʼ
              p_rbuf:��ȡ�������ݴ�ŵĻ���ָ��
              nbyte��Ҫ��ȡ������������λ���ֽ�
** ����ֵ	    ��
** ע��       
** �޸���־
*******************************************************************************/
void EEPROM_ReadBytes(uint32_t add, uint8_t * p_rbuf, uint32_t nbyte)
{
    add += EEPROM_BASE_ADDR;//����EEPROMʵ�ʵ�ַ    
    while(nbyte>0)
    {
        *p_rbuf = *(__IO uint8_t*)add;
        add++;
        p_rbuf++;
        nbyte--;
    }
}
