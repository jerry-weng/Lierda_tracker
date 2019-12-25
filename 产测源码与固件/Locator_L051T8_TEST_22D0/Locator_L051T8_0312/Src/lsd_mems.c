/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       lsd_mems.c
 *
 *  @brief      External flash storage implementation.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
//#include "Board.h"
#include "lsd_mems.h"
//#include "spi/lsd_spi.h"
#include "i2c.h"
#include "lis3dh.h"

//static PIN_Config BoardMemsPinTable[] =
//{
//  Board_SPI_MEMS_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MIN,
//  PIN_TERMINATE
//};
//
//static PIN_Handle hMemsPin = NULL;
//static PIN_State memsPinState;
#define I2C_READ_ADDRESS          0x33
#define I2C_WRITE_ADDRESS         0x32
                                        
 
static bool memsIsOpen = false;

void Lsd_MEMS_Init( void )
{
#define MEMS_DEV_ID     0x33
  uint8_t devId = 0;
  
  //Lsd_spi_Open();
  
  LIS3DH_GetWHO_AM_I( &devId );
  
  if( MEMS_DEV_ID != devId )
  {
    memsIsOpen = false;
    //Lsd_spi_Close();
    return;
  }
  LIS3DH_SetBoot();
  LIS3DH_SetMode(LIS3DH_POWER_DOWN);
  memsIsOpen = true;
  
  //Lsd_spi_Close();
}

void Lsd_MEMS_SetNormal_Mode( LIS3DH_ODR_t odr, LIS3DH_Fullscale_t scale )
{
  if( !memsIsOpen )
  {
    return;
  }
  //Lsd_spi_Open();//open spi
  //Reset all reister
  LIS3DH_SetBoot();
  //Set output data rate( sample rate ).
  LIS3DH_SetODR( odr );
  //Set Power mode as normal mode
  LIS3DH_SetMode( LIS3DH_NORMAL );
  //Set Full Scale
  LIS3DH_SetFullScale( scale );
  //Enable Axis
  LIS3DH_SetAxis( LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE );
  //Set interrupt source and interrupt pin
  LIS3DH_SetInt1Pin( LIS3DH_I1_INT1_ON_PIN_INT1_DISABLE );
  
  //Lsd_spi_Open();//close spi
}
/**
 * @fn          Lis3dhWakeUpMode
 * @brief       set to wakeup mode, enable high pass filter
 * @params      null
 * @return      MEMS_SUCCESS / MEMS_ERROR
 */
uint8_t Lis3dhWakeUpMode(void)
{ 
    // open SPI
    //Lsd_spi_Open();
        
    uint8_t ReSta = MEMS_SUCCESS;
   
    LIS3DH_SetBoot();
    
    // LIS3DH_CTRL_REG1
    ReSta &= LIS3DH_SetODR( LIS3DH_ODR_25Hz );  //sampling rate
    ReSta &= LIS3DH_SetMode( LIS3DH_LOW_POWER ); //low power mode
    ReSta &= LIS3DH_SetAxis( LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE );//XYZ enable
    // LIS3DH_CTRL_REG2
    ReSta &= LIS3DH_SetHPFMode( LIS3DH_HPM_NORMAL_MODE_RES );//high pass filter
    ReSta &= LIS3DH_HPFAOI1Enable( MEMS_ENABLE );//enable high pass filter on AOI_INT1 
    //ReSta &= LIS3DH_SetFilterDataSel(MEMS_ENABLE); // enable filter output on OUT_XYZ register(or FIFO)
    ReSta &= LIS3DH_SetHPFCutOFF( LIS3DH_HPFCF_0 );//set cutoff frequency, reffer to LIS3DH_AN 4.1 Tbale8
    // LIS3DH_CTRL_REG3
    ReSta &= LIS3DH_SetInt1Pin( LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE );//interrupt on INT1
    // LIS3DH_CTRL_REG4
    ReSta &= LIS3DH_SetFullScale( LIS3DH_FULLSCALE_8 );//scale
    ReSta &= LIS3DH_SetBDU( MEMS_ENABLE );//block update
    // LIS3DH_CTRL _REG5
    ReSta &= LIS3DH_Int1LatchEnable( MEMS_DISABLE );
    // LIS3DH_CTRL_REG6  CTRL_REG6 bit1 , for configuring active level on INT1,2
    //ReSta &= LIS3DH_SetInt2Pin( LIS3DH_INT_ACTIVE_LOW );
    // LIS3DH_INT1_THS
    ReSta &= LIS3DH_SetInt1Threshold( 4 );// 0.156g
    // LIS3DH_INT1_DURATION
    ReSta &= LIS3DH_SetInt1Duration( 2 ); // duration time when trigger interrupt,[0,127], unit: 1/50Hz
    // LIS3DH_INT1_CFG                  //AOI:6D=10b  when overload on XYZ, trigger an interrupt
    ReSta &= LIS3DH_SetIntConfiguration(LIS3DH_INT1_OR|
                                        LIS3DH_INT1_ZLIE_ENABLE|
                                        LIS3DH_INT1_YLIE_ENABLE|
                                        LIS3DH_INT1_XLIE_ENABLE);
    // Close SPI
    //Lsd_spi_Close();
    
    return ReSta;
}

void Lsd_MEMS_SetWakeup_Mode( LIS3DH_ODR_t odr, uint8_t highres, LIS3DH_Fullscale_t scale, uint16_t thresh )
{
  if( !memsIsOpen )
    return;
  
  //Lsd_spi_Open();
  
  LIS3DH_SetBoot();
  //Set output data rate( sample rate ).
  LIS3DH_SetODR( (LIS3DH_ODR_t)odr );
  //Set Power mode, resolution relative
  if( !highres )
  {
    LIS3DH_SetMode( LIS3DH_LOW_POWER );
  }
  else
  {
    LIS3DH_SetMode( LIS3DH_NORMAL );
  }
  //Enable Axis
  LIS3DH_SetAxis( LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE );
  
  //Config high pass filter mode as reference mode, reference level in REFERENCE register(26h)
  LIS3DH_SetHPFMode( LIS3DH_HPM_NORMAL_MODE_RES );
  //Enable high pass filter 
  LIS3DH_HPFAOI1Enable( MEMS_ENABLE );
  //Set cutoff frequency, ODR/(50*2^n) Hz
  LIS3DH_SetHPFCutOFF( LIS3DH_HPFCF_0 );
  
  //Select interrupt source and interrupt pin
  LIS3DH_SetInt1Pin( LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE );
  //Set Full scale
  LIS3DH_SetFullScale( (LIS3DH_Fullscale_t)scale );
  //Set Block data update
  LIS3DH_SetBDU( MEMS_ENABLE );
  
  //Lach interrupt 
  LIS3DH_Int1LatchEnable( MEMS_DISABLE );
  //Wakeup thresh n*(scale/128)=thresh

  LIS3DH_SetInt1Threshold( thresh );
//  LIS3DH_SetInt1Threshold( thresh*128/(1000*(2<<scale))+1 );// 4*8*1000/128=250mg
  //interrupt pulse width n*1000/odr ms
  LIS3DH_SetInt1Duration( 2 );
  //AOI:6D=10b  when overload on XYZ, trigger an interrupt
  LIS3DH_SetIntConfiguration(LIS3DH_INT1_OR|
                             LIS3DH_INT1_ZLIE_ENABLE|
                             LIS3DH_INT1_YLIE_ENABLE|
                             LIS3DH_INT1_XLIE_ENABLE);
  //Lsd_spi_Close();
}

void Lsd_MEMS_SetFifo_Mode( LIS3DH_ODR_t odr, uint8_t highres, LIS3DH_Fullscale_t scale, uint8_t overThresh )
{
  uint8_t reg;
  AxesRaw_t axe;
  
  if( !memsIsOpen )
    return;
  
  // open SPI
  //Lsd_spi_Open();
  // reset all reg
  LIS3DH_SetBoot();
  
  // LIS3DH_CTRL_REG1
  LIS3DH_SetODR( odr );  //sampling rate
  if( !highres )
  {
    LIS3DH_SetMode( LIS3DH_LOW_POWER );
  }
  else
  {
    LIS3DH_SetMode( LIS3DH_NORMAL );
  }
  LIS3DH_SetAxis( LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE );//XYZ enable
  
  // LIS3DH_CTRL_REG2: high-pass filter disabled
  LIS3DH_SetHPFMode(LIS3DH_HPM_NORMAL_MODE_RES);      //high pass filter
  LIS3DH_HPFAOI1Enable( MEMS_DISABLE );               //disable high pass filter on AOI_INT1 
  
  // LIS3DH_CTRL_REG3 : set interrupt on INT1
  LIS3DH_SetInt1Pin( LIS3DH_WTM_ON_INT1_ENABLE );// watermark interrupt on INT1 
  
  // LIS3DH_CTRL_REG4: set full scale
  LIS3DH_SetFullScale( scale );//scale
  
  // LIS3DH_CTRL_REG5: disable fifo first
  LIS3DH_FIFOModeEnable(LIS3DH_FIFO_DISABLE);
  
  // Set Watermark
  LIS3DH_SetWaterMark(28);
  
  // FIFO_CTRL_REG: enable stream mode and enable fifo
  LIS3DH_FIFOModeEnable(LIS3DH_FIFO_STREAM_MODE);
  
  // Clear old data of the FIFO
  LIS3DH_GetFifoSourceFSS(&reg);
  
  while(reg>0)
  {
    LIS3DH_GetAccAxesRaw(&axe);
    LIS3DH_GetFifoSourceFSS(&reg);
  }
  
  //Lsd_spi_Close();
}

void Lsd_MEMS_PowerDown( void )
{
  //Lsd_spi_Open();
  LIS3DH_SetMode(LIS3DH_POWER_DOWN);
  //Lsd_spi_Close();
}


//static void MEMS_Select(void)
//{
//  if( hMemsPin == NULL )
//  {
//    hMemsPin = PIN_open( &memsPinState, BoardMemsPinTable );
//  }
//  PIN_setOutputValue( hMemsPin, Board_SPI_MEMS_CS, 0 );
//}
//
///**
//* @fn          MEMS_Deselect
//* @brief       disable spi sensor
//* @param       NULL
//* @return      NULL
//*/
//static void MEMS_Deselect(void)
//{
//  if( hMemsPin == NULL )
//  {
//    hMemsPin = PIN_open( &memsPinState, BoardMemsPinTable );
//  }
//  PIN_setOutputValue(hMemsPin,Board_SPI_MEMS_CS, 1);
//}
//
//
//void Lsd_MEMS_Diable( void )
//{
//  MEMS_Deselect();
//}

///**
//* @fn          SpiReadAddr
//* @brief       read data from an addr
//* @param       addr - spi addr
//* @param       *data - pointer to data
//* @return      bool true/false
//*/
//bool MEMS_SPI_ReadAddr(uint8_t addr, uint8_t *data)
//{
//  // enable spi CS
//  MEMS_Select();
//  
//  // write an address to SPI
//  int ret = Lsd_spi_Write(&addr, sizeof(addr));
//  if (ret != 0)
//  {
//    /* failure */
//    // disable SPI CS
//    MEMS_Deselect();
//    
//    // close SPI
//    //Lsd_spi_Close();
//    return false;
//  }
//  
//  // read 1 byte data
//  ret = Lsd_spi_Read(data, sizeof(uint8_t));
//  // disable SPI CS
//  MEMS_Deselect();
//  
//  if (ret != 0)
//  {
//    /* failure */
//    return false;
//  }
//  else
//  {
//    /* success */
//    return true;
//  }
//}
//
///**
//* @fn          SpiWriteAddr
//* @brief       read data from an addr
//* @param       addr - spi addr
//* @param       data - data
//* @return      bool true/false
//*/
//bool MEMS_SPI_WriteAddr(uint8_t addr, uint8_t data)
//{
//  // enable spi CS
//  MEMS_Select();
//  
//  // write an address to SPI
//  int ret = Lsd_spi_Write(&addr, sizeof(addr));
//  if (ret != 0)
//  {
//    // disable SPI CS
//    MEMS_Deselect();
//    
//    // close SPI
//    //Lsd_spi_Close();
//    return false;
//  }
//  
//  // read 1 byte data
//  ret = Lsd_spi_Write(&data, sizeof(uint8_t));
//  // disable SPI CS
//  MEMS_Deselect();
//  
//  if (ret != 0)
//  {
//    /* failure */
//    return false;
//  }
//  else
//  {
//    /* success */
//    return true;
//  }
//}


bool MEMS_I2C_ReadAddr(uint8_t addr, uint8_t *data)
{
    if(HAL_I2C_Mem_Read(&hi2c1, I2C_READ_ADDRESS,  addr, 1, data, 1, 0x10) != HAL_OK)
    {
        return false;
    }
//    //写入地址 SAD + W
//    if(!= HAL_OK)
//    {
//        return false;
//    }
//    //写入地址 SUB
//    if(!= HAL_OK)
//    {
//        return false;
//    }
//    //读取SAD + R
//    if(!= HAL_OK)
//    {
//        return false;
//    }
    return true;
}


bool MEMS_I2C_WriteAddr(uint8_t addr, uint8_t data)
{
    if(HAL_I2C_Mem_Write(&hi2c1, I2C_WRITE_ADDRESS,  addr, 1, &data, 1, 0x10) != HAL_OK)
    {
        return false;
    }
    return true; 
}












