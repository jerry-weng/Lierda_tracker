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
 *  @file       ExtFlash.h
 *
 *  @brief      External flash storage abstraction.
 *
 *  ============================================================================
 */
#ifndef __LSD_MEMS_H__
#define __LSD_MEMS_H__

#include <stdlib.h>
#include <stdbool.h>
#include "lis3dh.h"
   
extern void Lsd_MEMS_Init( void );

extern void Lsd_MEMS_SetNormal_Mode( LIS3DH_ODR_t odr, LIS3DH_Fullscale_t scale );

extern void Lsd_MEMS_SetWakeup_Mode( LIS3DH_ODR_t odr, uint8_t highres, LIS3DH_Fullscale_t scale, uint16_t thresh );

extern void Lsd_MEMS_SetFifo_Mode( LIS3DH_ODR_t odr, uint8_t highres, LIS3DH_Fullscale_t scale, uint8_t overThresh );

extern void Lsd_MEMS_PowerDown( void );

extern void Lsd_MEMS_Diable( void );

extern bool MEMS_SPI_ReadAddr(uint8_t addr, uint8_t *data);

extern bool MEMS_SPI_WriteAddr(uint8_t addr, uint8_t data);


extern bool MEMS_I2C_ReadAddr(uint8_t addr, uint8_t *data);

extern bool MEMS_I2C_WriteAddr(uint8_t addr, uint8_t data);


extern void Lsd_MEMS_AxisDisable( void );
uint8_t Lis3dhWakeUpMode(void);
#ifdef __cplusplus
extern "C"
{
#endif
  
#ifdef __cplusplus
}
#endif

#endif //__LSD_MEMS_H__