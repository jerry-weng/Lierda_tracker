/** 
  ******************************************************************************
  * @文件   
  * @作者   罗桂发
  * @版本   V1.0.0
  * @日期   2017-05-03 
  * @说明   
  ********  利尔达科技集团股份有限公司  www.lierda.com ***********************            
  *
***/

#ifndef TOOLS_H
#define TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif
     
#include "stdint.h"
#include "stdarg.h"
    
#ifndef CONST
  #define CONST const
#endif

#ifndef GENERIC
  #define GENERIC
#endif

typedef enum 
{
  CONFIG_ERROR = 1, 
  CONFIG_SUCCESS = !CONFIG_ERROR
}ConfigErrorStatus;
    
    
#define TWO_DIGITS_LEN                 2
#define FOUR_DIGITS_LEN                4  

#define Conn(x,y) x##y
    
    

#define BigtoLittle16(A)   (( ((uint16_t)(A) & 0xff00) >> 8)    |  (( (uint16_t)(A) & 0x00ff) << 8))  


 
#define BigtoLittle32(A)   ((( (uint32_t)(A) & 0xff000000) >> 24) | \
                                       (( (uint32_t)(A) & 0x00ff0000) >> 8)   | \
                                         (( (uint32_t)(A) & 0x0000ff00) << 8)   | \
                                       (( (uint32_t)(A) & 0x000000ff) << 24))  
 
#define BigtoLittle64(A)              ((( (uint64_t)(A) & 0xff00000000000000) >> 56) | \
                                       (( (uint64_t)(A) & 0x00ff000000000000) >> 40)   | \
                                       (( (uint64_t)(A) & 0x0000ff0000000000) >> 24)   | \
                                       (( (uint64_t)(A) & 0x000000ff00000000) >> 8) |  \
                                       (( (uint64_t)(A) & 0x00000000ff000000) << 8)  |           \
                                       (( (uint64_t)(A) & 0x0000000000ff0000) << 24)   |         \
                                       (( (uint64_t)(A) & 0x000000000000ff00) << 40)   |         \
                                       (( (uint64_t)(A) & 0x00000000000000ff) << 56))  


#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

extern uint8_t at_cmd_data_p[1024];
extern uint8_t checkSum(uint8_t *buf,uint16_t len);     
extern void *tool_memcpy( void*, const void GENERIC *, unsigned int );
extern void tool_strcat(uint8_t secLen, ...);
extern uint16_t HexsToAscs(uint8_t *hexs,char * ascs,uint16_t length);
extern uint16_t ascsToHexs(char *ascs,uint8_t * hexs,uint16_t length);
extern uint8_t checkSumChar(char *ch, uint16_t len);
extern uint8_t charsToHex(char *asc);
//extern void hex2String(uint8_t* hex, uint16_t len, char* tmp);
extern void hex2String(const uint8_t *source, uint16_t sourceLen, char *dest); 
extern void ascsToascString(char* asc, char* string, uint16_t length);
extern char *mystrstr(const char*s1,const char*s2, uint16_t s1Len);
extern uint8_t hex_to_dig(uint8_t hex);
extern uint8_t charsToHex1(char *asc);
#ifdef __cplusplus
}
#endif

#endif
     