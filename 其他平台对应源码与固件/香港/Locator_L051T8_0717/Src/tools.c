/** 
  ******************************************************************************
  * @文件   tools.c
  * @作者   罗桂发\陈恺霖
  * @版本   V1.0.0
  * @日期   2017-05-03 
  * @说明   
  ********  利尔达科技集团股份有限公司  www.lierda.com ***********************            
  *
***/
#include "tools.h"
#include "stdint.h"
#include <stdlib.h>
//#include "stdio.h" 
#include "string.h"

uint8_t at_cmd_data_p[1024];

void  hexToAscs(uint8_t hex,char *ascs)
{
  uint8_t h,l;
  h=(hex>>4)&0x0f;
  l=(hex&0x0f);
  
  
  if(h<=9)   //lgf0503  ((h>=0)&&(h<=9))
    ascs[0]=h+0x30;
  else if((h>=10)&&(h<=15)){
    ascs[0]=h+0x41-10;
  }else{
    ascs[0]=0xff;
  }
  
  if(l<=9)   //lgf0503   ((l>=0)&&(l<=9))
    ascs[1]=l+0x30;
  else if((l>=10)&&(l<=15)){
    ascs[1]=l+0x41-10;
  }else{
    ascs[1]=0xff;
  }  
}
uint16_t HexsToAscs(uint8_t *hexs,char * ascs,uint16_t length)
{
  uint16_t j=0;
  for(uint16_t i=0;i<length;i++){
    hexToAscs(hexs[i],ascs+j);
    j+=2;
  } 
  return j;
}
uint8_t charsToHex(char *asc)
{
  
  uint8_t hex=0;
  if((asc[0]>='0')&&(asc[0]<='9')){
    hex=asc[0]-0x30;
  }
  else if((asc[0]>='a')&&(asc[0]<='f')){
    hex=asc[0]-'a'+0xa;
  }
  else if((asc[0]>='A')&&(asc[0]<='F')){
    hex=asc[0]-'A'+0xa;
  }
  
  hex=hex<<4;
  
  if((asc[1]>='0')&&(asc[1]<='9')){
    hex+=(asc[1]-0x30);
  }
  else if((asc[1]>='a')&&(asc[1]<='f')){
    hex+=(asc[1]-'a'+0xa);
  }
  else if((asc[1]>='A')&&(asc[1]<='F')){
    hex+=(asc[1]-'A'+0xa);
  } 
  
  return hex;
}
uint16_t  ascsToHexs(char *ascs,uint8_t * hexs,uint16_t length)
{
  uint16_t j=0;
  for(uint16_t i=0;i<length;i+=2){
    
    hexs[j++]=charsToHex(ascs+i);
    //hexs[i]=charToHex(ascs[i]);   
  } 
  return j;
}

uint8_t checkSum(uint8_t *buf, uint16_t len)
{
   uint8_t result=0;
   uint16_t i;
   for(i=0;i<len;i++)
   {
      result+=buf[i]; 
   }
   return result;
}

//字符串，校验和
uint8_t checkSumChar(char *ch, uint16_t len)
{
    uint8_t hex = 0;
    for(uint16_t i=0;i<=len;i+=2)
    {
        hex = hex + charsToHex(ch+i);
    }
    return hex;
}  
    
/*
* @fn      tool_memcpy
* @brief   memcpy 
* @param   
* @return   
*/
void *tool_memcpy( void *dst, const void GENERIC *src, unsigned int len )
{
  uint8_t *pDst;
  const uint8_t GENERIC *pSrc;

  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc++;

  return ( pDst );
}

/*
* @fn      tool_strcat
* @brief   字符串拼接 
* @param   片段长度，字符，字符....
* @return   
*/
void tool_strcat(uint8_t secLen, ...)
{  
    va_list arg_ptr;
    uint8_t strSumLen = 0;    //字符串总长
    char* curStr;             //当前字符串
    
    //长度必须大于2，才能完成拼接
    if(secLen < 2)
    {
        exit (0);
    }
    else
    {
        uint8_t i=0;
        //获取总长
        va_start(arg_ptr, secLen); 
        for(i=0; i<secLen; i++)
        {
            curStr = va_arg(arg_ptr, char*);
            strSumLen = strSumLen + strlen(curStr);
        }
        va_end(arg_ptr);
        
        //char* result = malloc(strSumLen+1);//+1 for the zero-terminator
        
        //in real code you would check for errors in malloc here  
//        if (result == NULL) 
//        {
//            exit (1);
//        }
        //拼接字符串
        va_start(arg_ptr, secLen);
        for(i=0; i<secLen; i++)
        {
            curStr = va_arg(arg_ptr, char*);
            //先拷贝第一个字符串，作为头字段
            if(i == 0)
            {
                strcpy((char*)at_cmd_data_p, curStr);
            }
            else
            {
                strcat((char*)at_cmd_data_p, curStr);
            }
        }
        va_end(arg_ptr);
        return;
    }
    return;
}

char HexToChar(uint8_t hex)
{
  
  char ch=0;
  if((hex>0)&&(hex<=9))
  {
    ch=hex+0x30;
  }
  else if((hex>9)&&(hex<=15))
  {
    ch=hex-10+0x41;
  }
  else if(hex==0)
  {
      ch=0x30;
  }
  return ch;
}

/*
* @fn      hex2String
* @brief   hex转字符串
* @param   
* @return   
*/
//void hex2String(uint8_t* hex, uint16_t len, char* tmp)
//{
//    uint8_t data;
//    uint16_t hexlen;
//    hexlen = len*2;
//    for(uint8_t i=0,j=0; i<hexlen ;i=i+2,j++)
//    {
//        data = (*(hex+j))/16;
//        *(tmp+i) = HexToChar(data);
//        data = (*(hex+j))%16;
//        *(tmp+i+1) = HexToChar(data);
//    }
//}

void hex2String(const uint8_t *source, uint16_t sourceLen, char *dest)  
{  
    short i;  
    unsigned char highByte, lowByte;  
  
    for (i = 0; i < sourceLen; i++)  
    {  
        highByte = source[i] >> 4;  
        lowByte = source[i] & 0x0f ;  
  
        highByte += 0x30;  
  
        if (highByte > 0x39)  
                dest[i * 2] = highByte + 0x07;  
        else  
                dest[i * 2] = highByte;  
  
        lowByte += 0x30;  
        if (lowByte > 0x39)  
            dest[i * 2 + 1] = lowByte + 0x07;  
        else  
            dest[i * 2 + 1] = lowByte;  
    }  
    return;  
}
/*
* @fn      ascsToascString
* @brief   asc转asc字符串
* @param   
* @return   
*/
void ascsToascString(char* asc, char* string, uint16_t length)
{
    //例："312E312E3230313730393039" <-- 1.1.20170909
    uint16_t j=0;
    for(uint16_t i=0;j<length;i+=2,j++)
    {
        string[i]= asc[j]/16;
        if(string[i] > 9)
        {
            string[i] = string[i] + 0x41 - 10;
        }
        else
        {
            string[i] = string[i] + 0x30;
        }
        string[i+1] = asc[j]%16;
        if(string[i+1] > 9)
        {
            string[i+1] = string[i+1] + 0x41 - 10;
        }
        else
        {
            string[i+1] = string[i+1] + 0x30;
        }
    }
}

/*
* @fn      mystrstr
* @brief   字符串查找函数
* @param   
* @return   
*/
char *mystrstr(const char*s1,const char*s2, uint16_t s1Len)
{
    int n;
    int len = 0;
    if(*s2)
    {
        while(*s1 && len<s1Len)
        {
            for(n=0;*(s1+n)==*(s2+n);n++)
            {
                if(!*(s2+n+1))
                    return(char*)s1;
            }
            s1++;
            len++;
        }
        return NULL;
    }
    else
        return (char*)s1;
}

//16进制转10进制
uint8_t hex_to_dig(uint8_t hex)
{
    uint8_t d;
    d = hex/16*10 + hex%16;
    return d;
}


uint8_t charsToHex1(char *asc)
{
  
  uint8_t hex=0;
  if((asc[0]>='0')&&(asc[0]<='9')){
    hex=asc[0]-0x30;
  }
  else if((asc[0]>='a')&&(asc[0]<='f')){
    hex=asc[0]-'a'+0xa;
  }
  else if((asc[0]>='A')&&(asc[0]<='F')){
    hex=asc[0]-'A'+0xa;
  }
  return hex;
}

