各个oc目前版本如文件夹中所示，为历史版本
最新的固件建议基于 AIS_OC正式版（此版本为最新版）重新生成！！！
若要生产请确认NB模组和它的版本为最新

注意！！
1.源码中屏蔽了整机的出厂检测
若要生产需修改屏蔽如下语句：
修改前：
#warning 调试用，屏蔽出厂检测!!   
   PT_flag[0] = SET;

修改后：
#warning 调试用，屏蔽出厂检测!!   
//   PT_flag[0] = SET;

2.杨彬这边之前硬件小批的时候LDO错买成了3.3V的，后续若生产时硬件改回来了
软件需修改如下地方：
修改前：
AD_Value  = (AD_Value*3200) >> 12;

修改后：
AD_Value  = (AD_Value*3000) >> 12;


3.若要修改南向IP地址 
软件所需修改如下地方：
nb.h文件
#define AT_IOT_IP_LEN                   18                     //IP长度
#define AT_IOT_IP                       "=10.143.132.1,5683"   //IOT平台地址
将其中的ip地址更换为所需要的即可

main.C文件
OTA_flag.ip[0] = 10;
OTA_flag.ip[1] = 143;
OTA_flag.ip[2] = 132;
OTA_flag.ip[3] = 1;
将其中的ip地址更换为所需要的即可

4.传感器3dh目前的 SAO口是拉低的，杨彬这边新的板子后续打算拉高，这样设备地址需要改变
软件所需修改如下地方：
lsd_mems.c文件

修改前：
#define I2C_READ_ADDRESS          0x31
#define I2C_WRITE_ADDRESS         0x30
修改后：
#define I2C_READ_ADDRESS          0x33
#define I2C_WRITE_ADDRESS         0x32


修改后需重新生成对应代码。