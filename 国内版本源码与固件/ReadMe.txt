1.Locator_L051T8_Schedule0723OTA_IMSI           
文件夹中为对应LSD4SA-NBTK000001_V01 硬件源码（LDO为3.3V等）

2.Locator_L051T8_Schedule0723OTA_IMSI_SENSOR    
文件夹中基于1的源码 对应LSD4SA-NBTK000001_V03 硬件，对应修改部分功能，增加了三轴传感器

3.Locator_L051T8_Schedule0723OTA_IMSI_WANMA
文件夹中基于1的源码 对应LSD4SA-NBTK000001_V01 硬件，为对 浙江万马 客户的订制品
修改了上报频率固定为1小时，每隔15分钟搜星记录一次。

其中boot文件为沈帅帅这边代码生成，为FOTA功能固件。
上述源码在编译生成后的固件与boot合并之后的 固件才可使用。
--FOTA相关功能可联系沈帅帅