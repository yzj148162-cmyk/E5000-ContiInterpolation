#ifndef MACRO_H
#define MACRO_H

/*
 * 文件总览：
 * - 项目级通信类型宏定义，统一标识电机、力传感器和 EtherCAT 设备的接入方式。
 * - 这些常量会在参数初始化和硬件配置中用于选择 HardwareInterface 的具体读写路径。
 */


// 以下为通讯类型 1开头是电机 2开头是传感器
#define COM_EC_LS 102// 雷赛EtherCAT
#define COM_485_SBT 200// 斯巴拓力传感器485通讯
#define COM_EC_LS_SBT 203// 斯巴拓力传感器雷赛EtherCAT

#endif // MACRO_H
