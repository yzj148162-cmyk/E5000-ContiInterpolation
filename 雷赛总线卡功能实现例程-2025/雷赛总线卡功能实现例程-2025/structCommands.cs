using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace 雷赛总线卡功能实现例程_2025
{
    public struct structCommands
    {
        public bool ExcuteConnect ;         //连接控制卡
        public bool ExcuteDisConnect;       //断开连接控制卡
        public bool ExcuteInit ;            //初始化控制卡
        public bool ExcuteSoftReset ;       //热复位控制卡
        public bool ExcuteEmgStop ;         //急停位控制卡
        public bool ExcuteMovePos;
        public bool ExcuteMoveTorque;//转矩运动
        public bool ExcuteMoveOscillate;//正余弦运动
        public bool ExcuteStopOscillate;//停止正余弦运动
        public bool ExcuteStopMove;
        public bool ExcuteTraceData;//启动采样跟踪
        public bool ExcuteStopTraceData;//停止采样跟踪
        public bool ExcuteClearAxisError ;  //清除轴报警
        public bool ExcuteClearEtherCATError;  //清除总线报警
        public bool ExcuteClearAxisPos;     //清零轴位置
        public bool ExcuteAxisPowerOn;
        public bool ExcuteSetDaValue;       //设置模拟量值
        public bool ExcuteTurnOnOffOutbit;       //切换输出口开关
        public bool ExcuteClearExtraEncoder;//清楚辅助编码器的值
        public bool ExcuteHcmpEnable;       //启动高速位置比较功能
        public bool ExcuteHcmpDisable;       //关闭高速位置比较功能
        public bool ExcuteVectorMove;       //启动插补运动
        public bool ExcuteStopVectorMove;   //停止插补运动
        public bool ExcuteGearMove;       //启动龙门跟随
        public bool ExcuteStopGearMove;       //停止龙门跟随
        public bool ExcuteGroupMove;        //启动指令缓存运动
        public bool ExcuteGroupStopMove;    //停止指令缓存运动
        public bool ExcuteGroupPauseMove;   //暂停指令缓存运动
        public bool ExcuteReadExtraPDO;        //读取扩展PDO的值
        public bool ExcuteWriteExtraPDO;        //写扩展PDO的值
        public bool ExcuteReadPDO;        //读取PDO的值
        public bool ExcuteWritePDO;        //写PDO的值
        public bool ExcuteAutoRun;          //自动运行
    }
}
