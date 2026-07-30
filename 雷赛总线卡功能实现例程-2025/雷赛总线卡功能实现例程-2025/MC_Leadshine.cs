using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace 雷赛总线卡功能实现例程_2025
{
    public class MC_Leadshine
    {
        private const ushort AxisNumMax = 256;
        public ushort CardID { get; set; }              //控制卡连接ID号
        public bool IsConnectDone { get; set; }         //控制卡连接状态
        public bool IsInitDone { get; set; }            //控制卡初始化状态
        public bool IsSoftReset { get; set; }           //控制卡热复位状态
        public bool IsEMG { get; set; }                 //控制卡急停状态
        public bool IsTracing { get; set; }             //控制卡采样启动中
        public bool IsAutoRunning { get; set; }         //控制卡自动运行状态
        public ushort EtherCATState { get; set; }       //控制卡EhherCAT总线通讯状态,0为正常，非0不正常
        public ushort LocalBusState { get; set; }       //控制卡背板总线通讯状态,0为正常，非0不正常
        public string ReleaseVersion { get; set; }      //控制卡发布版本号
        public uint CardVersion { get; set; }           //控制卡硬件版本号
        public uint FirmID { get; set; }                //控制卡固件类型
        public uint SubFirmID { get; set; }             //控制卡固件版本号
        public int EthercatTime { get; set; }               //控制卡总线周期时间，单位us
        public uint SlaveNum { get; set; }                //控制卡从站数量
        public uint AxisNum { get; set; }               //控制卡轴的数量
        public uint InputNum { get; set; }              //控制卡输入口数量
        public uint OutputNum { get; set; }             //控制卡输出口数量
        public uint AdNum { get; set; }                 //控制卡模拟量输入通道数量
        public uint DaNum { get; set; }                 //控制卡模拟量输出通道数量   
        public bool[] AxisBusy { get; set; }            // 控制卡轴运行状态，false停止，true运行
        public bool[] AxisAlarm { get; set; }           // 控制卡轴报警Alarm状态，true报警
        public bool[] AxisELP { get; set; }             // 控制卡轴正限位状态，true报警
        public bool[] AxisELN { get; set; }             // 控制卡轴负限位状态，true报警
        public bool[] AxisORG { get; set; }              // 控制卡轴原点状态，true有效
        public bool[] AxisEMG { get; set; }             // 控制卡轴急停EMC状态，true报警
        public bool[] AxisPowerOn { get; set; }         // 控制卡轴使能状态状态，true使能
        public bool[] AxisMoveDir { get; set; }         // 控制卡轴运动方向，false负向，true正向
        public ushort[] AxisHomeResult { get; set; }    // 控制卡轴回零结果，false未回零，true回零正常
        public ushort[] AxisRunMode { get; set; }    // 控制卡轴运动模式
        public int[] AxisStopReason { get; set; }       // 控制卡轴停止原因
        public double[] AxisCommandPos { get; set; }    // 控制卡轴指令位置
        public double[] AxisActualPos { get; set; }     // 控制卡轴反馈位置
        public double[] AxisCurrentSpeed { get; set; }  // 控制卡轴当前运动速度
        public double[] AxisCurrentTorque { get; set; }  // 控制卡轴当前运动速度
        public double[] AxisStartVel { get; set; }      // 控制卡轴起始速度
        public double[] AxisMaxVel { get; set; }        // 控制卡轴最大运行速度
        public double[] AxisStopVel { get; set; }       // 控制卡轴停止速度
        public double[] AxisTimeAcc { get; set; }       // 控制卡轴加速时间，单位秒
        public double[] AxisTimeDec { get; set; }       // 控制卡轴减速时间，单位秒
        public double[] AxisTargetPos { get; set; }     // 控制卡轴目标位置
        public double[] AxisDistancePos { get; set; }   // 控制卡轴运动距离
        public double[] ADValue { get; set; }           //AD模拟量输入的值，单位v或mA
        public double[] DAValue { get; set; }           //DA模拟量输出的值，单位v或mA
        public int[] ExtraEncoder { get; set; }         //辅助编码器的值，单位pulse

        public int remainedPoints { get; set; }                                  //返回可添加比较点数
        public double currentPoint { get; set; }                                 //返回当前比较点位置，单位：pluse
        public int runnedPoints { get; set; }                                    //返回已比较点数

        public short shStartFlag { get; set; }//Trace采集启动标志
        public short shTriggeredFlag { get; set; }//Trace采集触发标志
        public short shLostflag { get; set; }//Trace采集溢出标志
        public int iValidNum { get; set; }      //Trace已采集但未被读取的数据个数
        public int iFreeNum { get; set; }       //Trace剩余可用于保存采集数据的个数
        public int iObjectTotalBytes { get; set; }  //Trace采集对象总字节数
        public int iValidObjectTotalNum  { get; set; }  //Trace采集对象总个数
        public short[] ObjectBytes { get; set; }//Trace采集对象的字节大小
        public short runState { get; set; }//坐标系的插补运动状态运动状态，0：运动中，1：暂停中，2：正常停止，3：未启动，4：空闲
        public int remainSpace { get; set; }//插补缓冲区剩余插补空间
        public int currentMark { get; set; }//插补缓冲区当前插补段号
        public double vectorCurrentSpeed { get; set; }//插补系的合速度值

        public uint uiGroupRemainSpace { get; set; }//缓存区剩余空间
        public ushort usGroupStste { get; set; }//当前指令的运行状态
        public ushort usGroupEnable { get; set; }//Group打开，0表示未使能；1表示使能
        public uint uiGroupStopReason { get; set; }//停止原因
        public ushort usGroupTrigPhase { get; set; }//当前指令等待触发的过程
        public uint uiGroupMark { get; set; }//当前执行段的段号 


        public MC_Leadshine()
        {
            this.AxisBusy = new bool[AxisNumMax];
            this.AxisAlarm = new bool[AxisNumMax];
            this.AxisELP = new bool[AxisNumMax];
            this.AxisELN = new bool[AxisNumMax];
            this.AxisORG = new bool[AxisNumMax];
            this.AxisEMG = new bool[AxisNumMax];
            this.AxisPowerOn = new bool[AxisNumMax];
            this.AxisMoveDir = new bool[AxisNumMax];
            this.AxisHomeResult = new ushort[AxisNumMax];
            this.AxisRunMode = new ushort[AxisNumMax];
            this.AxisStopReason = new int[AxisNumMax];
            this.AxisCommandPos = new double[AxisNumMax];
            this.AxisActualPos = new double[AxisNumMax];
            this.AxisCurrentSpeed = new double[AxisNumMax];
            this.AxisCurrentTorque = new double[AxisNumMax];
            this.AxisStartVel = new double[AxisNumMax];
            this.AxisMaxVel = new double[AxisNumMax];
            this.AxisStopVel = new double[AxisNumMax];
            this.AxisTimeAcc = new double[AxisNumMax];
            this.AxisTimeDec = new double[AxisNumMax];
            this.AxisTargetPos = new double[AxisNumMax];
            this.AxisDistancePos = new double[AxisNumMax];
            this.ADValue = new double[AxisNumMax];
            this.DAValue = new double[AxisNumMax];
            this.ExtraEncoder = new int[AxisNumMax];
            this.ObjectBytes = new short[2048];
        }
    }
}
