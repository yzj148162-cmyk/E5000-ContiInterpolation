using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace 雷赛总线卡功能实现例程_2025
{ 
    public struct structAdvancPlanParam_t  //4阶S型，4阶正余弦
    {
        public double fs;          //起始速度，单位：unit/s
        public double f;           //运行速度，单位：unit/s
        public double fe;          //结束速度，单位：unit/s
        public double avg_acc;     //平均加速度，单位：unit/s^2
        public double avg_dec;     //平均减速度，单位：unit/s^2
        public double JaRatio;     //加加速度比率
        public double JdRatio;     //减减速度比率
    }
    public struct structSplusePlanParam_t  //Splus型、加加速限制的4阶S规划参数结构体
    {
        public double fs;          //起始速度，单位：unit/s
        public double f;           //运行速度，单位：unit/s
        public double fe;          //结束速度，单位：unit/s
        public double avg_acc;     //最大加速度，单位：unit/s^2
        public double avg_dec;     //最大减速度，单位：unit/s^2
        public double Ja;          //最大加加速度，单位：unit/s^3
        public double Jd;          //最大减减速度，单位：unit/s^3
    }
}
