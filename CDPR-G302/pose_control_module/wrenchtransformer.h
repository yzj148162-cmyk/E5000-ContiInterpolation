#ifndef WRENCHTRANSFORMER_H
#define WRENCHTRANSFORMER_H

#include "forceinteractiontypes.h"

struct WrenchTransformResult
{
    ForceInteractionWrenchSample sample;
    QString errorMessage;
};

// 只负责传感器坐标系到平台质心 body frame 的确定性变换。
// 已知平移为平台局部 +Z 方向 0.32548 m，且传感器三轴与平台局部三轴同向，
// 所以 R_ES=I。输出仍在平台质心 body frame；平动力在动力学内部再转到全局系。
// 阶段B模拟量和阶段C真实量均采用“传感器测量点、传感器坐标系”语义，
// 因而统一经过本模块；阶段C还会在这里减去准备时冻结的软件零点。
class WrenchTransformer
{
public:
    explicit WrenchTransformer(const ForceSensorTransformConfig& configuration);
    WrenchTransformResult toPlatformCenterOfMass(
            const ForceInteractionWrenchSample& sensorSample) const;

private:
    ForceSensorTransformConfig configuration_;
};

#endif // WRENCHTRANSFORMER_H
