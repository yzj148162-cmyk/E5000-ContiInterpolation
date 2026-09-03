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
// 模拟力默认已经位于平台质心，不经过本模块。
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
