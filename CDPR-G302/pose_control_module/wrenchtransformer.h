#ifndef WRENCHTRANSFORMER_H
#define WRENCHTRANSFORMER_H

#include "forceinteractiontypes.h"

struct WrenchTransformResult
{
    ForceInteractionWrenchSample sample;
    QString errorMessage;
};

// 只负责传感器坐标系到平台质心 body frame 的确定性变换。
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
