#ifndef CDPRFORCEINPUT_H
#define CDPRFORCEINPUT_H

#include "CdprConfiguration.h"
#include "CdprControlTypes.h"

#include <QString>

struct CdprWrenchTransformResult
{
    CdprWrenchSample sample;
    QString errorText;
};

class CdprWrenchTransformer
{
public:
    explicit CdprWrenchTransformer(const CdprForceSensorConfig &configuration);

    CdprWrenchTransformResult toPlatformCenterOfMass(
        const CdprWrenchSample &sensorSample) const;

private:
    CdprForceSensorConfig configuration_;
};

class SimulatedWrenchSource
{
public:
    void setSensorWrench(const CdprVector6 &wrench);
    CdprVector6 sensorWrench() const;
    CdprWrenchSample sample(const CdprFrameStamp &stamp) const;

private:
    CdprVector6 wrench_ {};
};

// Trace对象类型尚未确定。本骨架在未配置前始终输出无效帧，
// 防止把零值或旧值伪装成真实F/T数据。
class TraceFtSensorSource
{
public:
    bool configured() const;
    QString statusText() const;
    CdprWrenchSample latestSample() const;
};

#endif // CDPRFORCEINPUT_H
