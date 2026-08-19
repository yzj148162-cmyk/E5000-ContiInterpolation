#ifndef CDPRFORCEINPUT_H
#define CDPRFORCEINPUT_H

#include "CdprConfiguration.h"
#include "CdprControlTypes.h"

#include <functional>
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
    bool configure(const CdprSimulatedWrenchProfile &profile,
                   QString *errorText = nullptr);
    CdprSimulatedWrenchProfile profile() const;
    void setSensorWrench(const CdprVector6 &wrench);
    CdprVector6 sensorWrench() const;
    CdprWrenchSample sample(const CdprFrameStamp &stamp,
                            double elapsedS = 0.0) const;
    bool evaluate(double elapsedS, CdprVector6 &wrench,
                  QString *errorText = nullptr) const;
    QString summary() const;

private:
    CdprSimulatedWrenchProfile profile_;
    std::array<std::function<double(double)>, kCdprDofCount>
        formulaEvaluators_ {};
    bool configured_ = true;
};

// 真实F/T必须由硬件接口从与8轴反馈一致的Trace帧中解码后送入本适配器。
// 对象类型未配置前始终输出无效帧，防止把零值或旧值伪装成真实数据。
class TraceFtSensorSource
{
public:
    void acceptSameFrameSample(const CdprWrenchSample &sample);
    void clear();
    bool configured() const;
    QString statusText() const;
    CdprWrenchSample latestSample() const;

private:
    CdprWrenchSample latestSample_;
    bool configured_ = false;
};

#endif // CDPRFORCEINPUT_H
