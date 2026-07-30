#include "CdprForceInput.h"

#include <cmath>

namespace {
bool finite6(const CdprVector6 &values)
{
    for (double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}
CdprVector3 rotate(const std::array<double, 9> &rotation,
                   const CdprVector3 &value)
{
    return {
        rotation[0] * value.x + rotation[1] * value.y
            + rotation[2] * value.z,
        rotation[3] * value.x + rotation[4] * value.y
            + rotation[5] * value.z,
        rotation[6] * value.x + rotation[7] * value.y
            + rotation[8] * value.z
    };
}

CdprVector3 cross(const CdprVector3 &left, const CdprVector3 &right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}
}

CdprWrenchTransformer::CdprWrenchTransformer(
    const CdprForceSensorConfig &configuration)
    : configuration_(configuration)
{
}

CdprWrenchTransformResult CdprWrenchTransformer::toPlatformCenterOfMass(
    const CdprWrenchSample &sensorSample) const
{
    CdprWrenchTransformResult result;
    result.sample.stamp = sensorSample.stamp;
    result.sample.coordinate =
        CdprWrenchCoordinate::PlatformBodyAtCenterOfMass;
    if (!sensorSample.valid || !sensorSample.stamp.valid) {
        result.errorText = QStringLiteral("F/T输入帧无效。");
        return result;
    }
    if (sensorSample.coordinate != CdprWrenchCoordinate::Sensor) {
        result.errorText = QStringLiteral("F/T输入不是传感器坐标系。");
        return result;
    }
    if (!finite6(sensorSample.wrench)) {
        result.errorText = QStringLiteral("F/T输入包含非有限数。");
        return result;
    }

    CdprVector6 calibrated {};
    for (int index = 0; index < 6; ++index) {
        calibrated[static_cast<size_t>(index)] =
            static_cast<double>(configuration_.wrenchReactionSign)
            * configuration_.channelScale[static_cast<size_t>(index)]
            * (sensorSample.wrench[static_cast<size_t>(index)]
               - configuration_.channelBias[static_cast<size_t>(index)]);
    }

    const CdprVector3 forceSensor {
        calibrated[0], calibrated[1], calibrated[2]
    };
    const CdprVector3 momentSensor {
        calibrated[3], calibrated[4], calibrated[5]
    };
    const CdprVector3 forcePlatform =
        rotate(configuration_.rotationSensorToPlatform, forceSensor);
    const CdprVector3 momentAtSensorPlatform =
        rotate(configuration_.rotationSensorToPlatform, momentSensor);
    const CdprVector3 offsetMoment =
        cross(configuration_.originInPlatformM, forcePlatform);
    const CdprVector3 momentAtCenterOfMass {
        momentAtSensorPlatform.x + offsetMoment.x,
        momentAtSensorPlatform.y + offsetMoment.y,
        momentAtSensorPlatform.z + offsetMoment.z
    };

    result.sample.wrench = {
        forcePlatform.x, forcePlatform.y, forcePlatform.z,
        momentAtCenterOfMass.x, momentAtCenterOfMass.y,
        momentAtCenterOfMass.z
    };
    result.sample.valid = finite6(result.sample.wrench);
    if (!result.sample.valid) {
        result.errorText = QStringLiteral("F/T坐标变换结果包含非有限数。");
    }
    return result;
}

void SimulatedWrenchSource::setSensorWrench(const CdprVector6 &wrench)
{
    wrench_ = wrench;
}

CdprVector6 SimulatedWrenchSource::sensorWrench() const
{
    return wrench_;
}

CdprWrenchSample SimulatedWrenchSource::sample(
    const CdprFrameStamp &stamp) const
{
    CdprWrenchSample result;
    result.stamp = stamp;
    result.wrench = wrench_;
    result.coordinate = CdprWrenchCoordinate::Sensor;
    result.valid = stamp.valid && finite6(wrench_);
    return result;
}

bool TraceFtSensorSource::configured() const
{
    return false;
}

QString TraceFtSensorSource::statusText() const
{
    return QStringLiteral("Trace F/T对象类型待配置，当前不生成有效力帧");
}

CdprWrenchSample TraceFtSensorSource::latestSample() const
{
    return {};
}
