#include "wrenchtransformer.h"

#include <cmath>

namespace {

bool finite6(const ForceInteractionVector6& values)
{
    for(double value : values){
        if(!std::isfinite(value)){
            return false;
        }
    }
    return true;
}

ForceInteractionVector3 rotate(const ForceInteractionMatrix3& matrix,
                               const ForceInteractionVector3& value)
{
    return {{
        matrix[0] * value[0] + matrix[1] * value[1] + matrix[2] * value[2],
        matrix[3] * value[0] + matrix[4] * value[1] + matrix[5] * value[2],
        matrix[6] * value[0] + matrix[7] * value[1] + matrix[8] * value[2]
    }};
}

ForceInteractionVector3 cross(const ForceInteractionVector3& left,
                              const ForceInteractionVector3& right)
{
    return {{
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]
    }};
}

} // namespace

WrenchTransformer::WrenchTransformer(const ForceSensorTransformConfig& configuration)
    : configuration_(configuration)
{
}

WrenchTransformResult WrenchTransformer::toPlatformCenterOfMass(
        const ForceInteractionWrenchSample& sensorSample) const
{
    WrenchTransformResult result;
    result.sample.stamp = sensorSample.stamp;
    result.sample.coordinate =
            ForceInteractionWrenchCoordinate::PlatformBodyAtCenterOfMass;
    if(!configuration_.configured){
        result.errorMessage = QStringLiteral("真实F/T安装变换尚未配置");
        return result;
    }
    if(!sensorSample.valid || !sensorSample.stamp.valid ||
            sensorSample.coordinate != ForceInteractionWrenchCoordinate::Sensor ||
            !finite6(sensorSample.wrench)){
        result.errorMessage = QStringLiteral("传感器坐标系F/T输入帧无效");
        return result;
    }
    if(configuration_.wrenchReactionSign != 1 &&
            configuration_.wrenchReactionSign != -1){
        result.errorMessage = QStringLiteral("F/T整体方向必须为+1或-1");
        return result;
    }

    ForceInteractionVector6 calibrated{};
    for(int index = 0; index < kForceInteractionDofCount; ++index){
        const size_t offset = static_cast<size_t>(index);
        calibrated[offset] =
                static_cast<double>(configuration_.wrenchReactionSign) *
                configuration_.channelScale[offset] *
                (sensorSample.wrench[offset] - configuration_.channelBias[offset]);
    }
    const ForceInteractionVector3 forceSensor{{
        calibrated[0], calibrated[1], calibrated[2]
    }};
    const ForceInteractionVector3 momentSensor{{
        calibrated[3], calibrated[4], calibrated[5]
    }};
    const ForceInteractionVector3 forcePlatform =
            rotate(configuration_.rotationSensorToPlatform, forceSensor);
    const ForceInteractionVector3 momentAtSensor =
            rotate(configuration_.rotationSensorToPlatform, momentSensor);
    const ForceInteractionVector3 offsetMoment =
            cross(configuration_.sensorOriginInPlatformM, forcePlatform);

    result.sample.wrench = {{
        forcePlatform[0], forcePlatform[1], forcePlatform[2],
        momentAtSensor[0] + offsetMoment[0],
        momentAtSensor[1] + offsetMoment[1],
        momentAtSensor[2] + offsetMoment[2]
    }};
    result.sample.valid = finite6(result.sample.wrench);
    if(!result.sample.valid){
        result.errorMessage = QStringLiteral("F/T坐标变换结果包含非有限数");
    }
    return result;
}
