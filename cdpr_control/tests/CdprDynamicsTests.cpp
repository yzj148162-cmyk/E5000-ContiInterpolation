#include <QCoreApplication>
#include <QDebug>

#include <cmath>

#include "cdpr/CdprDynamics.h"
#include "cdpr/CdprForceInput.h"

namespace {
bool near(double left, double right, double tolerance)
{
    return std::abs(left - right) <= tolerance;
}
bool require(bool condition, const QString &message)
{
    if (!condition) {
        qCritical().noquote() << QStringLiteral("失败：") + message;
    }
    return condition;
}

CdprRigidBodyConfig testBody()
{
    CdprRigidBodyConfig body;
    body.massKg = 2.0;
    body.inertiaKgM2 = {1, 0, 0, 0, 2, 0, 0, 0, 4};
    return body;
}

CdprWrenchSample validBodyWrench(const CdprVector6 &wrench)
{
    CdprWrenchSample sample;
    sample.stamp = {1, 1000, true};
    sample.wrench = wrench;
    sample.coordinate =
        CdprWrenchCoordinate::PlatformBodyAtCenterOfMass;
    sample.valid = true;
    return sample;
}
}

int main(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);
    bool passed = true;

    CdprForceSensorConfig sensor;
    sensor.rotationSensorToPlatform =
        {1, 0, 0, 0, 1, 0, 0, 0, 1};
    sensor.originInPlatformM = {0.0, 1.0, 0.0};
    CdprWrenchSample raw;
    raw.stamp = {2, 2000, true};
    raw.wrench = {1, 0, 0, 0, 0, 0};
    raw.coordinate = CdprWrenchCoordinate::Sensor;
    raw.valid = true;
    const CdprWrenchTransformResult transformed =
        CdprWrenchTransformer(sensor).toPlatformCenterOfMass(raw);
    passed &= require(transformed.sample.valid,
                      QStringLiteral("F/T坐标变换应有效。"));
    passed &= require(near(transformed.sample.wrench[0], 1.0, 1.0e-12)
                          && near(transformed.sample.wrench[5],
                                  -1.0, 1.0e-12),
                      QStringLiteral("偏心力矩r×F计算错误。"));

    sensor.wrenchReactionSign = -1;
    const CdprWrenchTransformResult reversed =
        CdprWrenchTransformer(sensor).toPlatformCenterOfMass(raw);
    passed &= require(near(reversed.sample.wrench[0], -1.0, 1.0e-12)
                          && near(reversed.sample.wrench[5],
                                  1.0, 1.0e-12),
                      QStringLiteral("作用/反作用整体符号计算错误。"));

    CdprDynamics dynamics;
    QString error;
    passed &= require(dynamics.configure(
                          testBody(), CdprNewmarkConfig {}, &error),
                      QStringLiteral("Newmark配置失败：%1").arg(error));
    CdprPlatformState6 initial;
    initial.poseValid = true;
    initial.twistValid = true;
    initial.accelerationValid = true;
    passed &= require(dynamics.reset(initial, &error),
                      QStringLiteral("Newmark重置失败：%1").arg(error));

    const CdprDynamicsResult forceStep =
        dynamics.step(validBodyWrench({4, 0, 0, 0, 0, 0}), 0.01);
    passed &= require(forceStep.valid && forceStep.converged,
                      QStringLiteral("恒力Newmark单步失败：%1")
                          .arg(forceStep.errorText));
    passed &= require(near(forceStep.state.acceleration[0],
                           2.0, 1.0e-10)
                          && near(forceStep.state.twist[0],
                                  0.02, 1.0e-10)
                          && near(forceStep.state.pose[0],
                                  0.0001, 1.0e-10),
                      QStringLiteral("恒力平动结果不符合解析解。"));

    CdprPlatformState6 driftInitial;
    driftInitial.poseValid = true;
    driftInitial.twistValid = true;
    driftInitial.twist[1] = 3.0;
    passed &= require(dynamics.reset(driftInitial, &error),
                      QStringLiteral("自由漂移重置失败：%1").arg(error));
    const CdprDynamicsResult driftStep =
        dynamics.step(validBodyWrench({}), 0.02);
    passed &= require(driftStep.valid
                          && near(driftStep.state.pose[1],
                                  0.06, 1.0e-10)
                          && near(driftStep.state.twist[1],
                                  3.0, 1.0e-10),
                      QStringLiteral("零外力自由漂移结果错误。"));

    if (passed) {
        qInfo() << QStringLiteral(
            "CdprDynamics与F/T变换离线测试通过。");
        return 0;
    }
    return 1;
}
