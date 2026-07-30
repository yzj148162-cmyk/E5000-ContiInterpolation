#include <QCoreApplication>
#include <QDebug>
#include <QDir>

#include <algorithm>
#include <cmath>

#include "cdpr/CdprConfiguration.h"
#include "cdpr/CdprKinematics.h"

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

CdprPlatformState6 platformAt(const CdprVector6 &pose,
                              const CdprVector6 &twist = {})
{
    CdprPlatformState6 platform;
    platform.pose = pose;
    platform.twist = twist;
    platform.poseValid = true;
    platform.twistValid = true;
    return platform;
}
}

int main(int argc, char *argv[])
{
    QCoreApplication application(argc, argv);
    const QString configurationPath = argc > 1
        ? QString::fromLocal8Bit(argv[1])
        : QDir(QCoreApplication::applicationDirPath())
              .absoluteFilePath(QStringLiteral("../cdpr_configuration.json"));

    CdprConfiguration configuration;
    QStringList validationMessages;
    if (!require(CdprConfigurationFile::load(
                     configurationPath, configuration, validationMessages),
                 QStringLiteral("无法加载测试配置：%1")
                     .arg(configurationPath))) {
        return 1;
    }

    CdprKinematics kinematics(configuration);
    QString geometryError;
    if (!require(kinematics.geometryValid(&geometryError),
                 QStringLiteral("几何参数无效：%1").arg(geometryError))) {
        return 1;
    }

    const CdprInverseKinematicsResult inverse =
        kinematics.inverse(platformAt(configuration.initialPlatformPose));
    if (!require(inverse.valid,
                 QStringLiteral("初始位姿逆运动学失败：%1")
                     .arg(inverse.errorText))) {
        return 1;
    }

    const CdprVector8 expected {
        2.36504768, 2.13622273, 3.07659694, 2.90438518,
        3.08957108, 2.89058023, 2.38190083, 2.11741599
    };
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        if (!require(near(inverse.cables.lengthM[static_cast<size_t>(cable)],
                          expected[static_cast<size_t>(cable)], 2.0e-6),
                     QStringLiteral("第%1根初始绳长与G302基准不一致：%2 m")
                         .arg(cable)
                         .arg(inverse.cables.lengthM[
                                  static_cast<size_t>(cable)],
                              0, 'g', 10))) {
            return 1;
        }
    }

    // 初始姿态为零，此时欧拉角微分与世界角速度的一阶关系一致，可直接用
    // 中心差分核对“绳长增加为正”的速度雅可比符号。
    const CdprVector6 twist {
        0.013, -0.009, 0.011, 0.007, -0.005, 0.006
    };
    const CdprInverseKinematicsResult moving =
        kinematics.inverse(platformAt(configuration.initialPlatformPose, twist));
    if (!require(moving.valid && moving.cables.velocityValid,
                 QStringLiteral("绳速逆解无效。"))) {
        return 1;
    }
    constexpr double timeStep = 1.0e-5;
    CdprVector6 plusPose = configuration.initialPlatformPose;
    CdprVector6 minusPose = configuration.initialPlatformPose;
    for (int dof = 0; dof < kCdprDofCount; ++dof) {
        plusPose[static_cast<size_t>(dof)]
            += twist[static_cast<size_t>(dof)] * timeStep;
        minusPose[static_cast<size_t>(dof)]
            -= twist[static_cast<size_t>(dof)] * timeStep;
    }
    const CdprInverseKinematicsResult plus =
        kinematics.inverse(platformAt(plusPose));
    const CdprInverseKinematicsResult minus =
        kinematics.inverse(platformAt(minusPose));
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        const double numerical =
            (plus.cables.lengthM[static_cast<size_t>(cable)]
             - minus.cables.lengthM[static_cast<size_t>(cable)])
            / (2.0 * timeStep);
        if (!require(near(
                         numerical,
                         moving.cables.velocityMps[static_cast<size_t>(cable)],
                         2.0e-8),
                     QStringLiteral("第%1根绳索的解析绳速与有限差分不一致。")
                         .arg(cable))) {
            return 1;
        }
    }

    CdprVector6 forwardGuess = configuration.initialPlatformPose;
    forwardGuess[0] += 0.005;
    forwardGuess[1] -= 0.004;
    forwardGuess[2] += 0.003;
    forwardGuess[3] += 0.005;
    forwardGuess[4] -= 0.004;
    forwardGuess[5] += 0.003;
    const CdprForwardKinematicsResult forward =
        kinematics.forward(inverse.cables.lengthM, forwardGuess);
    if (!require(forward.valid && forward.converged,
                 QStringLiteral("正运动学未收敛：%1")
                     .arg(forward.errorText))) {
        return 1;
    }
    for (int dof = 0; dof < kCdprDofCount; ++dof) {
        if (!require(near(forward.pose[static_cast<size_t>(dof)],
                          configuration.initialPlatformPose[
                              static_cast<size_t>(dof)],
                          1.0e-6),
                     QStringLiteral("正逆运动学闭环的第%1个位姿分量不一致。")
                         .arg(dof))) {
            return 1;
        }
    }

    // 两个实例使用各自复制的几何参数；修改第二套配置不能污染第一实例。
    CdprConfiguration secondConfiguration = configuration;
    secondConfiguration.cables[0].frameAnchorM.x += 0.1;
    CdprKinematics secondKinematics(secondConfiguration);
    const CdprInverseKinematicsResult second =
        secondKinematics.inverse(platformAt(configuration.initialPlatformPose));
    const CdprInverseKinematicsResult firstAgain =
        kinematics.inverse(platformAt(configuration.initialPlatformPose));
    if (!require(second.valid && firstAgain.valid
                     && !near(second.cables.lengthM[0],
                              firstAgain.cables.lengthM[0], 1.0e-4)
                     && near(firstAgain.cables.lengthM[0],
                             inverse.cables.lengthM[0], 1.0e-12),
                 QStringLiteral("运动学实例之间发生了几何状态污染。"))) {
        return 1;
    }

    qInfo().noquote()
        << QStringLiteral(
               "通过：初始绳长、绳速雅可比、正逆运动学闭环和实例隔离测试。"
               " FK迭代=%1，最大残差=%2 m")
               .arg(forward.iterations)
               .arg(forward.maximumResidualM, 0, 'g', 6);
    return 0;
}
