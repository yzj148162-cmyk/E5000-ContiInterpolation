#include "hardware/EthercatInterface.h"

#include "LTDMC.h"
#include "hardware/PdoAccess.h"

namespace {
constexpr WORD kCardNo = 0;
constexpr qsizetype kMaxPvtTablePoints = 5000;
}

bool EthercatInterface::initializeBoard(QString &errorMessage) const
{
    const short rc = dmc_board_init();
    if (rc != 0) {
        errorMessage = QStringLiteral("dmc_board_init 失败，rc=%1").arg(rc);
        return false;
    }
    return true;
}

bool EthercatInterface::closeBoard(QString &errorMessage) const
{
    const short rc = dmc_board_close();
    if (rc != 0) {
        errorMessage = QStringLiteral("dmc_board_close 失败，rc=%1").arg(rc);
        return false;
    }
    return true;
}

bool EthercatInterface::stopAxis(const MotionConfig &config, QString &errorMessage) const
{
    const short rc = dmc_stop(kCardNo, static_cast<WORD>(config.axis), 0);
    if (rc != 0) {
        errorMessage = QStringLiteral("dmc_stop 失败，rc=%1").arg(rc);
        return false;
    }
    return true;
}

bool EthercatInterface::enableAxis(const MotionConfig &config, QString &errorMessage) const
{
    const short rc = nmc_set_axis_enable(kCardNo, static_cast<WORD>(config.axis));
    if (rc != 0) {
        errorMessage = QStringLiteral("nmc_set_axis_enable 失败，rc=%1").arg(rc);
        return false;
    }
    return true;
}

bool EthercatInterface::disableAxis(const MotionConfig &config, QString &errorMessage) const
{
    const short rc = nmc_set_axis_disable(kCardNo, static_cast<WORD>(config.axis));
    if (rc != 0) {
        errorMessage = QStringLiteral("nmc_set_axis_disable 失败，rc=%1").arg(rc);
        return false;
    }
    return true;
}

bool EthercatInterface::configureCspMode(const MotionConfig &config, QString &errorMessage) const
{
    const long opModeCsp = 8;
    short rc = nmc_set_node_od(kCardNo, config.ecatPort, config.node, 0x6060, 0x00, 9, opModeCsp);
    if (rc != 0) {
        errorMessage = QStringLiteral("写 0x6060=8 失败，rc=%1").arg(rc);
        return false;
    }

    rc = nmc_set_node_od(kCardNo, config.ecatPort, config.node, 0x60C2, 0x01, 8, config.cspInterpPeriodMs);
    if (rc != 0) {
        errorMessage = QStringLiteral("写 0x60C2:01 失败，rc=%1").arg(rc);
        return false;
    }

    rc = nmc_set_node_od(kCardNo, config.ecatPort, config.node, 0x60C2, 0x02, 8, config.cspInterpIndex);
    if (rc != 0) {
        errorMessage = QStringLiteral("写 0x60C2:02 失败，rc=%1").arg(rc);
        return false;
    }

    return true;
}

bool EthercatInterface::writeCspTargetPositionRaw(const MotionConfig &config,
                                                  qint32 targetPosRaw,
                                                  QString &errorMessage) const
{
    const PdoAccess access = PdoAccess::fromMotionConfig(config);
    const short rc = nmc_write_rxpdo_extra(kCardNo, config.ecatPort, access.rxTargetPosAddr, 2, targetPosRaw);
    if (rc != 0) {
        errorMessage = QStringLiteral("写 CSP 目标位置失败，rc=%1").arg(rc);
        return false;
    }
    return true;
}

bool EthercatInterface::startPvtsMotion(const MotionConfig &config,
                                        const QVector<TrajectoryPoint> &trajectory,
                                        QString &errorMessage) const
{
    if (trajectory.size() < 2) {
        errorMessage = QStringLiteral("PVTS 轨迹点数不足，至少需要 2 个点。");
        return false;
    }
    if (trajectory.size() > kMaxPvtTablePoints) {
        errorMessage = QStringLiteral("PVTS 轨迹点数超限：count=%1，控制卡上限=%2。")
                           .arg(trajectory.size())
                           .arg(kMaxPvtTablePoints);
        return false;
    }

    QVector<double> timeS;
    QVector<double> posUnit;
    timeS.reserve(trajectory.size());
    posUnit.reserve(trajectory.size());

    const double firstTimeS = trajectory.first().timeS;
    const qint32 firstPosRaw = trajectory.first().targetPosRaw;

    for (const TrajectoryPoint &point : trajectory) {
        // 手册要求 PVTS 表中的时间和位置都以首点为参考。
        timeS.push_back(point.timeS - firstTimeS);
        posUnit.push_back(static_cast<double>(point.targetPosRaw - firstPosRaw) / config.rawPerDeg);
    }

    timeS[0] = 0.0;
    posUnit[0] = 0.0;

    const double velBegin = static_cast<double>(trajectory.first().targetVelRaw) / config.rawPerDeg;
    const double velEnd = static_cast<double>(trajectory.last().targetVelRaw) / config.rawPerDeg;

    const short rcTable = dmc_pvts_table_unit(kCardNo,
                                              static_cast<WORD>(config.axis),
                                              static_cast<DWORD>(trajectory.size()),
                                              timeS.data(),
                                              posUnit.data(),
                                              velBegin,
                                              velEnd);
    if (rcTable != 0) {
        errorMessage = QStringLiteral("dmc_pvts_table_unit 失败，rc=%1").arg(rcTable);
        return false;
    }

    WORD axisList[1] = { static_cast<WORD>(config.axis) };
    const short rcMove = dmc_pvt_move(kCardNo, 1, axisList);
    if (rcMove != 0) {
        errorMessage = QStringLiteral("dmc_pvt_move 失败，rc=%1").arg(rcMove);
        return false;
    }

    return true;
}

bool EthercatInterface::readPvtRunIndex(const MotionConfig &config,
                                        quint32 &runIndex,
                                        QString &errorMessage) const
{
    DWORD index = 0;
    const short rc = dmc_pvt_get_run_index(kCardNo, static_cast<WORD>(config.axis), &index);
    if (rc != 0) {
        errorMessage = QStringLiteral("dmc_pvt_get_run_index 失败，rc=%1").arg(rc);
        return false;
    }
    runIndex = static_cast<quint32>(index);
    return true;
}

bool EthercatInterface::isAxisMotionDone(const MotionConfig &config, bool &done, QString &errorMessage) const
{
    WORD state = 0;
    const short rc = dmc_check_done_ex(kCardNo, static_cast<WORD>(config.axis), &state);
    if (rc != 0) {
        errorMessage = QStringLiteral("dmc_check_done_ex 失败，rc=%1").arg(rc);
        return false;
    }
    done = (state != 0);
    return true;
}

bool EthercatInterface::readFeedback(const MotionConfig &config, FeedbackData &feedback, QString &errorMessage) const
{
    if (config.mode == MotionMode::CSP) {
        const PdoAccess access = PdoAccess::fromMotionConfig(config);

        WORD statusValue = 0;
        const short rcStatus =
            nmc_read_txpdo_extra_short(kCardNo, config.ecatPort, access.txStatusAddr, 1, &statusValue);
        if (rcStatus != 0) {
            errorMessage = QStringLiteral("读 0x6041 失败，rc=%1").arg(rcStatus);
            return false;
        }

        int modeRaw = 0;
        const short rcMode = nmc_read_txpdo_extra(kCardNo, config.ecatPort, access.txModeAddr, 1, &modeRaw);
        if (rcMode != 0) {
            errorMessage = QStringLiteral("读 0x6061 失败，rc=%1").arg(rcMode);
            return false;
        }

        int posRaw = 0;
        const short rcPos = nmc_read_txpdo_extra(kCardNo, config.ecatPort, access.txActualPosAddr, 2, &posRaw);
        if (rcPos != 0) {
            errorMessage = QStringLiteral("读 0x6064 失败，rc=%1").arg(rcPos);
            return false;
        }

        feedback.statusWord = statusValue;
        feedback.modeDisplay = static_cast<qint8>(modeRaw & 0xFF);
        feedback.actualPosRaw = static_cast<qint32>(posRaw);
        feedback.errorRaw = feedback.targetPosRaw - feedback.actualPosRaw;
        feedback.fault = (statusValue & 0x0008) != 0;
        return true;
    }

    double posDeg = 0.0;
    const short rc = dmc_get_position_unit(kCardNo, static_cast<WORD>(config.axis), &posDeg);
    if (rc != 0) {
        errorMessage = QStringLiteral("读 PVT 当前位置失败，rc=%1").arg(rc);
        return false;
    }

    feedback.actualPosRaw = qRound(posDeg * config.rawPerDeg);
    feedback.errorRaw = feedback.targetPosRaw - feedback.actualPosRaw;
    feedback.modeDisplay = 0;
    feedback.statusWord = 0;
    feedback.fault = false;
    return true;
}

bool EthercatInterface::readStartPositionRaw(const MotionConfig &config,
                                             qint32 &startPosRaw,
                                             QString &errorMessage) const
{
    FeedbackData feedback;
    if (!readFeedback(config, feedback, errorMessage)) {
        return false;
    }
    startPosRaw = feedback.actualPosRaw;
    return true;
}
