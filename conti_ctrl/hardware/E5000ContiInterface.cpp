#include "hardware/E5000ContiInterface.h"

#include <QThread>
#include <QStringList>

#include "LTDMC.h"

namespace {
bool checkResult(short result, const QString &operation, QString &errorMessage)
{
    if (result == 0) {
        return true;
    }
    errorMessage = QStringLiteral("%1 失败，错误码=%2").arg(operation).arg(result);
    return false;
}
}

bool E5000ContiInterface::initializeBoard(WORD requestedCardNo,
                                          short &boardCount,
                                          QString &errorMessage) const
{
    boardCount = dmc_board_init();
    // 雷赛手册规定：0 表示未检测到控制卡；正数为成功检测到的板卡数量；负数为错误码。
    if (boardCount <= 0) {
        errorMessage = QStringLiteral("dmc_board_init 失败，返回值=%1（未检测到控制卡或初始化异常）")
                           .arg(boardCount);
        return false;
    }
    if (requestedCardNo >= static_cast<WORD>(boardCount)) {
        errorMessage = QStringLiteral("请求卡号 %1 无效：当前仅检测到 %2 张控制卡（卡号范围 0-%3）")
                           .arg(requestedCardNo)
                           .arg(boardCount)
                           .arg(boardCount - 1);
        dmc_board_close();
        return false;
    }
    return true;
}

bool E5000ContiInterface::closeBoard(QString &errorMessage) const
{
    return checkResult(dmc_board_close(), QStringLiteral("dmc_board_close"), errorMessage);
}

bool E5000ContiInterface::setAxisEquivalent(WORD cardNo,
                                             WORD axis,
                                             double pulsePerUnit,
                                             QString &errorMessage) const
{
    if (pulsePerUnit <= 0.0) {
        errorMessage = QStringLiteral("轴 %1 的脉冲当量必须大于 0").arg(axis);
        return false;
    }
    return checkResult(dmc_set_equiv(cardNo, axis, pulsePerUnit),
                       QStringLiteral("dmc_set_equiv(axis=%1, equiv=%2)")
                           .arg(axis)
                           .arg(pulsePerUnit, 0, 'f', 4),
                       errorMessage);
}

bool E5000ContiInterface::clearAxisFaults(WORD cardNo,
                                          WORD axis,
                                          QString &noticeMessage,
                                          QString &errorMessage) const
{
    noticeMessage.clear();

    WORD busErrorCode = 0;
    if (!checkResult(nmc_get_errcode(cardNo, 2, &busErrorCode),
                     QStringLiteral("nmc_get_errcode"), errorMessage)) {
        return false;
    }
    if (busErrorCode != 0) {
        if (!checkResult(nmc_clear_errcode(cardNo, 2),
                         QStringLiteral("nmc_clear_errcode"), errorMessage)) {
            return false;
        }
        noticeMessage += QStringLiteral("检测到总线错误码 %1，已执行清除。 ").arg(busErrorCode);
        QThread::msleep(20);
    }

    WORD axisErrorCode = 0;
    if (!checkResult(nmc_get_axis_errcode(cardNo, axis, &axisErrorCode),
                     QStringLiteral("nmc_get_axis_errcode"), errorMessage)) {
        return false;
    }
    if (axisErrorCode != 0) {
        if (!checkResult(nmc_clear_axis_errcode(cardNo, axis),
                         QStringLiteral("nmc_clear_axis_errcode"), errorMessage)) {
            return false;
        }
        noticeMessage += QStringLiteral("检测到轴错误码 %1，已执行清除。").arg(axisErrorCode);
        QThread::msleep(20);
    }
    return true;
}

bool E5000ContiInterface::enableAxis(WORD cardNo, WORD axis, QString &errorMessage) const
{
    short enableResult = 0;
    short stateResult = 0;
    WORD stateMachine = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        enableResult = nmc_set_axis_enable(cardNo, axis);
        stateResult = nmc_get_axis_state_machine(cardNo, axis, &stateMachine);
        if (enableResult == 0 && stateResult == 0 && stateMachine == 4U) {
            return true;
        }
        QThread::msleep(50);
    }
    errorMessage = QStringLiteral("轴 %1 未进入操作使能状态：enable 返回码=%2，状态读取返回码=%3，状态机=%4")
                       .arg(axis)
                       .arg(enableResult)
                       .arg(stateResult)
                       .arg(stateMachine);
    return false;
}

bool E5000ContiInterface::disableAxis(WORD cardNo, WORD axis, QString &errorMessage) const
{
    return checkResult(nmc_set_axis_disable(cardNo, axis), QStringLiteral("nmc_set_axis_disable"), errorMessage);
}

bool E5000ContiInterface::readPositionUnit(WORD cardNo, WORD axis, double &position, QString &errorMessage) const
{
    return checkResult(dmc_get_position_unit(cardNo, axis, &position),
                       QStringLiteral("dmc_get_position_unit"), errorMessage);
}

bool E5000ContiInterface::readTargetPositionUnit(WORD cardNo, WORD axis,
                                                  double &position, QString &errorMessage) const
{
    return checkResult(dmc_get_target_position_unit(cardNo, axis, &position),
                       QStringLiteral("dmc_get_target_position_unit"), errorMessage);
}

bool E5000ContiInterface::readAxisFeedback(WORD cardNo, WORD axis, AxisFeedback &feedback) const
{
    feedback = AxisFeedback {};
    feedback.axis = axis;

    QStringList errors;
    // 位置和跟随误差由 dmc_trace_* 的同帧 PDO 样本提供，不能在这里用逐次 API 读取替代。
    const short stateResult = nmc_get_axis_state_machine(cardNo, axis, &feedback.stateMachine);
    if (stateResult != 0) {
        errors << QStringLiteral("状态机 rc=%1").arg(stateResult);
    }
    const short errorResult = nmc_get_axis_errcode(cardNo, axis, &feedback.axisErrorCode);
    if (errorResult != 0) {
        errors << QStringLiteral("轴错误码 rc=%1").arg(errorResult);
    }

    feedback.valid = errors.isEmpty();
    feedback.errorText = errors.join(QStringLiteral("；"));
    return feedback.valid;
}

bool E5000ContiInterface::startPointMove(const SingleAxisJogConfig &config,
                                         QString &errorMessage) const
{
    const double safeVelocity = qMax(qAbs(config.maxVelocityUnitPerSecond), 1e-5);

    // 与 0523_final 的单电机点位逻辑一致：若旧运动尚未结束，先减速停下再下发新目标。
    if (dmc_check_done(config.cardNo, config.axis) == 0) {
        if (!stopAxis(config.cardNo, config.axis, false, errorMessage)) {
            return false;
        }
        bool stopped = false;
        for (int retry = 0; retry < 100; ++retry) {
            if (dmc_check_done(config.cardNo, config.axis) != 0) {
                stopped = true;
                break;
            }
            QThread::msleep(10);
        }
        if (!stopped) {
            errorMessage = QStringLiteral("轴 %1 在 1 s 内未完成减速停止，拒绝下发新的点位命令").arg(config.axis);
            return false;
        }
    }

    const short clearResult = dmc_clear_stop_reason(config.cardNo, config.axis);
    const short profileResult = dmc_set_profile_unit(config.cardNo, config.axis, 0.0,
                                                      safeVelocity, 0.1, 0.1, 0.0);
    const short sCurveResult = dmc_set_s_profile(config.cardNo, config.axis, 0, 0.0);
    // 与雷赛官方上位机一致：posi_mode=0 表示相对点位运动。
    // targetPositionUnit 是相对于下发时刻当前位置的位移，正负号决定运动方向。
    const short moveResult = dmc_pmove_unit(config.cardNo, config.axis,
                                             config.targetPositionUnit, 0);
    if (clearResult == 0 && profileResult == 0 && sCurveResult == 0 && moveResult == 0) {
        return true;
    }
    errorMessage = QStringLiteral("单轴点位启动失败：clear=%1，profile=%2，s-curve=%3，pmove=%4")
                       .arg(clearResult)
                       .arg(profileResult)
                       .arg(sCurveResult)
                       .arg(moveResult);
    return false;
}

bool E5000ContiInterface::stopAxis(WORD cardNo, WORD axis, bool emergency,
                                   QString &errorMessage) const
{
    return checkResult(dmc_stop(cardNo, axis, emergency ? 1 : 0),
                       emergency ? QStringLiteral("dmc_stop(立即停止)")
                                 : QStringLiteral("dmc_stop(减速停止)"),
                       errorMessage);
}

bool E5000ContiInterface::isAxisMotionDone(WORD cardNo, WORD axis) const
{
    return dmc_check_done(cardNo, axis) != 0;
}

bool E5000ContiInterface::axisMotionDone(WORD cardNo, WORD axis, bool &done,
                                          QString &errorMessage) const
{
    WORD state = 0;
    if (!checkResult(dmc_check_done_ex(cardNo, axis, &state),
                     QStringLiteral("dmc_check_done_ex"), errorMessage)) {
        return false;
    }
    done = state != 0;
    return true;
}

bool E5000ContiInterface::axisRunMode(WORD cardNo, WORD axis, quint16 &mode,
                                      QString &errorMessage) const
{
    WORD runMode = 0;
    if (!checkResult(dmc_get_axis_run_mode(cardNo, axis, &runMode),
                     QStringLiteral("dmc_get_axis_run_mode"), errorMessage)) {
        return false;
    }
    mode = runMode;
    return true;
}

bool E5000ContiInterface::isContiMotionDone(const ContiTestConfig &config) const
{
    return dmc_check_done_multicoor(config.cardNo, config.crdNo) != 0;
}

bool E5000ContiInterface::configureAndOpen(const ContiTestConfig &config, QString &errorMessage) const
{
    WORD axes[2] = {config.activeAxis, config.holdAxis};

    // 手册要求：前瞻设置必须发生在 open_list 前。
    if (!checkResult(dmc_conti_set_lookahead_mode(config.cardNo, config.crdNo,
                                                   config.lookaheadEnabled ? 1 : 0,
                                                   0, config.pathErrorUnit, 0),
                     QStringLiteral("dmc_conti_set_lookahead_mode"), errorMessage)) {
        return false;
    }
    if (!checkResult(dmc_set_vector_profile_unit(config.cardNo, config.crdNo, 0,
                                                  config.maxVectorVelocity,
                                                  config.accelerationTimeS,
                                                  config.decelerationTimeS, 0),
                     QStringLiteral("dmc_set_vector_profile_unit"), errorMessage)) {
        return false;
    }
    if (!checkResult(dmc_set_vector_s_profile(config.cardNo, config.crdNo, 0, config.sCurveTimeS),
                     QStringLiteral("dmc_set_vector_s_profile"), errorMessage)) {
        return false;
    }
    return checkResult(dmc_conti_open_list(config.cardNo, config.crdNo, 2, axes),
                       QStringLiteral("dmc_conti_open_list"), errorMessage);
}

bool E5000ContiInterface::pushLine(const ContiTestConfig &config,
                                    const ContiPoint &point,
                                    long mark,
                                    QString &errorMessage) const
{
    WORD axes[2] = {config.activeAxis, config.holdAxis};
    double target[2] = {point.targetUnit[0], point.targetUnit[1]};
    return checkResult(dmc_conti_line_unit(config.cardNo, config.crdNo, 2, axes, target,
                                            1, mark),
                       QStringLiteral("dmc_conti_line_unit"), errorMessage);
}

bool E5000ContiInterface::start(const ContiTestConfig &config, QString &errorMessage) const
{
    return checkResult(dmc_conti_start_list(config.cardNo, config.crdNo),
                       QStringLiteral("dmc_conti_start_list"), errorMessage);
}

ContiSpeedRatioResult E5000ContiInterface::changeSpeedRatio(const ContiTestConfig &config,
                                                             QString &errorMessage) const
{
    const short result = dmc_conti_change_speed_ratio(config.cardNo, config.crdNo, config.speedRatio);
    if (result == 0) {
        return ContiSpeedRatioResult::Applied;
    }
    // 手册错误码 5049：当前段尚未规划。start_list 返回后短暂出现属于正常就绪过程，
    // 由上层在后续补点周期重试，不能作为连续插补启动失败处理。
    if (result == 5049) {
        return ContiSpeedRatioResult::NotReady;
    }
    checkResult(result, QStringLiteral("dmc_conti_change_speed_ratio"), errorMessage);
    return ContiSpeedRatioResult::Failed;
}

bool E5000ContiInterface::stop(const ContiTestConfig &config, bool emergency, QString &errorMessage) const
{
    return checkResult(dmc_conti_stop_list(config.cardNo, config.crdNo, emergency ? 1 : 0),
                       QStringLiteral("dmc_conti_stop_list"), errorMessage);
}

bool E5000ContiInterface::closeList(const ContiTestConfig &config, QString &errorMessage) const
{
    return checkResult(dmc_conti_close_list(config.cardNo, config.crdNo),
                       QStringLiteral("dmc_conti_close_list"), errorMessage);
}

long E5000ContiInterface::currentMark(const ContiTestConfig &config) const
{
    return dmc_conti_read_current_mark(config.cardNo, config.crdNo);
}

long E5000ContiInterface::remainSpace(const ContiTestConfig &config) const
{
    return dmc_conti_remain_space(config.cardNo, config.crdNo);
}

short E5000ContiInterface::runState(const ContiTestConfig &config) const
{
    return dmc_conti_get_run_state(config.cardNo, config.crdNo);
}
