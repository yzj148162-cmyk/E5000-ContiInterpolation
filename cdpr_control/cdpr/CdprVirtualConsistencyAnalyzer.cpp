#include "cdpr/CdprVirtualConsistencyAnalyzer.h"

#include "cdpr/CdprConfiguration.h"
#include "cdpr/CdprKinematics.h"
#include "common/ContiTypes.h"

#include <QDir>
#include <QFile>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;
// The eight independently measured cable lengths are generally not exactly
// compatible with one six-DOF pose. Keep every finite least-squares solution,
// and classify its largest cable residual using strict (10 nm) and engineering
// (0.01 mm) quality thresholds instead of discarding the whole Trace frame.
constexpr double kStrictCableResidualToleranceM = 1.0e-8;
constexpr double kEngineeringCableResidualToleranceM = 1.0e-5;

struct ExpectedTrajectory
{
    QVector<double> timeS;
    QVector<CdprVector6> pose;
    std::array<QVector<double>, kCdprCableCount> positionDegree;
    std::array<QVector<double>, kCdprCableCount> velocityDegreePerSecond;
};

bool readJsonObject(const QString &path, QJsonObject &object, QString &error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        error = QStringLiteral("无法读取%1：%2").arg(path, file.errorString());
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (!document.isObject()) {
        error = QStringLiteral("JSON解析失败：%1（偏移%2）")
                    .arg(parseError.errorString()).arg(parseError.offset);
        return false;
    }
    object = document.object();
    return true;
}

bool readExpectedTrajectory(const QString &path, const CdprConfiguration &configuration,
                            ExpectedTrajectory &result, QString &error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = QStringLiteral("无法读取期望轨迹：%1").arg(file.errorString());
        return false;
    }
    QTextStream stream(&file);
    const QStringList header = stream.readLine().trimmed().split(',');
    QHash<QString, int> columns;
    for (int index = 0; index < header.size(); ++index) {
        columns.insert(header.at(index).trimmed(), index);
    }
    const QStringList poseColumns {
        QStringLiteral("platform_x_m"), QStringLiteral("platform_y_m"),
        QStringLiteral("platform_z_m"), QStringLiteral("platform_roll_rad"),
        QStringLiteral("platform_pitch_rad"), QStringLiteral("platform_yaw_rad")
    };
    if (!columns.contains(QStringLiteral("time_s"))) {
        error = QStringLiteral("expected_trajectory.csv缺少time_s列。");
        return false;
    }
    for (const QString &name : poseColumns) {
        if (!columns.contains(name)) {
            error = QStringLiteral("expected_trajectory.csv缺少末端位姿列%1；请使用新版运行记录。").arg(name);
            return false;
        }
    }
    std::array<int, kCdprCableCount> positionColumns {};
    std::array<int, kCdprCableCount> velocityColumns {};
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        const int axis = configuration.cables[static_cast<size_t>(cable)].axis;
        const QString positionName = QStringLiteral("axis%1_planned_position_deg").arg(axis);
        const QString velocityName = QStringLiteral("axis%1_planned_velocity_deg_per_s").arg(axis);
        if (!columns.contains(positionName) || !columns.contains(velocityName)) {
            error = QStringLiteral("期望轨迹缺少映射轴%1的位置或速度列。").arg(axis);
            return false;
        }
        positionColumns[static_cast<size_t>(cable)] = columns.value(positionName);
        velocityColumns[static_cast<size_t>(cable)] = columns.value(velocityName);
    }

    const int timeColumn = columns.value(QStringLiteral("time_s"));
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList values = line.split(',');
        if (values.size() != header.size()) {
            error = QStringLiteral("期望轨迹存在列数不一致的行。");
            return false;
        }
        bool valid = false;
        const double timeS = values.at(timeColumn).toDouble(&valid);
        if (!valid || (!result.timeS.isEmpty() && timeS <= result.timeS.constLast())) {
            error = QStringLiteral("期望轨迹时间列无效或不递增。");
            return false;
        }
        CdprVector6 pose {};
        for (int dof = 0; dof < kCdprDofCount; ++dof) {
            pose[static_cast<size_t>(dof)] = values.at(columns.value(poseColumns.at(dof))).toDouble(&valid);
            if (!valid) {
                error = QStringLiteral("期望轨迹末端位姿存在无效值。");
                return false;
            }
        }
        result.timeS.append(timeS);
        result.pose.append(pose);
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            const size_t offset = static_cast<size_t>(cable);
            const double position = values.at(positionColumns[offset]).toDouble(&valid);
            if (!valid) {
                error = QStringLiteral("期望轨迹轴位置存在无效值。");
                return false;
            }
            const double velocity = values.at(velocityColumns[offset]).toDouble(&valid);
            if (!valid) {
                error = QStringLiteral("期望轨迹轴速度存在无效值。");
                return false;
            }
            result.positionDegree[offset].append(position);
            result.velocityDegreePerSecond[offset].append(velocity);
        }
    }
    if (result.timeS.size() < 2) {
        error = QStringLiteral("期望轨迹点数不足。");
        return false;
    }
    return true;
}

bool readTraceFrames(const QString &path, QVector<TraceTelemetryFrame> &frames,
                     QString &error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        error = QStringLiteral("无法读取Trace二进制文件：%1").arg(file.errorString());
        return false;
    }
    const qint64 frameBytes = static_cast<qint64>(sizeof(TraceTelemetryFrame));
    if (file.size() <= 0 || file.size() % frameBytes != 0) {
        error = QStringLiteral("Trace二进制长度与当前记录格式不匹配。");
        return false;
    }
    const qint64 count = file.size() / frameBytes;
    frames.resize(static_cast<qsizetype>(count));
    const qint64 bytes = file.read(reinterpret_cast<char *>(frames.data()), file.size());
    if (bytes != file.size()) {
        error = QStringLiteral("读取Trace二进制数据不完整。");
        return false;
    }
    return true;
}

int axisIndex(const TraceTelemetryFrame &frame, quint16 axis)
{
    for (int index = 0; index < frame.axisCount; ++index) {
        if (frame.axes[index] == axis) {
            return index;
        }
    }
    return -1;
}

bool frameHasMappedAxes(const TraceTelemetryFrame &frame,
                         const CdprConfiguration &configuration)
{
    for (const CdprCableAxisConfig &cable : configuration.cables) {
        const int index = axisIndex(frame, static_cast<quint16>(cable.axis));
        if (index < 0 || (frame.validAxisMask & static_cast<quint8>(1U << index)) == 0U) {
            return false;
        }
    }
    return true;
}

double pulseToDegree(qint32 pulse)
{
    return static_cast<double>(pulse) / MotorUnit::kPhysicalPulsesPerDegree;
}

bool interpolateScalar(const QVector<double> &timeS, const QVector<double> &values,
                       double time, double &result)
{
    if (time < timeS.constFirst() || time > timeS.constLast() || values.size() != timeS.size()) {
        return false;
    }
    const auto upper = std::upper_bound(timeS.cbegin(), timeS.cend(), time);
    if (upper == timeS.cbegin()) {
        result = values.constFirst();
        return true;
    }
    if (upper == timeS.cend()) {
        result = values.constLast();
        return true;
    }
    const int high = static_cast<int>(upper - timeS.cbegin());
    const int low = high - 1;
    const double fraction = (time - timeS.at(low)) / (timeS.at(high) - timeS.at(low));
    result = values.at(low) + fraction * (values.at(high) - values.at(low));
    return true;
}

bool interpolateExpected(const ExpectedTrajectory &trajectory, double timeS,
                         CdprVector6 &pose, CdprVector8 &positionDegree,
                         CdprVector8 &velocityDegreePerSecond)
{
    if (timeS < trajectory.timeS.constFirst() || timeS > trajectory.timeS.constLast()) {
        return false;
    }
    const auto upper = std::upper_bound(trajectory.timeS.cbegin(),
                                        trajectory.timeS.cend(), timeS);
    const int high = upper == trajectory.timeS.cend()
        ? trajectory.timeS.size() - 1
        : static_cast<int>(upper - trajectory.timeS.cbegin());
    const int low = high == 0 ? 0 : high - 1;
    const double denominator = trajectory.timeS.at(high) - trajectory.timeS.at(low);
    const double fraction = denominator > 0.0
        ? (timeS - trajectory.timeS.at(low)) / denominator : 0.0;
    for (int dof = 0; dof < kCdprDofCount; ++dof) {
        pose[static_cast<size_t>(dof)] = trajectory.pose.at(low)[static_cast<size_t>(dof)]
            + fraction * (trajectory.pose.at(high)[static_cast<size_t>(dof)]
                          - trajectory.pose.at(low)[static_cast<size_t>(dof)]);
    }
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        const size_t offset = static_cast<size_t>(cable);
        positionDegree[offset] = trajectory.positionDegree[offset].at(low)
            + fraction * (trajectory.positionDegree[offset].at(high)
                          - trajectory.positionDegree[offset].at(low));
        velocityDegreePerSecond[offset] = trajectory.velocityDegreePerSecond[offset].at(low)
            + fraction * (trajectory.velocityDegreePerSecond[offset].at(high)
                          - trajectory.velocityDegreePerSecond[offset].at(low));
    }
    return true;
}

std::array<std::array<double, 3>, 3> rotationFromPose(const CdprVector6 &pose)
{
    const double roll = pose[3];
    const double pitch = pose[4];
    const double yaw = pose[5];
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
             {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
             {-sp, cp * sr, cp * cr}}};
}

double orientationErrorDegree(const CdprVector6 &planned, const CdprVector6 &actual)
{
    const auto rp = rotationFromPose(planned);
    const auto ra = rotationFromPose(actual);
    double trace = 0.0;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            trace += rp[row][column] * ra[row][column];
        }
    }
    const double cosine = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / kPi;
}

double median(QVector<double> values)
{
    if (values.isEmpty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const int middle = values.size() / 2;
    return values.size() % 2 == 0
        ? (values.at(middle - 1) + values.at(middle)) * 0.5
        : values.at(middle);
}

int closestFrameIndex(const QVector<TraceTelemetryFrame> &frames, quint64 sequence)
{
    int best = -1;
    quint64 bestDistance = std::numeric_limits<quint64>::max();
    for (int index = 0; index < frames.size(); ++index) {
        const quint64 value = frames.at(index).traceSequence;
        const quint64 distance = value > sequence ? value - sequence : sequence - value;
        if (distance < bestDistance) {
            bestDistance = distance;
            best = index;
        }
    }
    return best;
}

bool findPvtStartSequence(const QVector<TraceTelemetryFrame> &frames,
                          const ExpectedTrajectory &trajectory,
                          const CdprConfiguration &configuration,
                          quint64 seedSequence, int tracePeriodUs,
                          quint64 &sequence, double &rmseDegree)
{
    const int seedIndex = closestFrameIndex(frames, seedSequence);
    if (seedIndex < 0 || tracePeriodUs <= 0) {
        return false;
    }

    // dmc_pvt_move() returning on the host is not the hardware start instant.
    // Locate the first type05 pulse change, then subtract the planned time at
    // which the first half-pulse displacement is expected.  This gives a
    // robust centre for the detailed curve-matching window even when the card
    // starts hundreds of milliseconds after the host request.
    const TraceTelemetryFrame &seedFrame = frames.at(seedIndex);
    int commandMovementIndex = -1;
    for (int frameIndex = seedIndex + 1; frameIndex < frames.size(); ++frameIndex) {
        const TraceTelemetryFrame &frame = frames.at(frameIndex);
        if (!frameHasMappedAxes(frame, configuration)) {
            continue;
        }
        bool changed = false;
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            const quint16 axis = static_cast<quint16>(
                configuration.cables[static_cast<size_t>(cable)].axis);
            const int seedAxisIndex = axisIndex(seedFrame, axis);
            const int frameAxisIndex = axisIndex(frame, axis);
            if (seedAxisIndex >= 0 && frameAxisIndex >= 0
                && frame.commandPulse[frameAxisIndex]
                    != seedFrame.commandPulse[seedAxisIndex]) {
                changed = true;
                break;
            }
        }
        if (changed) {
            commandMovementIndex = frameIndex;
            break;
        }
    }
    if (commandMovementIndex < 0) {
        return false;
    }

    int plannedMovementPoint = -1;
    const double halfPulseDegree = 0.5 / MotorUnit::kPhysicalPulsesPerDegree;
    for (int point = 0; point < trajectory.timeS.size(); ++point) {
        bool changed = false;
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            if (std::abs(trajectory.positionDegree[static_cast<size_t>(cable)].at(point))
                >= halfPulseDegree) {
                changed = true;
                break;
            }
        }
        if (changed) {
            plannedMovementPoint = point;
            break;
        }
    }
    if (plannedMovementPoint < 0) {
        return false;
    }

    const double periodS = tracePeriodUs / 1000000.0;
    const qint64 plannedMovementFrames = static_cast<qint64>(std::llround(
        trajectory.timeS.at(plannedMovementPoint) / periodS));
    const quint64 movementSequence = frames.at(commandMovementIndex).traceSequence;
    const quint64 estimatedStartSequence = movementSequence
        > static_cast<quint64>(std::max<qint64>(0, plannedMovementFrames))
        ? movementSequence - static_cast<quint64>(plannedMovementFrames)
        : seedSequence;
    const int estimatedStartIndex = closestFrameIndex(frames, estimatedStartSequence);
    const int searchRadius = std::max(40, 100000 / tracePeriodUs);
    const int before = std::max(std::max(0, seedIndex - 8),
                                estimatedStartIndex - searchRadius);
    const int after = std::min(static_cast<int>(frames.size()) - 1,
                               estimatedStartIndex + searchRadius);
    double bestMeanSse = std::numeric_limits<double>::infinity();
    int bestIndex = -1;
    const int minimumComparedFrames = std::max(
        10, static_cast<int>(trajectory.timeS.constLast() / periodS * 0.8));
    for (int candidate = before; candidate <= after; ++candidate) {
        if (!frameHasMappedAxes(frames.at(candidate), configuration)) {
            continue;
        }
        double sse = 0.0;
        int count = 0;
        for (int frameIndex = candidate; frameIndex < frames.size(); ++frameIndex) {
            const TraceTelemetryFrame &frame = frames.at(frameIndex);
            const double timeS = static_cast<double>(frame.traceSequence
                - frames.at(candidate).traceSequence) * periodS;
            if (timeS > trajectory.timeS.constLast()) {
                break;
            }
            CdprVector6 ignoredPose {};
            CdprVector8 plannedPosition {};
            CdprVector8 ignoredVelocity {};
            if (!interpolateExpected(trajectory, timeS, ignoredPose,
                                     plannedPosition, ignoredVelocity)
                || !frameHasMappedAxes(frame, configuration)) {
                continue;
            }
            for (int cable = 0; cable < kCdprCableCount; ++cable) {
                const quint16 axis = static_cast<quint16>(configuration.cables[static_cast<size_t>(cable)].axis);
                const int startAxisIndex = axisIndex(frames.at(candidate), axis);
                const int currentAxisIndex = axisIndex(frame, axis);
                const double commandDelta = pulseToDegree(frame.commandPulse[currentAxisIndex])
                    - pulseToDegree(frames.at(candidate).commandPulse[startAxisIndex]);
                const double error = commandDelta - plannedPosition[static_cast<size_t>(cable)];
                sse += error * error;
                ++count;
            }
        }
        const double meanSse = count > 0 ? sse / static_cast<double>(count)
                                         : std::numeric_limits<double>::infinity();
        if (count >= kCdprCableCount * minimumComparedFrames
            && meanSse < bestMeanSse) {
            bestMeanSse = meanSse;
            bestIndex = candidate;
        }
    }
    if (bestIndex < 0) {
        return false;
    }
    sequence = frames.at(bestIndex).traceSequence;
    rmseDegree = std::sqrt(bestMeanSse);
    return true;
}

double fitGlobalDelay(const QVector<TraceTelemetryFrame> &frames, int startIndex,
                      quint64 startSequence, const ExpectedTrajectory &trajectory,
                      const CdprConfiguration &configuration, int tracePeriodUs,
                      double initialDelayMs, bool &fitted)
{
    fitted = false;
    if (startIndex < 0 || !frameHasMappedAxes(frames.at(startIndex), configuration)) {
        return initialDelayMs;
    }
    const TraceTelemetryFrame &baseline = frames.at(startIndex);
    const double periodS = tracePeriodUs / 1000000.0;
    const double searchStepMs = std::min(0.25, static_cast<double>(tracePeriodUs) / 1000.0);
    double bestDelay = initialDelayMs;
    double bestSse = std::numeric_limits<double>::infinity();
    for (double delayMs = std::max(0.0, initialDelayMs - 5.0);
         delayMs <= initialDelayMs + 5.0001; delayMs += searchStepMs) {
        double sse = 0.0;
        int count = 0;
        for (int frameIndex = startIndex; frameIndex < frames.size(); ++frameIndex) {
            const TraceTelemetryFrame &frame = frames.at(frameIndex);
            if (!frameHasMappedAxes(frame, configuration) || frame.traceSequence < startSequence) {
                continue;
            }
            const double plannedTimeS = static_cast<double>(frame.traceSequence - startSequence)
                * periodS - delayMs / 1000.0;
            CdprVector6 ignoredPose {};
            CdprVector8 plannedPosition {};
            CdprVector8 plannedVelocity {};
            if (!interpolateExpected(trajectory, plannedTimeS, ignoredPose,
                                     plannedPosition, plannedVelocity)) {
                continue;
            }
            double excitation = 0.0;
            for (double velocity : plannedVelocity) {
                excitation += velocity * velocity;
            }
            if (excitation < 1.0) {
                continue;
            }
            for (int cable = 0; cable < kCdprCableCount; ++cable) {
                const quint16 axis = static_cast<quint16>(configuration.cables[static_cast<size_t>(cable)].axis);
                const int baselineIndex = axisIndex(baseline, axis);
                const int frameAxisIndex = axisIndex(frame, axis);
                const double actualDelta = pulseToDegree(frame.actualPulse[frameAxisIndex])
                    - pulseToDegree(baseline.actualPulse[baselineIndex]);
                const double error = actualDelta - plannedPosition[static_cast<size_t>(cable)];
                sse += error * error;
                ++count;
            }
        }
        if (count >= kCdprCableCount * 20 && sse < bestSse) {
            bestSse = sse;
            bestDelay = delayMs;
            fitted = true;
        }
    }
    return bestDelay;
}
}

CdprVirtualConsistencyAnalysisResult CdprVirtualConsistencyAnalyzer::analyze(
    const QString &runDirectory)
{
    CdprVirtualConsistencyAnalysisResult result;
    const QDir directory(runDirectory);
    auto fail = [&](const QString &message) {
        result.errorText = message;
        result.summary = QStringLiteral("虚拟运动学一致性分析失败：%1").arg(message);
        return result;
    };
    if (!directory.exists()) {
        return fail(QStringLiteral("运行记录目录不存在。"));
    }

    CdprConfiguration configuration;
    QStringList configurationErrors;
    if (!CdprConfigurationFile::load(directory.filePath(QStringLiteral("configuration_snapshot.json")),
                                     configuration, configurationErrors)) {
        return fail(QStringLiteral("配置快照无效：%1").arg(configurationErrors.join(QStringLiteral("；"))));
    }
    QJsonObject metadata;
    QJsonObject context;
    QString error;
    if (!readJsonObject(directory.filePath(QStringLiteral("metadata.json")), metadata, error)
        || !readJsonObject(directory.filePath(QStringLiteral("run_context.json")), context, error)) {
        return fail(error);
    }
    if (metadata.value(QStringLiteral("formatVersion")).toInt() != 3
        || metadata.value(QStringLiteral("frameBytes")).toInt()
            != static_cast<int>(sizeof(TraceTelemetryFrame))) {
        return fail(QStringLiteral("Trace记录格式版本或帧大小不匹配。"));
    }
    const int tracePeriodUs = context.value(QStringLiteral("trace_period_us")).toInt(
        metadata.value(QStringLiteral("traceSamplePeriodUs")).toInt());
    const double winchRadiusM = context.value(QStringLiteral("winch_radius_m")).toDouble();
    if (tracePeriodUs <= 0 || !std::isfinite(winchRadiusM) || winchRadiusM <= 0.0) {
        return fail(QStringLiteral("运行上下文中的Trace周期或虚拟绞盘半径无效。"));
    }

    ExpectedTrajectory expected;
    if (!readExpectedTrajectory(directory.filePath(QStringLiteral("expected_trajectory.csv")),
                                configuration, expected, error)) {
        return fail(error);
    }
    QVector<TraceTelemetryFrame> frames;
    if (!readTraceFrames(directory.filePath(QStringLiteral("trace_position.bin")), frames, error)) {
        return fail(error);
    }
    result.inputFrameCount = static_cast<quint64>(frames.size());

    quint64 gaps = 0;
    for (int index = 1; index < frames.size(); ++index) {
        if (frames.at(index).traceSequence != frames.at(index - 1).traceSequence + 1U) {
            ++gaps;
        }
    }
    result.sequenceGapCount = gaps;

    const QString mode = context.value(QStringLiteral("mode")).toString();
    quint64 startSequence = 0;
    double pvtAnchorRmse = 0.0;
    if (mode == QStringLiteral("offline_pvt")) {
        const quint64 seed = static_cast<quint64>(context.value(
            QStringLiteral("pvt_start_request_pre_trace_sequence")).toDouble());
        if (!findPvtStartSequence(frames, expected, configuration, seed,
                                  tracePeriodUs, startSequence, pvtAnchorRmse)) {
            return fail(QStringLiteral("无法由type05指令位置与PVT规划表确定启动Trace锚点。"));
        }
    } else if (mode == QStringLiteral("velocity_control")) {
        startSequence = static_cast<quint64>(context.value(
            QStringLiteral("velocity_start_trace_sequence")).toDouble());
        if (startSequence == 0) {
            return fail(QStringLiteral("实时速度闭环记录缺少启动Trace锚点。"));
        }
    } else {
        return fail(QStringLiteral("运行模式未知，无法选择启动锚点策略。"));
    }
    const int startIndex = closestFrameIndex(frames, startSequence);
    if (startIndex < 0 || !frameHasMappedAxes(frames.at(startIndex), configuration)) {
        return fail(QStringLiteral("启动锚点附近没有完整有效的八轴Trace帧。"));
    }
    result.startTraceSequence = frames.at(startIndex).traceSequence;

    QVector<double> calibratedDelays;
    const QJsonArray delayArray = context.value(QStringLiteral("trace_delay_calibration")).toArray();
    for (const QJsonValue &itemValue : delayArray) {
        const QJsonObject item = itemValue.toObject();
        if (item.value(QStringLiteral("calibrated")).toBool()) {
            const double value = item.value(QStringLiteral("applied_delay_ms")).toDouble();
            if (std::isfinite(value) && value >= 0.0) {
                calibratedDelays.append(value);
            }
        }
    }
    const double initialDelayMs = median(calibratedDelays);
    bool globalDelayFitted = false;
    result.globalDelayMs = calibratedDelays.isEmpty()
        ? 0.0 : fitGlobalDelay(frames, startIndex, result.startTraceSequence,
                               expected, configuration, tracePeriodUs,
                               initialDelayMs, globalDelayFitted);

    CdprKinematics kinematics(configuration);
    CdprPlatformState6 initialPlatform;
    initialPlatform.pose = expected.pose.constFirst();
    initialPlatform.poseValid = true;
    const CdprInverseKinematicsResult initialInverse = kinematics.inverse(initialPlatform);
    if (!initialInverse.valid) {
        return fail(QStringLiteral("初始规划位姿逆运动学失败：%1").arg(initialInverse.errorText));
    }
    const TraceTelemetryFrame &baseline = frames.at(startIndex);
    const QString analysisDirectory = directory.filePath(QStringLiteral("analysis"));
    if (!QDir().mkpath(analysisDirectory)) {
        return fail(QStringLiteral("无法创建分析输出目录。"));
    }
    QSaveFile csvFile(QDir(analysisDirectory).filePath(QStringLiteral("virtual_consistency.csv")));
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return fail(QStringLiteral("无法创建分析CSV：%1").arg(csvFile.errorString()));
    }
    QTextStream csv(&csvFile);
    csv.setRealNumberNotation(QTextStream::FixedNotation);
    csv.setRealNumberPrecision(9);
    csv << "trace_sequence,trace_run_time_s,planned_time_s,fk_solution_valid,"
           "fk_engineering_residual,fk_strict_residual,"
           "fk_rms_residual_m,fk_max_residual_m,"
           "plan_x_m,plan_y_m,plan_z_m,plan_roll_rad,plan_pitch_rad,plan_yaw_rad,"
           "virtual_x_m,virtual_y_m,virtual_z_m,virtual_roll_rad,virtual_pitch_rad,virtual_yaw_rad,"
           "translation_error_mm,orientation_error_deg";
    for (int cable = 0; cable < kCdprCableCount; ++cable) {
        csv << ",axis" << configuration.cables[static_cast<size_t>(cable)].axis
            << "_actual_deg,cable" << cable << "_virtual_length_m";
    }
    csv << '\n';

    const double periodS = tracePeriodUs / 1000000.0;
    double squaredTranslationError = 0.0;
    CdprVector6 previousPose = expected.pose.constFirst();
    bool previousPoseValid = false;
    for (int frameIndex = startIndex; frameIndex < frames.size(); ++frameIndex) {
        const TraceTelemetryFrame &frame = frames.at(frameIndex);
        if (!frameHasMappedAxes(frame, configuration) || frame.traceSequence < result.startTraceSequence) {
            continue;
        }
        const double traceRunTimeS = static_cast<double>(frame.traceSequence
            - result.startTraceSequence) * periodS;
        const double plannedTimeS = traceRunTimeS - result.globalDelayMs / 1000.0;
        CdprVector6 plannedPose {};
        CdprVector8 plannedPosition {};
        CdprVector8 plannedVelocity {};
        if (!interpolateExpected(expected, plannedTimeS, plannedPose,
                                 plannedPosition, plannedVelocity)) {
            continue;
        }
        ++result.eligibleFrameCount;
        CdprVector8 actualLengths = initialInverse.cables.lengthM;
        std::array<double, kCdprCableCount> actualDegrees {};
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            const CdprCableAxisConfig &mapping = configuration.cables[static_cast<size_t>(cable)];
            const int baselineAxisIndex = axisIndex(baseline, static_cast<quint16>(mapping.axis));
            const int frameAxisIndex = axisIndex(frame, static_cast<quint16>(mapping.axis));
            actualDegrees[static_cast<size_t>(cable)] = pulseToDegree(frame.actualPulse[frameAxisIndex])
                - pulseToDegree(baseline.actualPulse[baselineAxisIndex]);
            actualLengths[static_cast<size_t>(cable)] += mapping.direction * winchRadiusM
                * actualDegrees[static_cast<size_t>(cable)] * kPi / 180.0;
        }
        const CdprVector6 guess = previousPoseValid ? previousPose : plannedPose;
        // Keep the FK solver's strict stopping criterion so that it still
        // searches for the best pose.  Residual thresholds below classify the
        // result; they do not erase a finite least-squares solution.
        const CdprForwardKinematicsResult forward = kinematics.forward(actualLengths, guess);
        const bool solutionValid = forward.valid
            && std::all_of(forward.pose.cbegin(), forward.pose.cend(),
                           [](double value) { return std::isfinite(value); })
            && std::isfinite(forward.rmsResidualM)
            && std::isfinite(forward.maximumResidualM)
            && forward.rmsResidualM >= 0.0
            && forward.maximumResidualM >= 0.0;
        const bool engineeringResidual = solutionValid
            && forward.maximumResidualM <= kEngineeringCableResidualToleranceM;
        const bool strictResidual = solutionValid
            && forward.maximumResidualM <= kStrictCableResidualToleranceM;
        if (engineeringResidual) {
            ++result.engineeringResidualFrameCount;
        }
        if (strictResidual) {
            ++result.strictResidualFrameCount;
        }
        if (!solutionValid) {
            ++result.rejectedFrameCount;
        }
        if (solutionValid) {
            result.maximumSolvedCableResidualUm = std::max(
                result.maximumSolvedCableResidualUm,
                forward.maximumResidualM * 1.0e6);
        }
        csv << frame.traceSequence << ',' << traceRunTimeS << ',' << plannedTimeS << ','
            << (solutionValid ? 1 : 0) << ',' << (engineeringResidual ? 1 : 0) << ','
            << (strictResidual ? 1 : 0) << ','
            << forward.rmsResidualM << ',' << forward.maximumResidualM;
        for (double value : plannedPose) {
            csv << ',' << value;
        }
        if (solutionValid) {
            double translationSquared = 0.0;
            for (int index = 0; index < 3; ++index) {
                const double delta = forward.pose[static_cast<size_t>(index)]
                    - plannedPose[static_cast<size_t>(index)];
                translationSquared += delta * delta;
            }
            const double translationMm = std::sqrt(translationSquared) * 1000.0;
            const double orientationDegree = orientationErrorDegree(plannedPose, forward.pose);
            for (double value : forward.pose) {
                csv << ',' << value;
            }
            csv << ',' << translationMm << ',' << orientationDegree;
            result.maximumTranslationErrorMm = std::max(result.maximumTranslationErrorMm, translationMm);
            result.maximumOrientationErrorDegree = std::max(result.maximumOrientationErrorDegree,
                                                             orientationDegree);
            squaredTranslationError += translationMm * translationMm;
            previousPose = forward.pose;
            previousPoseValid = true;
        } else {
            for (int index = 0; index < kCdprDofCount; ++index) {
                csv << ',';
            }
            csv << ",,";
        }
        for (int cable = 0; cable < kCdprCableCount; ++cable) {
            csv << ',' << actualDegrees[static_cast<size_t>(cable)]
                << ',' << actualLengths[static_cast<size_t>(cable)];
        }
        csv << '\n';
        if (solutionValid) {
            ++result.analyzedFrameCount;
        }
    }
    if (!csvFile.commit()) {
        return fail(QStringLiteral("提交分析CSV失败：%1").arg(csvFile.errorString()));
    }
    if (result.analyzedFrameCount == 0) {
        return fail(QStringLiteral("没有可完成正运动学的有效Trace帧。"));
    }
    result.rmsTranslationErrorMm = std::sqrt(
        squaredTranslationError / static_cast<double>(result.analyzedFrameCount));

    QJsonObject summary;
    summary.insert(QStringLiteral("mode"), mode);
    summary.insert(QStringLiteral("inputFrameCount"), static_cast<double>(result.inputFrameCount));
    summary.insert(QStringLiteral("eligibleFrameCount"), static_cast<double>(result.eligibleFrameCount));
    summary.insert(QStringLiteral("analyzedFrameCount"), static_cast<double>(result.analyzedFrameCount));
    summary.insert(QStringLiteral("engineeringAcceptedFrameCount"),
                   static_cast<double>(result.engineeringResidualFrameCount));
    summary.insert(QStringLiteral("strictResidualFrameCount"),
                   static_cast<double>(result.strictResidualFrameCount));
    summary.insert(QStringLiteral("rejectedFrameCount"),
                   static_cast<double>(result.rejectedFrameCount));
    summary.insert(QStringLiteral("engineeringCableResidualToleranceM"),
                   kEngineeringCableResidualToleranceM);
    summary.insert(QStringLiteral("strictCableResidualToleranceM"),
                   kStrictCableResidualToleranceM);
    summary.insert(QStringLiteral("maximumSolvedCableResidualUm"),
                   result.maximumSolvedCableResidualUm);
    summary.insert(QStringLiteral("sequenceGapCount"), static_cast<double>(result.sequenceGapCount));
    summary.insert(QStringLiteral("startTraceSequence"), static_cast<double>(result.startTraceSequence));
    summary.insert(QStringLiteral("pvtAnchorCommandRmseDegree"), pvtAnchorRmse);
    summary.insert(QStringLiteral("globalDelayInitialMs"), initialDelayMs);
    summary.insert(QStringLiteral("globalDelayMs"), result.globalDelayMs);
    summary.insert(QStringLiteral("globalDelayFitted"), globalDelayFitted);
    summary.insert(QStringLiteral("maximumTranslationErrorMm"), result.maximumTranslationErrorMm);
    summary.insert(QStringLiteral("rmsTranslationErrorMm"), result.rmsTranslationErrorMm);
    summary.insert(QStringLiteral("maximumOrientationErrorDegree"), result.maximumOrientationErrorDegree);
    summary.insert(QStringLiteral("note"), QStringLiteral(
        "正运动学始终使用同一Trace帧的八轴实际位置；全局延迟仅用于查询规划时间。"));
    QSaveFile summaryFile(QDir(analysisDirectory).filePath(QStringLiteral("summary.json")));
    if (!summaryFile.open(QIODevice::WriteOnly | QIODevice::Text)
        || summaryFile.write(QJsonDocument(summary).toJson(QJsonDocument::Indented)) < 0
        || !summaryFile.commit()) {
        return fail(QStringLiteral("无法写入分析摘要。"));
    }
    result.success = true;
    result.outputDirectory = analysisDirectory;
    result.summary = QStringLiteral("虚拟运动学一致性分析完成：数值有效=%1/%2帧，"
                                    "工程残差达标=%3帧，严格残差达标=%4帧，"
                                    "平移RMS=%5 mm，最大=%6 mm，姿态最大=%7°，"
                                    "最大绳长残差=%8 μm，全局延迟=%9 ms，Trace序号断裂=%10。")
                         .arg(result.analyzedFrameCount)
                         .arg(result.eligibleFrameCount)
                         .arg(result.engineeringResidualFrameCount)
                         .arg(result.strictResidualFrameCount)
                         .arg(result.rmsTranslationErrorMm, 0, 'f', 4)
                         .arg(result.maximumTranslationErrorMm, 0, 'f', 4)
                         .arg(result.maximumOrientationErrorDegree, 0, 'f', 4)
                         .arg(result.maximumSolvedCableResidualUm, 0, 'f', 3)
                         .arg(result.globalDelayMs, 0, 'f', 3)
                         .arg(result.sequenceGapCount);
    return result;
}
