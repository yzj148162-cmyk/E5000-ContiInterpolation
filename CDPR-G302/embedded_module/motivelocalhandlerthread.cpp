#include "motivelocalhandlerthread.h"
#include "qelapsedtimer.h"

#include <QDateTime>
#include <QtMath>

#include <algorithm>

/*
 * 文件总览：
 * - MotiveLocalHandlerThread 的实现文件，负责 Nokov 连接、轮询、数据有效性判断、位姿计算和采集状态机。
 * - 定时循环先拉取当前标记点，再交给 NokovPoseCalculator；采集模式下会累计多帧后发出完成或失败信号。
 */

MotiveLocalHandlerThread::MotiveLocalHandlerThread()
{
}

MotiveLocalHandlerThread::MotiveLocalHandlerThread(double _ctrlCycleMs, QString nokovIP)
    : ctrlCycleMs(_ctrlCycleMs)
{

    if (timer) {
        return;
    }

    timer = new QTimer(this);
    timer->setTimerType(Qt::PreciseTimer);
    timer->setInterval(static_cast<int>(ctrlCycleMs));
    connect(timer, &QTimer::timeout, this, &MotiveLocalHandlerThread::threadLoop);

    m_client = new NokovMinimalClient();

    QByteArray ipBytes = nokovIP.toLatin1();
    int ret = m_client->Initialize(ipBytes.data());

    qDebug() << "Nokov initialize ret =" << ret;

    if (ret != ErrorCode_OK) {
        emit displayInfoSignal(
            std::string("Failed to connect to Nokov server: ") + std::to_string(ret),
            "error"
        );
        return;
    }

    m_isConnected = true;
    isInit = true;
}

MotiveLocalHandlerThread::~MotiveLocalHandlerThread()
{
    if (timer) {
        timer->stop();
        delete timer;
        timer = nullptr;
    }

    if (m_client) {
        delete m_client;
        m_client = nullptr;
    }

    m_isConnected = false;
}

void MotiveLocalHandlerThread::startTimer()
{
    if (timer) {
        timer->start();
    }
}

void MotiveLocalHandlerThread::stopTimer()
{
    if (timer) {
        timer->stop();
        delete timer;
        timer = nullptr;
    }
    isFirstLoop = true;
}

void MotiveLocalHandlerThread::threadLoop()
{
    static int count = 0;
    static double startTimeS = 0.0, curTimeS = 0.0;
    static QElapsedTimer loopTimer;

    if (isFirstLoop) {
        qDebug() << "Motive thread created.";
        loopTimer.start();
        startTimeS = static_cast<double>(loopTimer.elapsed()) / 1000.0;
        Q_UNUSED(startTimeS);
        isFirstLoop = false;
    }

    curTimeS = static_cast<double>(loopTimer.elapsed()) / 1000.0;
    Q_UNUSED(curTimeS);

    dataProcessor();

    count++;
}

void MotiveLocalHandlerThread::beginPoseCapture(int sampleCount)
{
    resetCaptureState(true);
    if (m_client) {
        m_client->SetFrameDataEnabled(true);
    }
    m_captureSampleTarget = qMax(1, sampleCount);
    m_captureActive = true;
    m_captureStartTimestampMs = QDateTime::currentMSecsSinceEpoch();
    m_poseCalculator.resetRoleAssignment();

    if (extraInfo) {
        qDebug() << "Nokov pose capture started. target samples =" << m_captureSampleTarget;
    }

}

void MotiveLocalHandlerThread::setContinuousMonitoringEnabled(bool enabled)
{
    if(m_continuousMonitoringEnabled == enabled){
        return;
    }
    m_continuousMonitoringEnabled = enabled;
    m_lastContinuousMonitorUpdateMs = -1;
    m_lastProcessedFrameSequence = -1;
    m_lastProcessedConnectionGeneration = 0;
    if(!enabled && m_forceInteractionPoseStore){
        m_forceInteractionPoseStore->invalidate();
    }
    if(m_client){
        m_client->SetFrameDataEnabled(enabled || m_captureActive);
    }
}

void MotiveLocalHandlerThread::dataProcessor()
{
    if (!m_captureActive && !m_continuousMonitoringEnabled) {
        return;
    }

    if (!m_client || !m_isConnected) {
        if(m_captureActive){
            failPoseCapture(QStringLiteral("NOKOV client is not connected"));
        }
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (m_captureActive && m_captureStartTimestampMs >= 0 &&
            nowMs - m_captureStartTimestampMs > CAPTURE_TIMEOUT_MS) {
        failPoseCapture(QStringLiteral("采样超时"));
        return;
    }

    const NokovMinimalClient::CaptureFrame frame = m_client->GetCaptureFrame();
    if(frame.sequence < 0 || frame.receivedMonotonicUs <= 0){
        return;
    }
    // 2026-10-06: 定时器可以快于Nokov的10 ms抓拍周期；只处理真正的新帧，
    // 不允许重复读取缓存帧并伪造更年轻的控制反馈时间戳。
    if(frame.sequence == m_lastProcessedFrameSequence &&
            frame.connectionGeneration == m_lastProcessedConnectionGeneration){
        return;
    }
    m_lastProcessedFrameSequence = frame.sequence;
    m_lastProcessedConnectionGeneration = frame.connectionGeneration;

    const QVector<MarkerPoint> rigidMarkers = currentRigidMarkers(frame);
    {
        QMutexLocker locker(&m_poseMutex);
        m_lastMarkerCount = rigidMarkers.size();
    }
    if (rigidMarkers.size() != NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
        if (extraInfo) {
            qDebug() << "Nokov rigid body skipped. markers size =" << rigidMarkers.size()
                     << "required =" << NokovPoseCalculator::REQUIRED_MARKER_COUNT;
        }
        return;
    }

    NokovPoseCalculator::Result poseResult;

    if (m_poseCalculator.update(rigidMarkers, poseResult)) {
        if(m_captureActive){
            accumulatePoseSample(poseResult);
        }
        else{
            publishContinuousPose(poseResult, frame);
        }
        if (detailInfo) {
            qDebug() << "Nokov pose capture sample"
                     << m_captureSampleCount
                     << "/"
                     << m_captureSampleTarget;
        }
        if (m_captureActive && m_captureSampleCount >= m_captureSampleTarget) {
            finishPoseCapture();
        }
    } else {
        if (extraInfo) {
            qDebug() << "Nokov 3-marker pose invalid. markers size =" << rigidMarkers.size();
        }
        return;
    }
}

std::vector<std::vector<double>> MotiveLocalHandlerThread::getRigidPose() const
{
    // 使用 tempRigidPose，避免在更新瞬间读到空 rigidPose
    QMutexLocker locker(&m_poseMutex);
    return tempRigidPose;
}

std::vector<std::vector<double>> MotiveLocalHandlerThread::calCableStartPos()
{
    return {};
}

bool MotiveLocalHandlerThread::hasCurrentRigidBody() const
{
    QMutexLocker locker(&m_poseMutex);
    return m_lastRigidBodyValid;
}

bool MotiveLocalHandlerThread::hasRecentRigidBody(int maxAgeMs) const
{
    QMutexLocker locker(&m_poseMutex);
    if (m_lastValidRigidBodyTimestampMs < 0 || maxAgeMs < 0 || tempRigidPose.empty()) {
        return false;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    return (nowMs - m_lastValidRigidBodyTimestampMs) <= maxAgeMs;
}

qint64 MotiveLocalHandlerThread::lastValidRigidBodyTimestampMs() const
{
    QMutexLocker locker(&m_poseMutex);
    return m_lastValidRigidBodyTimestampMs;
}

bool MotiveLocalHandlerThread::hasCapturedRigidBody() const
{
    QMutexLocker locker(&m_poseMutex);
    return !tempRigidPose.empty() && tempRigidPose.front().size() >= 6;
}

int MotiveLocalHandlerThread::lastMarkerCount() const
{
    QMutexLocker locker(&m_poseMutex);
    return m_lastMarkerCount;
}

QVector<MarkerPoint> MotiveLocalHandlerThread::currentRigidMarkers(
        const NokovMinimalClient::CaptureFrame& frame)
{
    QVector<MarkerPoint> rigidMarkers;
    for (const RigidBodyData& rigidBody : frame.rigidBodies) {
        if (rigidBody.markers.size() == NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
            rigidMarkers = rigidBody.markers;
            break;
        }
    }
    if (rigidMarkers.isEmpty() &&
            frame.markers.size() == NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
        rigidMarkers = frame.markers;
    }

    return rigidMarkers;
}

void MotiveLocalHandlerThread::resetCaptureState(bool clearPose)
{
    if (m_client) {
        m_client->SetFrameDataEnabled(m_continuousMonitoringEnabled);
    }
    m_captureActive = false;
    m_captureSampleCount = 0;
    m_captureStartTimestampMs = -1;
    m_captureMarkerSums.clear();
    {
        QMutexLocker locker(&m_poseMutex);
        m_lastRigidBodyValid = false;
        m_lastMarkerCount = 0;
        m_lastValidRigidBodyTimestampMs = -1;
        rigidPose.clear();
        if (clearPose) {
            tempRigidPose.clear();
        }
    }
    if(clearPose && m_forceInteractionPoseStore){
        m_forceInteractionPoseStore->invalidate();
    }
}

void MotiveLocalHandlerThread::publishContinuousPose(
        const NokovPoseCalculator::Result& poseResult,
        const NokovMinimalClient::CaptureFrame& frame)
{
    const QVector3D origin = poseResult.positionMm;
    const QVector3D eulerAnglesDeg = poseResult.eulerDeg;
    const std::vector<std::vector<double>> pose{{
        static_cast<double>(origin.x()),
        static_cast<double>(origin.y()),
        static_cast<double>(origin.z()),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.x())),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.y())),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.z()))
    }};
    // 2026-10-06: 控制快照跟随每个Nokov新帧；原有UI/低频安全通道仍保持
    // 200 ms节流，避免在线张力接入扩大界面刷新和跨线程信号负担。
    if(m_forceInteractionPoseStore){
        ForceInteractionMocapPose observation;
        observation.valid = true;
        observation.sourceFrameSequence = frame.sequence;
        observation.connectionGeneration = frame.connectionGeneration;
        observation.receivedMonotonicUs = frame.receivedMonotonicUs;
        observation.receivedWallClockMs = frame.receivedAtMs;
        std::copy(pose.front().begin(), pose.front().begin() + 6,
                  observation.poseMmRad.begin());
        m_forceInteractionPoseStore->publish(observation);
    }
    if(m_lastContinuousMonitorUpdateMs >= 0 &&
            frame.receivedAtMs - m_lastContinuousMonitorUpdateMs <
                CONTINUOUS_MONITOR_PERIOD_MS){
        return;
    }
    {
        QMutexLocker locker(&m_poseMutex);
        rigidPose = pose;
        tempRigidPose = pose;
        m_lastRigidBodyValid = true;
        m_lastValidRigidBodyTimestampMs = frame.receivedAtMs;
    }
    m_lastContinuousMonitorUpdateMs = frame.receivedAtMs;
    emit dataUpdateSignal(pose);
}

void MotiveLocalHandlerThread::failPoseCapture(const QString& reason)
{
    const int markerCount = m_lastMarkerCount;
    resetCaptureState(true);
    emit poseCaptureFailed(reason.toStdString(), markerCount);
}

void MotiveLocalHandlerThread::accumulatePoseSample(const NokovPoseCalculator::Result& poseResult)
{
    if (poseResult.orderedMarkers.size() != NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
        return;
    }

    if (m_captureMarkerSums.isEmpty()) {
        m_captureMarkerSums = poseResult.orderedMarkers;
    } else if (m_captureMarkerSums.size() == poseResult.orderedMarkers.size()) {
        for (int i = 0; i < m_captureMarkerSums.size(); ++i) {
            m_captureMarkerSums[i].x += poseResult.orderedMarkers[i].x;
            m_captureMarkerSums[i].y += poseResult.orderedMarkers[i].y;
            m_captureMarkerSums[i].z += poseResult.orderedMarkers[i].z;
        }
    }

    ++m_captureSampleCount;
}

void MotiveLocalHandlerThread::finishPoseCapture()
{
    if (m_captureSampleCount <= 0 ||
            m_captureMarkerSums.size() != NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
        failPoseCapture(QStringLiteral("有效采样数量不足"));
        return;
    }

    QVector<MarkerPoint> averageMarkers = m_captureMarkerSums;
    for (MarkerPoint& marker : averageMarkers) {
        marker.x /= static_cast<float>(m_captureSampleCount);
        marker.y /= static_cast<float>(m_captureSampleCount);
        marker.z /= static_cast<float>(m_captureSampleCount);
    }

    NokovPoseCalculator::Result poseResult;
    if (!NokovPoseCalculator::calculateFromOrderedMarkers(averageMarkers, poseResult)) {
        failPoseCapture(QStringLiteral("平均marker位姿计算失败"));
        return;
    }

    const QVector3D origin = poseResult.positionMm;
    const QVector3D eulerAnglesDeg = poseResult.eulerDeg;
    const std::vector<std::vector<double>> completedPose{{
        static_cast<double>(origin.x()),
        static_cast<double>(origin.y()),
        static_cast<double>(origin.z()),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.x())),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.y())),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.z()))
    }};
    {
        QMutexLocker locker(&m_poseMutex);
        rigidPose = completedPose;
        tempRigidPose = completedPose;
        m_lastRigidBodyValid = true;
        m_lastValidRigidBodyTimestampMs = QDateTime::currentMSecsSinceEpoch();
    }
    const int sampleCount = m_captureSampleCount;
    m_captureActive = false;
    m_captureSampleCount = 0;
    m_captureStartTimestampMs = -1;
    m_captureMarkerSums.clear();
    if (m_client) {
        m_client->SetFrameDataEnabled(m_continuousMonitoringEnabled);
    }

    if (detailInfo) {
        qDebug() << "Nokov pose capture finished. samples =" << sampleCount
                 << "pose(mm/rad):"
                 << completedPose[0][0]
                 << completedPose[0][1]
                 << completedPose[0][2]
                 << completedPose[0][3]
                 << completedPose[0][4]
                 << completedPose[0][5]
                 << "euler(deg):"
                 << eulerAnglesDeg.x()
                 << eulerAnglesDeg.y()
                 << eulerAnglesDeg.z();
    }

    emit dataUpdateSignal(completedPose);
    emit poseCaptureCompleted(completedPose, sampleCount);
}
