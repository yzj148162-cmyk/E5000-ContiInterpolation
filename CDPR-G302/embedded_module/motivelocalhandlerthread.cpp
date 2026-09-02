#include "motivelocalhandlerthread.h"
#include "qelapsedtimer.h"

#include <QDateTime>
#include <QtMath>

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

void MotiveLocalHandlerThread::dataProcessor()
{
    if (!m_captureActive) {
        return;
    }

    if (!m_client || !m_isConnected) {
        failPoseCapture(QStringLiteral("NOKOV client is not connected"));
        return;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (m_captureStartTimestampMs >= 0 &&
            nowMs - m_captureStartTimestampMs > CAPTURE_TIMEOUT_MS) {
        failPoseCapture(QStringLiteral("采样超时"));
        return;
    }

    const QVector<MarkerPoint> rigidMarkers = currentRigidMarkers();
    m_lastMarkerCount = rigidMarkers.size();
    if (rigidMarkers.size() != NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
        if (extraInfo) {
            qDebug() << "Nokov rigid body skipped. markers size =" << rigidMarkers.size()
                     << "required =" << NokovPoseCalculator::REQUIRED_MARKER_COUNT;
        }
        return;
    }

    NokovPoseCalculator::Result poseResult;

    if (m_poseCalculator.update(rigidMarkers, poseResult)) {
        accumulatePoseSample(poseResult);
        if (detailInfo) {
            qDebug() << "Nokov pose capture sample"
                     << m_captureSampleCount
                     << "/"
                     << m_captureSampleTarget;
        }
        if (m_captureSampleCount >= m_captureSampleTarget) {
            finishPoseCapture();
        }
    } else {
        if (extraInfo) {
            qDebug() << "Nokov 3-marker pose invalid. markers size =" << rigidMarkers.size();
        }
        return;
    }
}

std::vector<std::vector<double>> MotiveLocalHandlerThread::getRigidPose()
{
    // 使用 tempRigidPose，避免在更新瞬间读到空 rigidPose
    return tempRigidPose;
}

std::vector<std::vector<double>> MotiveLocalHandlerThread::calCableStartPos()
{
    return {};
}

bool MotiveLocalHandlerThread::hasCurrentRigidBody() const
{
    return m_lastRigidBodyValid;
}

bool MotiveLocalHandlerThread::hasRecentRigidBody(int maxAgeMs) const
{
    if (m_lastValidRigidBodyTimestampMs < 0 || maxAgeMs < 0 || tempRigidPose.empty()) {
        return false;
    }

    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    return (nowMs - m_lastValidRigidBodyTimestampMs) <= maxAgeMs;
}

bool MotiveLocalHandlerThread::hasCapturedRigidBody() const
{
    return !tempRigidPose.empty() && tempRigidPose.front().size() >= 6;
}

int MotiveLocalHandlerThread::lastMarkerCount() const
{
    return m_lastMarkerCount;
}

QVector<MarkerPoint> MotiveLocalHandlerThread::currentRigidMarkers()
{
    QVector<MarkerPoint> markers = m_client->GetMarkers();
    QVector<RigidBodyData> rigidBodies = m_client->GetRigidBodies();
    QVector<UnnamedMarkerPoint> unnamedMarkers = m_client->GetUnnamedMarkers();

    Q_UNUSED(unnamedMarkers);

    QVector<MarkerPoint> rigidMarkers;
    for (const RigidBodyData& rigidBody : rigidBodies) {
        if (rigidBody.markers.size() == NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
            rigidMarkers = rigidBody.markers;
            break;
        }
    }
    if (rigidMarkers.isEmpty() && markers.size() == NokovPoseCalculator::REQUIRED_MARKER_COUNT) {
        rigidMarkers = markers;
    }

    return rigidMarkers;
}

void MotiveLocalHandlerThread::resetCaptureState(bool clearPose)
{
    if (m_client) {
        m_client->SetFrameDataEnabled(false);
    }
    m_captureActive = false;
    m_captureSampleCount = 0;
    m_captureStartTimestampMs = -1;
    m_captureMarkerSums.clear();
    m_lastRigidBodyValid = false;
    m_lastMarkerCount = 0;
    m_lastValidRigidBodyTimestampMs = -1;
    rigidPose.clear();
    if (clearPose) {
        tempRigidPose.clear();
    }
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
    rigidPose.clear();
    rigidPose.push_back({
        static_cast<double>(origin.x()),
        static_cast<double>(origin.y()),
        static_cast<double>(origin.z()),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.x())),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.y())),
        qDegreesToRadians(static_cast<double>(eulerAnglesDeg.z()))
    });

    tempRigidPose = rigidPose;
    m_lastRigidBodyValid = true;
    m_lastValidRigidBodyTimestampMs = QDateTime::currentMSecsSinceEpoch();
    const int sampleCount = m_captureSampleCount;
    m_captureActive = false;
    m_captureSampleCount = 0;
    m_captureStartTimestampMs = -1;
    m_captureMarkerSums.clear();
    if (m_client) {
        m_client->SetFrameDataEnabled(false);
    }

    if (detailInfo) {
        qDebug() << "Nokov pose capture finished. samples =" << sampleCount
                 << "pose(mm/rad):"
                 << rigidPose[0][0]
                 << rigidPose[0][1]
                 << rigidPose[0][2]
                 << rigidPose[0][3]
                 << rigidPose[0][4]
                 << rigidPose[0][5]
                 << "euler(deg):"
                 << eulerAnglesDeg.x()
                 << eulerAnglesDeg.y()
                 << eulerAnglesDeg.z();
    }

    emit dataUpdateSignal(rigidPose);
    emit poseCaptureCompleted(rigidPose, sampleCount);
}
