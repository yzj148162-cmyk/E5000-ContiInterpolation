#include "motivelocalhandlerthread.h"
#include "nokovrigidbodyframe.h"
#include "qelapsedtimer.h"

#include <QDateTime>

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
        timer->deleteLater();
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
        timer->deleteLater();
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

void MotiveLocalHandlerThread::dataProcessor()
{
    rigidPose.clear();
    m_lastRigidBodyValid = false;
    m_lastMarkerCount = 0;

    if (!m_client || !m_isConnected) {
        emit dataUpdateSignal(tempRigidPose);
        return;
    }

    // 从 Nokov 客户端读取最新 marker 数据
    QVector<MarkerPoint> markers = m_client->GetMarkers();
    QVector<RigidBodyData> rigidBodies = m_client->GetRigidBodies();
    QVector<UnnamedMarkerPoint> unnamedMarkers = m_client->GetUnnamedMarkers();

    Q_UNUSED(rigidBodies);
    Q_UNUSED(unnamedMarkers);

    QVector<QVector3D> points;
    points.reserve(markers.size());

    for (const MarkerPoint& marker : markers) {
        points.append(QVector3D(marker.x, marker.y, marker.z));
    }

    m_lastMarkerCount = points.size();
    if (points.size() != NokovRigidBodyFrame::REQUIRED_POINT_COUNT) {
        if (extraInfo) {
            qDebug() << "Nokov rigid body skipped. markers size =" << points.size()
                     << "required =" << NokovRigidBodyFrame::REQUIRED_POINT_COUNT;
        }
        emit dataUpdateSignal(tempRigidPose);
        return;
    }

    NokovRigidBodyFrame frame(points);

    if (frame.isValid()) {
        QVector3D origin = frame.origin();
        QVector3D eulerAngles = frame.eulerAngles();

        rigidPose.push_back({
            static_cast<double>(origin.x()),
            static_cast<double>(origin.y()),
            static_cast<double>(origin.z()),
            static_cast<double>(eulerAngles.x()),
            static_cast<double>(eulerAngles.y()),
            static_cast<double>(eulerAngles.z())
        });
        m_lastRigidBodyValid = true;
        m_lastValidRigidBodyTimestampMs = QDateTime::currentMSecsSinceEpoch();

        if (detailInfo) {
            qDebug() << "Nokov rigid pose:"
                     << rigidPose[0][0]
                     << rigidPose[0][1]
                     << rigidPose[0][2]
                     << rigidPose[0][3]
                     << rigidPose[0][4]
                     << rigidPose[0][5];
        }
    } else {
        if (extraInfo) {
            qDebug() << "NokovRigidBodyFrame invalid. markers size =" << markers.size();
        }
        emit dataUpdateSignal(tempRigidPose);
        return;
    }

    tempRigidPose = rigidPose;
    emit dataUpdateSignal(rigidPose);
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

int MotiveLocalHandlerThread::lastMarkerCount() const
{
    return m_lastMarkerCount;
}
