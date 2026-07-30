#include "NokovMarkerProvider.h"

#include "NokovSDKClient.h"

#include <QMutexLocker>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

NokovMarkerProvider::NokovMarkerProvider() = default;

NokovMarkerProvider::~NokovMarkerProvider()
{
    disconnectFromServer();
}
bool NokovMarkerProvider::connectToServer(
    const QString &serverAddress, QString *errorText)
{
    disconnectFromServer();
    const QString trimmedAddress = serverAddress.trimmed();
    if (trimmedAddress.isEmpty()) {
        if (errorText) {
            *errorText = QStringLiteral("Nokov服务器地址不能为空。");
        }
        return false;
    }

    auto *candidate = new NokovSDKClient;
    candidate->SetVerbosityLevel(Verbosity_Info);
    candidate->SetMessageCallback(&NokovMarkerProvider::messageCallback);
    QByteArray addressBytes = trimmedAddress.toLatin1();
    const int initializeResult =
        candidate->Initialize(addressBytes.data());
    if (initializeResult != ErrorCode_OK) {
        delete candidate;
        const QString message = QStringLiteral(
            "Nokov SDK连接失败：地址=%1，错误码=%2。")
            .arg(trimmedAddress)
            .arg(initializeResult);
        {
            QMutexLocker locker(&mutex_);
            errorText_ = message;
        }
        if (errorText) {
            *errorText = message;
        }
        return false;
    }

    candidate->SetDataCallback(
        &NokovMarkerProvider::dataCallback, this);
    sServerDescription description;
    std::memset(&description, 0, sizeof(description));
    const int descriptionResult =
        candidate->GetServerDescription(&description);
    if (descriptionResult != ErrorCode_OK || !description.HostPresent) {
        candidate->Uninitialize();
        delete candidate;
        const QString message = QStringLiteral(
            "Nokov服务器未就绪：地址=%1，描述返回码=%2。")
            .arg(trimmedAddress)
            .arg(descriptionResult);
        {
            QMutexLocker locker(&mutex_);
            errorText_ = message;
        }
        if (errorText) {
            *errorText = message;
        }
        return false;
    }

    {
        QMutexLocker locker(&mutex_);
        client_ = candidate;
        connected_ = true;
        serverAddress_ = trimmedAddress;
        errorText_.clear();
        latestFrame_ = {};
    }
    if (errorText) {
        errorText->clear();
    }
    return true;
}

void NokovMarkerProvider::disconnectFromServer()
{
    NokovSDKClient *client = nullptr;
    {
        QMutexLocker locker(&mutex_);
        client = client_;
        client_ = nullptr;
        connected_ = false;
        latestFrame_.valid = false;
    }
    if (client) {
        client->Uninitialize();
        delete client;
    }
}

NokovMarkerStatus NokovMarkerProvider::status() const
{
    NokovMarkerStatus result;
    QMutexLocker locker(&mutex_);
    result.connected = connected_;
    result.serverAddress = serverAddress_;
    result.errorText = errorText_;
    result.latestFrame = latestFrame_;
    if (latestFrame_.hostReceptionTimeUs > 0) {
        result.frameAgeMs = std::max<qint64>(
            0, (monotonicNowUs()
                - latestFrame_.hostReceptionTimeUs) / 1000);
    }
    return result;
}

void NokovMarkerProvider::setPositionScaleToMeter(double scale)
{
    if (!std::isfinite(scale) || scale <= 0.0) {
        return;
    }
    QMutexLocker locker(&mutex_);
    positionScaleToMeter_ = scale;
}

void NokovMarkerProvider::dataCallback(
    sFrameOfMocapData *data, void *userData)
{
    if (data && userData) {
        static_cast<NokovMarkerProvider *>(userData)->acceptFrame(*data);
    }
}

void NokovMarkerProvider::messageCallback(
    int messageId, char *message)
{
    Q_UNUSED(messageId)
    Q_UNUSED(message)
}

void NokovMarkerProvider::acceptFrame(
    const sFrameOfMocapData &data)
{
    NokovMarkerFrame frame;
    frame.frameNumber = data.iFrame;
    frame.deviceTimestampRaw = data.iTimeStamp;
    frame.hostReceptionTimeUs = monotonicNowUs();
    const int markerCount =
        std::clamp(data.nLabeledMarkers, 0, MAX_LABELED_MARKERS);
    frame.markers.reserve(markerCount);
    double scale = 0.001;
    {
        QMutexLocker locker(&mutex_);
        scale = positionScaleToMeter_;
    }
    for (int index = 0; index < markerCount; ++index) {
        const sMarker &source = data.LabeledMarkers[index];
        NokovMarkerSample marker;
        marker.id = source.ID;
        marker.positionM = {
            static_cast<double>(source.x) * scale,
            static_cast<double>(source.y) * scale,
            static_cast<double>(source.z) * scale
        };
        frame.markers.append(marker);
    }
    frame.valid = markerCount > 0;
    QMutexLocker locker(&mutex_);
    if (!connected_) {
        return;
    }
    latestFrame_ = frame;
}

qint64 NokovMarkerProvider::monotonicNowUs()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

CdprMarkerPoseEstimateResult
PendingMarkerGeometryPoseEstimator::estimate(
    const NokovMarkerFrame &frame) const
{
    Q_UNUSED(frame)
    CdprMarkerPoseEstimateResult result;
    result.errorText = QStringLiteral(
        "标记点几何位姿重建算法尚未实现；未使用SDK刚体姿态。");
    return result;
}
