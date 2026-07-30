#ifndef NOKOVMARKERPROVIDER_H
#define NOKOVMARKERPROVIDER_H

#include "CdprControlTypes.h"

#include <QMutex>
#include <QString>
#include <QVector>

#include "NokovSDKTypes.h"

class NokovSDKClient;

struct NokovMarkerSample
{
    int id = -1;
    CdprVector3 positionM;
};

struct NokovMarkerFrame
{
    qint64 frameNumber = -1;
    qint64 deviceTimestampRaw = 0;
    qint64 hostReceptionTimeUs = 0;
    QVector<NokovMarkerSample> markers;
    bool valid = false;
};

struct NokovMarkerStatus
{
    bool connected = false;
    QString serverAddress;
    QString errorText;
    NokovMarkerFrame latestFrame;
    qint64 frameAgeMs = -1;
};

// 只采集SDK的LabeledMarkers；明确忽略SDK刚体姿态。
// SDK回调线程只复制整帧，位姿重建由独立估计器完成。
class NokovMarkerProvider
{
public:
    NokovMarkerProvider();
    ~NokovMarkerProvider();

    bool connectToServer(const QString &serverAddress,
                         QString *errorText = nullptr);
    void disconnectFromServer();
    NokovMarkerStatus status() const;
    void setPositionScaleToMeter(double scale);

private:
    static void dataCallback(sFrameOfMocapData *data, void *userData);
    static void messageCallback(int messageId, char *message);
    void acceptFrame(const sFrameOfMocapData &data);
    static qint64 monotonicNowUs();

    mutable QMutex mutex_;
    NokovSDKClient *client_ = nullptr;
    NokovMarkerFrame latestFrame_;
    QString serverAddress_;
    QString errorText_;
    double positionScaleToMeter_ = 0.001;
    bool connected_ = false;
};

struct CdprMarkerPoseEstimateResult
{
    CdprPlatformState6 platform;
    bool implemented = false;
    bool valid = false;
    QString errorText;
};

class CdprMarkerPoseEstimator
{
public:
    virtual ~CdprMarkerPoseEstimator() = default;
    virtual CdprMarkerPoseEstimateResult estimate(
        const NokovMarkerFrame &frame) const = 0;
};

// 当前阶段的明确占位实现：保留接口，但绝不返回伪造位姿。
class PendingMarkerGeometryPoseEstimator final
    : public CdprMarkerPoseEstimator
{
public:
    CdprMarkerPoseEstimateResult estimate(
        const NokovMarkerFrame &frame) const override;
};

#endif // NOKOVMARKERPROVIDER_H
