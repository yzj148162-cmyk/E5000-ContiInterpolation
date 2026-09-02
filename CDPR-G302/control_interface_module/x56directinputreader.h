#ifndef X56DIRECTINPUTREADER_H
#define X56DIRECTINPUTREADER_H

#include <QString>
#include <QtGlobal>

#include <array>
#include <memory>
#include <vector>

#include "endpointremotecontrol.h"

struct X56DirectInputDeviceInfo
{
    QString instanceId;
    QString displayName;
};

struct X56DirectInputSnapshot
{
    enum class State {
        Unsupported = 0,
        NoDevice,
        MultipleDevices,
        BoundDeviceMissing,
        DeviceOpenFailed,
        AcquisitionStale,
        WindowInactive,
        WaitingForRun,
        WaitingForNeutral,
        Ready,
        Armed,
        EnableRejected,
        RotationModeRejected
    };

    State state = State::Unsupported;
    EndpointRemoteMotionMode motionMode = EndpointRemoteMotionMode::None;
    std::array<double, 3> direction{};
    // 实机确认小指键为B06。物理按下时即请求独占输入；即使回中检查失败，
    // 也不能回落到残留键盘输入。
    bool takeoverRequested = false;
    bool takeoverActive = false;
    bool sampleFresh = false;
    // 仅在本轮确实执行了设备打开或状态读取时置位，供监督器统计真实失败。
    bool readAttempted = false;
    bool readFailed = false;
    qint64 retryAfterMs = 0;
    QString statusText;
    bool warning = false;
};

// X56生成“模式+三维归一化方向”：平动为全局XYZ，转动为全局定轴Rx/Ry/Rz。
// DirectInput对象和安全状态机仅由X56InputWorker采集线程访问。
class X56DirectInputReader
{
public:
    X56DirectInputReader();
    ~X56DirectInputReader();

    X56DirectInputReader(const X56DirectInputReader&) = delete;
    X56DirectInputReader& operator=(const X56DirectInputReader&) = delete;

    void configure(quintptr windowHandle,
                   bool fixedBindingEnabled,
                   const QString& boundInstanceId);
    void beginSession(quint64 sessionToken);
    void endSession();
    void shutdown();

    X56DirectInputSnapshot poll(bool remoteRunning,
                                bool applicationActive);
    std::vector<X56DirectInputDeviceInfo> devices() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

#endif // X56DIRECTINPUTREADER_H
