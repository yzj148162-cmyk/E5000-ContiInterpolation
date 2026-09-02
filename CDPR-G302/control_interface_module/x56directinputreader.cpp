#include "x56directinputreader.h"

#include <QUuid>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#ifdef Q_OS_WIN
#ifndef DIRECTINPUT_VERSION
#define DIRECTINPUT_VERSION 0x0800
#endif
#include <qt_windows.h>
#include <dinput.h>
#endif

namespace {
constexpr quint16 kX56VendorId = 0x0738;
constexpr quint16 kX56StickProductId = 0x2221;
constexpr double kX56AxisDeadzone = 0.08;
constexpr qint64 kX56EnumerationIntervalMs = 500;
constexpr qint64 kX56RetryIntervalMs = 500;

qint64 monotonicNowMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<qint64>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

double applyDeadzone(double value)
{
    if(!std::isfinite(value)){
        return 0.0;
    }
    value = std::clamp(value, -1.0, 1.0);
    const double magnitude = std::fabs(value);
    if(magnitude <= kX56AxisDeadzone){
        return 0.0;
    }
    const double remapped = (magnitude - kX56AxisDeadzone) /
            (1.0 - kX56AxisDeadzone);
    return std::copysign(std::clamp(remapped, 0.0, 1.0), value);
}

std::array<double, 3> normalizedDirection(std::array<double, 3> direction)
{
    const double norm = std::sqrt(direction[0] * direction[0] +
                                  direction[1] * direction[1] +
                                  direction[2] * direction[2]);
    if(norm > 1.0){
        for(double& value : direction){
            value /= norm;
        }
    }
    return direction;
}

#ifdef Q_OS_WIN
QString guidText(const GUID& guid)
{
    const QUuid uuid(guid.Data1,
                     guid.Data2,
                     guid.Data3,
                     guid.Data4[0],
                     guid.Data4[1],
                     guid.Data4[2],
                     guid.Data4[3],
                     guid.Data4[4],
                     guid.Data4[5],
                     guid.Data4[6],
                     guid.Data4[7]);
    return uuid.toString(QUuid::WithoutBraces).toUpper();
}

struct AxisRange
{
    LONG minimum = 0;
    LONG maximum = 65535;
    bool valid = false;
};

double normalizedAxis(LONG rawValue, const AxisRange& range)
{
    if(!range.valid || range.maximum <= range.minimum){
        return 0.0;
    }
    const double minimum = static_cast<double>(range.minimum);
    const double maximum = static_cast<double>(range.maximum);
    const double midpoint = minimum + (maximum - minimum) * 0.5;
    const double raw = std::clamp(static_cast<double>(rawValue), minimum, maximum);
    const double denominator = raw >= midpoint ?
                maximum - midpoint : midpoint - minimum;
    if(denominator <= std::numeric_limits<double>::epsilon()){
        return 0.0;
    }
    return std::clamp((raw - midpoint) / denominator, -1.0, 1.0);
}
#endif
}

struct X56DirectInputReader::Impl
{
    quintptr windowHandle = 0;
    bool fixedBindingEnabled = false;
    QString boundInstanceId;
    quint64 sessionToken = 0;
    std::vector<X56DirectInputDeviceInfo> publicDevices;

    bool sessionNeutralConfirmed = false;
    bool b06Armed = false;
    bool b06ReleaseRequired = true;
    bool b01RotationArmed = false;
    bool b01ReleaseRequired = true;
    bool previousB06 = false;
    bool previousB01 = false;

#ifdef Q_OS_WIN
    struct NativeDeviceInfo {
        GUID instanceGuid{};
        QString instanceId;
        QString displayName;
    };

    LPDIRECTINPUT8W directInput = nullptr;
    LPDIRECTINPUTDEVICE8W device = nullptr;
    std::vector<NativeDeviceInfo> nativeDevices;
    QString openedInstanceId;
    AxisRange mainXRange;
    AxisRange mainYRange;
    qint64 lastEnumerationMs = 0;
    qint64 nextOpenAttemptMs = 0;
    bool forceEnumeration = true;
    bool inputPollingActive = false;

    static BOOL CALLBACK enumerateDevicesCallback(
            const DIDEVICEINSTANCEW* instance,
            VOID* context)
    {
        if(!instance || !context){
            return DIENUM_CONTINUE;
        }
        const quint16 vendorId = LOWORD(instance->guidProduct.Data1);
        const quint16 productId = HIWORD(instance->guidProduct.Data1);
        if(vendorId != kX56VendorId || productId != kX56StickProductId){
            return DIENUM_CONTINUE;
        }
        Impl* self = static_cast<Impl*>(context);
        NativeDeviceInfo info;
        info.instanceGuid = instance->guidInstance;
        info.instanceId = guidText(instance->guidInstance);
        info.displayName = QString::fromWCharArray(instance->tszProductName).trimmed();
        if(info.displayName.isEmpty()){
            info.displayName = QStringLiteral("X56 Stick");
        }
        self->nativeDevices.push_back(info);
        return DIENUM_CONTINUE;
    }

    void resetSafetyState()
    {
        sessionNeutralConfirmed = false;
        b06Armed = false;
        b06ReleaseRequired = true;
        b01RotationArmed = false;
        b01ReleaseRequired = true;
        previousB06 = false;
        previousB01 = false;
    }

    void releaseDevice()
    {
        if(device){
            device->Unacquire();
            device->Release();
            device = nullptr;
        }
        openedInstanceId.clear();
        mainXRange = AxisRange();
        mainYRange = AxisRange();
        resetSafetyState();
    }

    void shutdown()
    {
        releaseDevice();
        if(directInput){
            directInput->Release();
            directInput = nullptr;
        }
        nativeDevices.clear();
        publicDevices.clear();
        lastEnumerationMs = 0;
        nextOpenAttemptMs = 0;
        forceEnumeration = true;
        inputPollingActive = false;
    }

    void allowImmediateOpenAttempt()
    {
        nextOpenAttemptMs = 0;
    }

    void scheduleOpenRetry(qint64 nowMs)
    {
        nextOpenAttemptMs = nowMs + kX56RetryIntervalMs;
        // 打开/读取失败后的枚举和重新打开共用同一退避窗口，避免任一路径
        // 在20 ms监督周期内形成设备访问风暴。
        lastEnumerationMs = nowMs;
        forceEnumeration = false;
    }

    qint64 retryRemainingMs(qint64 nowMs) const
    {
        return std::max<qint64>(0, nextOpenAttemptMs - nowMs);
    }

    bool ensureDirectInput()
    {
        if(directInput){
            return true;
        }
        const HRESULT result = DirectInput8Create(
                    GetModuleHandleW(nullptr),
                    DIRECTINPUT_VERSION,
                    IID_IDirectInput8W,
                    reinterpret_cast<void**>(&directInput),
                    nullptr);
        return SUCCEEDED(result) && directInput;
    }

    void enumerateDevices(bool force = false)
    {
        const qint64 nowMs = monotonicNowMs();
        if(!force && !forceEnumeration && lastEnumerationMs > 0 &&
                nowMs - lastEnumerationMs < kX56EnumerationIntervalMs){
            return;
        }
        forceEnumeration = false;
        lastEnumerationMs = nowMs;
        nativeDevices.clear();
        publicDevices.clear();
        if(!ensureDirectInput()){
            releaseDevice();
            return;
        }
        directInput->EnumDevices(DI8DEVCLASS_GAMECTRL,
                                 &Impl::enumerateDevicesCallback,
                                 this,
                                 DIEDFL_ATTACHEDONLY);
        std::sort(nativeDevices.begin(), nativeDevices.end(),
                  [](const NativeDeviceInfo& lhs, const NativeDeviceInfo& rhs){
            return lhs.instanceId < rhs.instanceId;
        });
        for(const NativeDeviceInfo& info : nativeDevices){
            publicDevices.push_back({info.instanceId, info.displayName});
        }

        bool openedStillSelectable = false;
        for(const NativeDeviceInfo& info : nativeDevices){
            if(info.instanceId == openedInstanceId &&
                    (!fixedBindingEnabled || info.instanceId == boundInstanceId)){
                openedStillSelectable = true;
                break;
            }
        }
        if(!openedStillSelectable){
            releaseDevice();
        }
    }

    const NativeDeviceInfo* selectedDeviceInfo() const
    {
        if(fixedBindingEnabled){
            const auto found = std::find_if(
                        nativeDevices.begin(), nativeDevices.end(),
                        [this](const NativeDeviceInfo& info){
                return info.instanceId.compare(boundInstanceId,
                                               Qt::CaseInsensitive) == 0;
            });
            return found == nativeDevices.end() ? nullptr : &*found;
        }
        return nativeDevices.size() == 1 ? &nativeDevices.front() : nullptr;
    }

    static AxisRange readAxisRange(LPDIRECTINPUTDEVICE8W source,
                                   DWORD offset)
    {
        AxisRange range;
        if(!source){
            return range;
        }
        DIPROPRANGE property{};
        property.diph.dwSize = sizeof(DIPROPRANGE);
        property.diph.dwHeaderSize = sizeof(DIPROPHEADER);
        property.diph.dwObj = offset;
        property.diph.dwHow = DIPH_BYOFFSET;
        if(SUCCEEDED(source->GetProperty(DIPROP_RANGE, &property.diph)) &&
                property.lMax > property.lMin){
            range.minimum = property.lMin;
            range.maximum = property.lMax;
            range.valid = true;
        }
        return range;
    }

    bool openSelectedDevice()
    {
        const NativeDeviceInfo* selected = selectedDeviceInfo();
        if(!selected || !directInput || windowHandle == 0){
            releaseDevice();
            return false;
        }
        if(device && openedInstanceId == selected->instanceId){
            return true;
        }
        releaseDevice();
        if(FAILED(directInput->CreateDevice(selected->instanceGuid,
                                            &device,
                                            nullptr)) || !device){
            return false;
        }
        const HWND window = reinterpret_cast<HWND>(windowHandle);
        if(FAILED(device->SetDataFormat(&c_dfDIJoystick2)) ||
                FAILED(device->SetCooperativeLevel(
                           window,
                           DISCL_FOREGROUND | DISCL_NONEXCLUSIVE))){
            releaseDevice();
            return false;
        }
        mainXRange = readAxisRange(device, DIJOFS_X);
        mainYRange = readAxisRange(device, DIJOFS_Y);
        if(!mainXRange.valid || !mainYRange.valid){
            releaseDevice();
            return false;
        }
        openedInstanceId = selected->instanceId;
        device->Acquire();
        return true;
    }

    bool readState(DIJOYSTATE2& state)
    {
        if(!device){
            return false;
        }
        HRESULT result = device->Poll();
        if(FAILED(result)){
            for(int attempt = 0; attempt < 3; ++attempt){
                result = device->Acquire();
                if(result != DIERR_INPUTLOST){
                    break;
                }
            }
            if(FAILED(result)){
                return false;
            }
            result = device->Poll();
        }
        if(FAILED(result) ||
                FAILED(device->GetDeviceState(sizeof(DIJOYSTATE2), &state))){
            return false;
        }
        return true;
    }
#else
    void resetSafetyState()
    {
        sessionNeutralConfirmed = false;
        b06Armed = false;
        b06ReleaseRequired = true;
        b01RotationArmed = false;
        b01ReleaseRequired = true;
        previousB06 = false;
        previousB01 = false;
    }

    void shutdown()
    {
        publicDevices.clear();
        resetSafetyState();
    }
#endif
};

X56DirectInputReader::X56DirectInputReader()
    : impl(std::make_unique<Impl>())
{
}

X56DirectInputReader::~X56DirectInputReader()
{
    shutdown();
}

void X56DirectInputReader::configure(quintptr windowHandle,
                                     bool fixedBindingEnabled,
                                     const QString& boundInstanceId)
{
    const QString normalizedId = boundInstanceId.trimmed().toUpper();
    const bool changed = impl->windowHandle != windowHandle ||
            impl->fixedBindingEnabled != fixedBindingEnabled ||
            impl->boundInstanceId != normalizedId;
    if(!changed){
        return;
    }
    impl->windowHandle = windowHandle;
    impl->fixedBindingEnabled = fixedBindingEnabled;
    impl->boundInstanceId = normalizedId;
#ifdef Q_OS_WIN
    impl->forceEnumeration = true;
    impl->allowImmediateOpenAttempt();
    impl->inputPollingActive = false;
    impl->releaseDevice();
#else
    impl->resetSafetyState();
#endif
}

void X56DirectInputReader::beginSession(quint64 sessionToken)
{
    impl->sessionToken = sessionToken;
#ifdef Q_OS_WIN
    impl->forceEnumeration = true;
    impl->allowImmediateOpenAttempt();
    impl->inputPollingActive = false;
    impl->releaseDevice();
#else
    impl->resetSafetyState();
#endif
}

void X56DirectInputReader::endSession()
{
    impl->sessionToken = 0;
#ifdef Q_OS_WIN
    impl->inputPollingActive = false;
    impl->allowImmediateOpenAttempt();
    impl->releaseDevice();
#else
    impl->resetSafetyState();
#endif
}

void X56DirectInputReader::shutdown()
{
    if(impl){
        impl->shutdown();
    }
}

X56DirectInputSnapshot X56DirectInputReader::poll(
        bool remoteRunning,
        bool applicationActive)
{
    X56DirectInputSnapshot snapshot;
#ifndef Q_OS_WIN
    Q_UNUSED(remoteRunning);
    Q_UNUSED(applicationActive);
    snapshot.state = X56DirectInputSnapshot::State::Unsupported;
    snapshot.statusText = QStringLiteral("X56：当前平台不支持 DirectInput");
    snapshot.warning = true;
    return snapshot;
#else
    impl->enumerateDevices();
    if(!impl->directInput){
        snapshot.state = X56DirectInputSnapshot::State::Unsupported;
        snapshot.statusText = QStringLiteral("X56：DirectInput 初始化失败，手柄输入已禁用");
        snapshot.warning = true;
        return snapshot;
    }
    if(impl->fixedBindingEnabled && impl->boundInstanceId.isEmpty()){
        snapshot.state = X56DirectInputSnapshot::State::BoundDeviceMissing;
        snapshot.statusText = QStringLiteral("X56：已启用固定绑定，但尚未选择设备实例");
        snapshot.warning = true;
        return snapshot;
    }
    if(impl->nativeDevices.empty()){
        snapshot.state = impl->fixedBindingEnabled ?
                    X56DirectInputSnapshot::State::BoundDeviceMissing :
                    X56DirectInputSnapshot::State::NoDevice;
        snapshot.statusText = impl->fixedBindingEnabled ?
                    QStringLiteral("X56：绑定设备未连接，手柄输入已禁用") :
                    QStringLiteral("X56：未检测到 X56 Stick，键盘/界面仍可使用");
        snapshot.warning = impl->fixedBindingEnabled;
        impl->releaseDevice();
        return snapshot;
    }
    if(!impl->fixedBindingEnabled && impl->nativeDevices.size() > 1){
        snapshot.state = X56DirectInputSnapshot::State::MultipleDevices;
        snapshot.statusText = QStringLiteral(
                    "X56：检测到 %1 只同型号手柄，未固定绑定，手柄输入已禁用")
                .arg(impl->nativeDevices.size());
        snapshot.warning = true;
        impl->releaseDevice();
        return snapshot;
    }
    if(impl->fixedBindingEnabled && !impl->selectedDeviceInfo()){
        snapshot.state = X56DirectInputSnapshot::State::BoundDeviceMissing;
        snapshot.statusText = QStringLiteral("X56：绑定设备未连接，手柄输入已禁用");
        snapshot.warning = true;
        impl->releaseDevice();
        return snapshot;
    }

    // 未建立有效会话、末端遥控尚未进入Running或窗口失焦时，只保留
    // 500 ms设备枚举/状态更新，不执行Acquire、Poll或GetDeviceState。
    if(impl->sessionToken == 0 || !remoteRunning){
        if(impl->device){
            impl->releaseDevice();
        }
        else{
            impl->resetSafetyState();
        }
        impl->inputPollingActive = false;
        snapshot.state = X56DirectInputSnapshot::State::WaitingForRun;
        snapshot.statusText = QStringLiteral(
                    "X56：已连接，等待末端遥控进入运行状态（B15～B17已忽略）");
        return snapshot;
    }
    if(!applicationActive){
        if(impl->device){
            impl->releaseDevice();
        }
        else{
            impl->resetSafetyState();
        }
        impl->inputPollingActive = false;
        snapshot.state = X56DirectInputSnapshot::State::WindowInactive;
        snapshot.statusText = QStringLiteral("X56：主窗口未激活，手柄已解除使能");
        return snapshot;
    }
    if(!impl->inputPollingActive){
        // 每次从非运行态/失焦态恢复，都必须重新完成全输入回中。
        impl->resetSafetyState();
        impl->allowImmediateOpenAttempt();
        impl->inputPollingActive = true;
    }

    const qint64 nowMs = monotonicNowMs();
    const qint64 retryRemainingMs = impl->retryRemainingMs(nowMs);
    if(retryRemainingMs > 0){
        snapshot.state = X56DirectInputSnapshot::State::DeviceOpenFailed;
        snapshot.statusText = QStringLiteral(
                    "X56：设备打开/读取失败，输入保持为零，按500 ms退避重试");
        snapshot.warning = true;
        snapshot.retryAfterMs = retryRemainingMs;
        return snapshot;
    }

    snapshot.readAttempted = true;
    if(!impl->openSelectedDevice()){
        snapshot.state = X56DirectInputSnapshot::State::DeviceOpenFailed;
        snapshot.statusText = QStringLiteral(
                    "X56：设备打开/读取失败，输入保持为零，按500 ms退避重试");
        snapshot.warning = true;
        snapshot.readFailed = true;
        impl->scheduleOpenRetry(nowMs);
        snapshot.retryAfterMs = kX56RetryIntervalMs;
        return snapshot;
    }

    DIJOYSTATE2 state{};
    if(!impl->readState(state)){
        snapshot.state = X56DirectInputSnapshot::State::DeviceOpenFailed;
        snapshot.statusText = QStringLiteral(
                    "X56：设备打开/读取失败，输入保持为零，按500 ms退避重试");
        snapshot.warning = true;
        snapshot.readFailed = true;
        impl->releaseDevice();
        impl->scheduleOpenRetry(nowMs);
        snapshot.retryAfterMs = kX56RetryIntervalMs;
        return snapshot;
    }
    impl->allowImmediateOpenAttempt();
    snapshot.sampleFresh = true;

    // 实机采集确认：主摇杆右/前分别为DirectInput X+/Y-。
    // 平动采用全局+X右、+Y前；转动输入表示ZYX欧拉角Rx/Ry变化率，
    // Rz变化率固定为零，再由控制器换算为真实的全局角速度。
    const double mainRight = applyDeadzone(
                normalizedAxis(state.lX, impl->mainXRange));
    const double mainForward = applyDeadzone(
                -normalizedAxis(state.lY, impl->mainYRange));
    const auto pressed = [&state](int oneBasedButton){
        const int index = oneBasedButton - 1;
        return index >= 0 && index < 128 &&
                (state.rgbButtons[index] & 0x80) != 0;
    };
    const bool b01 = pressed(1);
    const bool b06 = pressed(6);
    const bool h2Up = pressed(11);
    const bool h2Down = pressed(13);
    const bool mainStickNeutral = std::fabs(mainRight) <= 1.0e-12 &&
            std::fabs(mainForward) <= 1.0e-12;
    const bool h2Neutral = !h2Up && !h2Down;
    const bool allNeutral = mainStickNeutral && h2Neutral &&
            !b01 && !b06;

    // 任一模式键按下都独占输入；即使回中检查失败，也不回落到UI旧输入。
    snapshot.takeoverRequested = b06 || b01;

    if(!impl->sessionNeutralConfirmed && allNeutral){
        impl->sessionNeutralConfirmed = true;
        impl->b06ReleaseRequired = false;
        impl->b01ReleaseRequired = false;
    }

    const bool neutralForB06 = mainStickNeutral && h2Neutral;
    if(!b06){
        impl->b06Armed = false;
        if(impl->b06ReleaseRequired && neutralForB06){
            impl->b06ReleaseRequired = false;
            impl->sessionNeutralConfirmed = true;
        }
    }
    const bool b06Rising = b06 && !impl->previousB06;
    bool b06RejectedThisFrame = false;
    if(b06Rising){
        if(impl->sessionNeutralConfirmed &&
                !impl->b06ReleaseRequired && neutralForB06){
            impl->b06Armed = true;
        }
        else{
            impl->b06Armed = false;
            impl->b06ReleaseRequired = true;
            b06RejectedThisFrame = true;
        }
    }

    if(!b01){
        impl->b01RotationArmed = false;
        if(impl->b01ReleaseRequired && mainStickNeutral){
            impl->b01ReleaseRequired = false;
        }
    }
    const bool b01Rising = b01 && !impl->previousB01;
    bool b01RejectedThisFrame = false;
    if(b01Rising){
        if(impl->sessionNeutralConfirmed && !b06 &&
                !impl->b01ReleaseRequired && mainStickNeutral && h2Neutral){
            impl->b01RotationArmed = true;
        }
        else{
            impl->b01RotationArmed = false;
            impl->b01ReleaseRequired = true;
            b01RejectedThisFrame = true;
        }
    }

    // B06优先。B06释放而B01仍保持按下时，不会无回中地自动切入转动；
    // 必须松开并重新按下B01，控制器侧还会等平动速度完全降为零。
    const bool translationActive = b06 && impl->b06Armed;
    const bool rotationActive = !b06 && b01 && impl->b01RotationArmed;
    snapshot.takeoverActive = translationActive || rotationActive;
    if(translationActive){
        snapshot.motionMode = EndpointRemoteMotionMode::Translation;
        const double zDirection = (h2Up ? 1.0 : 0.0) -
                (h2Down ? 1.0 : 0.0);
        snapshot.direction = normalizedDirection(
                    {mainRight, mainForward, zDirection});
    }
    else if(rotationActive){
        snapshot.motionMode =
                EndpointRemoteMotionMode::YawLockedEulerRotation;
        snapshot.direction = normalizedDirection(
                    {-mainForward, mainRight, 0.0});
    }

    if(b06 && !impl->b06Armed){
        snapshot.state = X56DirectInputSnapshot::State::EnableRejected;
        snapshot.statusText = b06RejectedThisFrame ?
                    QStringLiteral(
                        "X56：小指键(B06)平动使能被拒绝；请松开B06，令主摇杆和凹陷型苦力帽回中后重试") :
                    QStringLiteral(
                        "X56：等待松开小指键(B06)并完成全部输入回中，手柄输出保持为零");
        snapshot.warning = true;
    }
    else if(!b06 && b01 && !impl->b01RotationArmed){
        snapshot.state = X56DirectInputSnapshot::State::RotationModeRejected;
        snapshot.statusText = b01RejectedThisFrame ?
                    QStringLiteral(
                        "X56：食指扳机(B01)转动使能被拒绝；请松开扳机、令主摇杆和凹陷型苦力帽回中后重试") :
                    QStringLiteral(
                        "X56：食指扳机(B01)仍按住但转动模式尚未重新武装；输出保持为零");
        snapshot.warning = true;
    }
    else if(snapshot.takeoverActive){
        snapshot.state = X56DirectInputSnapshot::State::Armed;
        snapshot.statusText = translationActive ?
                    QStringLiteral("X56：B06平动模式；主摇杆前/右=+Y/+X，凹陷型苦力帽上/下=+Z/-Z（B06优先于扳机）") :
                    QStringLiteral("X56：B01转动模式；主摇杆前/后=Rx减小/增大，右/左=Ry增大/减小；无Rz角速度");
    }
    else if(!impl->sessionNeutralConfirmed || impl->b06ReleaseRequired){
        snapshot.state = X56DirectInputSnapshot::State::WaitingForNeutral;
        snapshot.statusText = QStringLiteral(
                    "X56：等待主摇杆、凹陷型苦力帽、食指扳机和小指键全部回中");
    }
    else{
        snapshot.state = X56DirectInputSnapshot::State::Ready;
        snapshot.statusText = impl->fixedBindingEnabled ?
                    QStringLiteral("X56：绑定设备就绪；按住B06平动，按住食指扳机B01转动") :
                    QStringLiteral("X56：已就绪；按住B06平动，按住食指扳机B01转动");
        if(impl->nativeDevices.size() > 1){
            snapshot.statusText += QStringLiteral(
                        "（另检测到同型号设备，当前使用固定绑定实例）");
            snapshot.warning = true;
        }
    }

    impl->previousB06 = b06;
    impl->previousB01 = b01;
    return snapshot;
#endif
}

std::vector<X56DirectInputDeviceInfo> X56DirectInputReader::devices() const
{
    return impl ? impl->publicDevices :
                  std::vector<X56DirectInputDeviceInfo>();
}
