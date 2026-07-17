#include "NokovMinimalClient.h"
#include <iostream>
#include <cmath>
#include <cstring>

// 构造函数
NokovMinimalClient::NokovMinimalClient() 
    : _client(nullptr), _initialized(false), _lastTimestamp(0)
{}

// 析构函数
NokovMinimalClient::~NokovMinimalClient()
{
    Uninitialize();
}

// 初始化SDK客户端
int NokovMinimalClient::Initialize(const char* ipAddress)
{
    if (_initialized) {
        return ErrorCode_OK;
    }

    // 创建客户端实例
    _client = new NokovSDKClient();
    if (!_client) {
        return ErrorCode_Internal;
    }

    // 设置消息回调
    _client->SetMessageCallback(MessageHandlerStatic);
    _client->SetVerbosityLevel(Verbosity_Info);

    // 初始化网络连接
    int retCode = _client->Initialize(const_cast<char*>(ipAddress));
    if (retCode != ErrorCode_OK) {
        std::cerr << "Failed to initialize client. Error code: " << retCode << std::endl;
        delete _client;
        _client = nullptr;
        return retCode;
    }

    // 设置数据和通知回调
    _client->SetDataCallback(DataHandlerStatic, this);
    _client->SetNotifyMsgCallback(NotifyHandlerStatic, this);

    // 检查服务器连接
    sServerDescription serverDesc;
    memset(&serverDesc, 0, sizeof(serverDesc));
    _client->GetServerDescription(&serverDesc);
    if (!serverDesc.HostPresent) {
        std::cerr << "Failed to connect to server: Host not present" << std::endl;
        Uninitialize();
        return ErrorCode_External;
    }

    std::cout << "Successfully connected to server" << std::endl;
    _initialized = true;
    return ErrorCode_OK;
}

// 清理SDK客户端
void NokovMinimalClient::Uninitialize()
{
    if (_client) {
        _client->Uninitialize();
        delete _client;
        _client = nullptr;
    }
    _initialized = false;
    _markers.clear();
    _unnamedMarkers.clear();
    _rigidBodies.clear();
    _lastTimestamp = 0;
}

// 设置数据回调
void NokovMinimalClient::SetDataCallback(DataCallback callback)
{ _dataCallback = callback; }

// 设置消息回调
void NokovMinimalClient::SetMessageCallback(MessageCallback callback)
{ _messageCallback = callback; }

// 设置通知回调
void NokovMinimalClient::SetNotifyCallback(NotifyCallback callback)
{ _notifyCallback = callback; }

// 获取标记点数据
const QVector<MarkerPoint> NokovMinimalClient::GetMarkers() const{
    QMutexLocker locker(&_dataMutex);
    return _markers;
}

// 获取刚体数据
const QVector<RigidBodyData> NokovMinimalClient::GetRigidBodies() const{
    QMutexLocker locker(&_dataMutex);
    return _rigidBodies;
}

// 获取所有未命名标记点数据
const QVector<UnnamedMarkerPoint> NokovMinimalClient::GetUnnamedMarkers() const{
    QMutexLocker locker(&_dataMutex);
    return _unnamedMarkers;
}

// 根据ID获取单个标记点数据
const MarkerPoint* NokovMinimalClient::GetMarkerById(int id) const
{
    for (const auto& marker : _markers) {
        if (marker.id == id) {
            return &marker;
        }
    }
    return nullptr;
}

// 根据ID获取单个刚体数据
const RigidBodyData* NokovMinimalClient::GetRigidBodyById(int id) const
{
    for (const auto& rigidBody : _rigidBodies) {
        if (rigidBody.id == id) {
            return &rigidBody;
        }
    }
    return nullptr;
}

// 根据索引获取单个未命名标记点数据
const UnnamedMarkerPoint* NokovMinimalClient::GetUnnamedMarkerByIndex(int index) const
{
    if (index >= 0 && index < _unnamedMarkers.size()) {
        return &_unnamedMarkers[index];
    }
    return nullptr;
}

// 静态数据回调函数
void NokovMinimalClient::DataHandlerStatic(sFrameOfMocapData* data, void* pUserData)
{
    if (!data || !pUserData) {
        return;
    }

    NokovMinimalClient* client = static_cast<NokovMinimalClient*>(pUserData);
    client->ProcessFrameData(data);
}

// 静态消息回调函数
void NokovMinimalClient::MessageHandlerStatic(int msgType, char* msg)
{
    // 这里可以添加全局消息处理逻辑
    std::cout << "Message: " << msg << " (Type: " << msgType << ")" << std::endl;
}

// 静态通知回调函数
void NokovMinimalClient::NotifyHandlerStatic(sNotifyMsg* pNotify, void* pUserData)
{
    if (!pNotify || !pUserData) {
        return;
    }

    NokovMinimalClient* client = static_cast<NokovMinimalClient*>(pUserData);
    if (client->_notifyCallback) {
        client->_notifyCallback(pNotify);
    }
}

// 处理帧数据
void NokovMinimalClient::ProcessFrameData(const sFrameOfMocapData* data)
{
    if (!data) {
        return;
    }

    // QMutexLocker 会在当前作用域结束（函数返回）时自动解锁
    // QMutex可以这么理解：QMutex是一个钥匙。在该函数中调用了该对象，意味着钥匙被拿走了。这时候GetMarkers被调用了，到QMutexLocker locker(&_dataMutex)这一步，
    // 会发现钥匙被人拿走了。这时它只能进行等待（blocking），直到钥匙被放回来（即ProcessFrameData运行完自动释放QMutexLocker），才能继续
    QMutexLocker locker(&_dataMutex);

    // 更新标记点数据
    // 根据实验结果，在程序长时间运行后，此处的clear会导致程序报错，原因是：多线程数据竞争，即两个不同的线程同时在操作同一个变量
    // 冲突点：此处的clear和GetMarkers()。该函数ProcessFrameData是回调函数，由动捕硬件的数据传入而触发，是一个线程。而GetMarkers由QT的动捕线程调用，是另一个线程
    // 当进行clear时，Qt 检测到数据正在被别人引用，进而报错
    // 解决方案：
    // 1. 将原来的const QVector<MarkerPoint>& NokovMinimalClient::GetMarkers() const，去掉其中的&。即改为返回对象副本，其它线程拿到的是独立的数据
    // 2. 添加互斥锁成员变量，如该函数和GetMarkers函数的locker所示。
    // 下面其它变量的clear及其对应函数，同理
    _markers.clear();

    for (int i = 0; i < data->nLabeledMarkers; ++i) {
        MarkerPoint marker;
        marker.id = data->LabeledMarkers[i].ID;
        marker.x = data->LabeledMarkers[i].x;
        marker.y = data->LabeledMarkers[i].y;
        marker.z = data->LabeledMarkers[i].z;
        _markers.push_back(marker);
    }

    // 更新未命名标记点数据
    _unnamedMarkers.clear();
    for (int i = 0; i < data->nOtherMarkers; ++i) {
        UnnamedMarkerPoint marker;
        marker.index = i;
        marker.x = data->OtherMarkers[i][0];
        marker.y = data->OtherMarkers[i][1];
        marker.z = data->OtherMarkers[i][2];
        _unnamedMarkers.push_back(marker);
    }

    // 更新刚体数据
    _rigidBodies.clear();
    for (int i = 0; i < data->nRigidBodies; ++i) {
        const sRigidBodyData& rb = data->RigidBodies[i];
        RigidBodyData rigidBody;

        // 基本位置和姿态
        rigidBody.id = rb.ID;
        rigidBody.x = rb.x;
        rigidBody.y = rb.y;
        rigidBody.z = rb.z;
        rigidBody.qx = rb.qx;
        rigidBody.qy = rb.qy;
        rigidBody.qz = rb.qz;
        rigidBody.qw = rb.qw;

        // 转换四元数到欧拉角
        QuaternionToEuler(rb.qx, rb.qy, rb.qz, rb.qw, 
                          rigidBody.roll, rigidBody.pitch, rigidBody.yaw);

        // 初始速度和加速度为0
        rigidBody.xVel = rigidBody.yVel = rigidBody.zVel = 0.0f;
        rigidBody.rollVel = rigidBody.pitchVel = rigidBody.yawVel = 0.0f;
        rigidBody.xAccel = rigidBody.yAccel = rigidBody.zAccel = 0.0f;
        rigidBody.rollAccel = rigidBody.pitchAccel = rigidBody.yawAccel = 0.0f;

        // 从扩展数据中提取速度和加速度信息
        for (int j = 0; j < data->extendData.nExtendDataNum; ++j) {
            if (data->extendData.extendData[j].type == ExtendDataRigidBody) {
                for (int k = 0; k < data->extendData.extendData[j].number; ++k) {
                    const sRigidBodyExtendData& rbed = data->extendData.extendData[j].extendData.rigidBodyExtend[k];
                    if (rbed.ID == rb.ID) {
                        // 线速度
                        rigidBody.xVel = rbed.xVel;
                        rigidBody.yVel = rbed.yVel;
                        rigidBody.zVel = rbed.zVel;
                        
                        // 角速度
                        rigidBody.rollVel = rbed.rollVel;
                        rigidBody.pitchVel = rbed.pitchVel;
                        rigidBody.yawVel = rbed.yawVel;
                        
                        // 线加速度
                        rigidBody.xAccel = rbed.xAvel;
                        rigidBody.yAccel = rbed.yAvel;
                        rigidBody.zAccel = rbed.zAvel;
                        
                        // 角加速度
                        rigidBody.rollAccel = rbed.rollAvel;
                        rigidBody.pitchAccel = rbed.pitchAvel;
                        rigidBody.yawAccel = rbed.yawAvel;
                        
                        break;
                    }
                }
                break;
            }
        }

        _rigidBodies.push_back(rigidBody);
    }

    // 保存历史数据（用于计算速度和加速度）
    _prevLastMarkers = _lastMarkers;
    _lastMarkers = _markers;
    _prevLastUnnamedMarkers = _lastUnnamedMarkers;
    _lastUnnamedMarkers = _unnamedMarkers;
    _prevLastRigidBodies = _lastRigidBodies;
    _lastRigidBodies = _rigidBodies;

    // 更新时间戳
    long long currentTimestamp = data->iTimeStamp;
    float dt = 0.0f;
    if (_lastTimestamp > 0) {
        dt = (currentTimestamp - _lastTimestamp) / 1000.0f; // 转换为秒
    }
    _lastTimestamp = currentTimestamp;

    // 计算标记点和未定义点的速度和加速度
    if (dt > 0.0f && dt < 1.0f) { // 确保时间间隔合理
        // 计算标记点的速度和加速度
        for (size_t i = 0; i < _markers.size(); ++i) {
            MarkerPoint& current = _markers[i];
            // 查找上一帧对应的标记点
            for (const auto& last : _lastMarkers) {
                if (last.id == current.id) {
                    // 计算速度
                    current.xVel = (current.x - last.x) / dt;
                    current.yVel = (current.y - last.y) / dt;
                    current.zVel = (current.z - last.z) / dt;
                    // 计算加速度（使用前前一帧数据）
                    if (!_prevLastMarkers.empty()) {
                        for (const auto& prev : _prevLastMarkers) {
                            if (prev.id == current.id) {
                                float lastXVel = (last.x - prev.x) / dt;
                                float lastYVel = (last.y - prev.y) / dt;
                                float lastZVel = (last.z - prev.z) / dt;
                                current.xAccel = (current.xVel - lastXVel) / dt;
                                current.yAccel = (current.yVel - lastYVel) / dt;
                                current.zAccel = (current.zVel - lastZVel) / dt;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }

        // 计算未定义点的速度和加速度
        for (size_t i = 0; i < _unnamedMarkers.size(); ++i) {
            UnnamedMarkerPoint& current = _unnamedMarkers[i];
            // 查找上一帧对应的未定义点
            for (const auto& last : _lastUnnamedMarkers) {
                if (last.index == current.index) {
                    // 计算速度
                    current.xVel = (current.x - last.x) / dt;
                    current.yVel = (current.y - last.y) / dt;
                    current.zVel = (current.z - last.z) / dt;
                    // 计算加速度（使用前前一帧数据）
                    if (!_prevLastUnnamedMarkers.empty()) {
                        for (const auto& prev : _prevLastUnnamedMarkers) {
                            if (prev.index == current.index) {
                                float lastXVel = (last.x - prev.x) / dt;
                                float lastYVel = (last.y - prev.y) / dt;
                                float lastZVel = (last.z - prev.z) / dt;
                                current.xAccel = (current.xVel - lastXVel) / dt;
                                current.yAccel = (current.yVel - lastYVel) / dt;
                                current.zAccel = (current.zVel - lastZVel) / dt;
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    // 调用用户回调
    if (_dataCallback) {
        _dataCallback(data);
    }
}

// 四元数转欧拉角
void NokovMinimalClient::QuaternionToEuler(float qx, float qy, float qz, float qw, 
                                           float& roll, float& pitch, float& yaw)
{
    // 转换四元数到欧拉角（ZYX顺序）
    const float PI = 3.14159265358979323846f;

    // 计算roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

    // 计算pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        pitch = copysignf(PI / 2.0f, sinp) * 180.0f / PI;
    } else {
        pitch = asinf(sinp) * 180.0f / PI;
    }

    // 计算yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}
