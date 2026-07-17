#pragma once

#include "NokovSDKClient.h"
#include "NokovSDKTypes.h"
#include <string>
#include <functional>
#include <QVector>
#include <QDebug>
#include <QMutex>

// 简化的数据回调类型定义
typedef std::function<void(const sFrameOfMocapData* data)> DataCallback;
typedef std::function<void(int msgType, const char* msg)> MessageCallback;
typedef std::function<void(sNotifyMsg* pNotify)> NotifyCallback;

// 标记点数据结构
typedef struct {
    int id;
    float x, y, z;
    float xVel, yVel, zVel;
    float xAccel, yAccel, zAccel;
} MarkerPoint;

// 未命名标记点数据结构
typedef struct {
    int index;
    float x, y, z;
    float xVel, yVel, zVel;
    float xAccel, yAccel, zAccel;
} UnnamedMarkerPoint;

// 刚体数据结构
typedef struct {
    int id;
    float x, y, z;
    float qx, qy, qz, qw;
    float roll, pitch, yaw;
    float xVel, yVel, zVel;
    float rollVel, pitchVel, yawVel;
    float xAccel, yAccel, zAccel;
    float rollAccel, pitchAccel, yawAccel;
} RigidBodyData;

class NokovMinimalClient {
public:
    NokovMinimalClient();
    ~NokovMinimalClient();

    // 初始化和清理
    int Initialize(const char* ipAddress);
    void Uninitialize();

    // 设置回调函数
    void SetDataCallback(DataCallback callback);
    void SetMessageCallback(MessageCallback callback);
    void SetNotifyCallback(NotifyCallback callback);

    // 获取最新数据
    const QVector<MarkerPoint> GetMarkers() const;
    const QVector<RigidBodyData> GetRigidBodies() const;
    const QVector<UnnamedMarkerPoint> GetUnnamedMarkers() const;
    
    // 根据ID获取单个标记点或刚体数据
    const MarkerPoint* GetMarkerById(int id) const;
    const RigidBodyData* GetRigidBodyById(int id) const;
    // 根据索引获取单个未命名标记点数据
    const UnnamedMarkerPoint* GetUnnamedMarkerByIndex(int index) const;

private:
    // SDK客户端
    NokovSDKClient* _client;
    bool _initialized;

    // 回调函数
    DataCallback _dataCallback;
    MessageCallback _messageCallback;
    NotifyCallback _notifyCallback;

    // 最新数据
    QVector<MarkerPoint> _markers;
    QVector<RigidBodyData> _rigidBodies;
    QVector<UnnamedMarkerPoint> _unnamedMarkers;

    // 历史数据（用于计算速度和加速度）
    QVector<MarkerPoint> _lastMarkers;
    QVector<MarkerPoint> _prevLastMarkers;
    QVector<UnnamedMarkerPoint> _lastUnnamedMarkers;
    QVector<UnnamedMarkerPoint> _prevLastUnnamedMarkers;
    QVector<RigidBodyData> _lastRigidBodies;
    QVector<RigidBodyData> _prevLastRigidBodies;

    // 时间戳（用于其他可能的时间相关计算）
    long long _lastTimestamp;

    mutable QMutex _dataMutex;

    // 静态回调函数
    static void DataHandlerStatic(sFrameOfMocapData* data, void* pUserData);
    static void MessageHandlerStatic(int msgType, char* msg);
    static void NotifyHandlerStatic(sNotifyMsg* pNotify, void* pUserData);

    // 数据处理
    void ProcessFrameData(const sFrameOfMocapData* data);
    void QuaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw);
};
