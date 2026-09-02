/*
 * 文件总览：
 * - NokovMinimalClient 是 Nokov SDK 的轻量封装，负责连接、注册回调、保存最新标记点/刚体数据。
 * - 对外提供线程安全的最新数据读取接口，供 MotiveLocalHandlerThread 和位姿计算模块使用。
 * - 本文件只声明封装结构，底层 SDK 类型来自第三方 Nokov 头文件。
 */

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
    QVector<MarkerPoint> markers;
    float meanError;
    short params;
    float xVel, yVel, zVel;
    float rollVel, pitchVel, yawVel;
    float xAccel, yAccel, zAccel;
    float rollAccel, pitchAccel, yawAccel;
} RigidBodyData;

class NokovMinimalClient {
public:
    // 创建 SDK 轻量封装对象，尚不连接服务器。
    NokovMinimalClient();
    // 析构时确保注销回调并释放 SDK 客户端。
    ~NokovMinimalClient();

    // 初始化和清理
    // 初始化 Nokov SDK 并连接指定 IP 的动捕服务器。
    int Initialize(const char* ipAddress);
    // 断开连接并清理 SDK 状态。
    void Uninitialize();

    // 设置回调函数
    // 设置原始帧数据回调；通常只在调试或外部扩展时使用。
    void SetDataCallback(DataCallback callback);
    // 设置 SDK 消息回调。
    void SetMessageCallback(MessageCallback callback);
    // 设置 SDK 通知回调。
    void SetNotifyCallback(NotifyCallback callback);
    // 控制是否在 SDK 回调中解析并缓存帧数据。
    void SetFrameDataEnabled(bool enabled);

    // 获取最新数据
    // 获取最近一帧命名标记点快照。
    const QVector<MarkerPoint> GetMarkers() const;
    // 获取最近一帧刚体快照。
    const QVector<RigidBodyData> GetRigidBodies() const;
    // 获取最近一帧未命名标记点快照。
    const QVector<UnnamedMarkerPoint> GetUnnamedMarkers() const;
    
    // 根据ID获取单个标记点或刚体数据
    // 按标记点 ID 查找最近缓存中的命名标记点。
    const MarkerPoint* GetMarkerById(int id) const;
    // 按刚体 ID 查找最近缓存中的刚体数据。
    const RigidBodyData* GetRigidBodyById(int id) const;
    // 根据索引获取单个未命名标记点数据
    // 按数组索引查找最近缓存中的未命名标记点。
    const UnnamedMarkerPoint* GetUnnamedMarkerByIndex(int index) const;

private:
    // SDK客户端
    NokovSDKClient* _client;
    bool _initialized;
    bool _frameDataEnabled;

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
    // SDK 静态数据回调入口，转发到对应 NokovMinimalClient 实例。
    static void DataHandlerStatic(sFrameOfMocapData* data, void* pUserData);
    // SDK 静态消息回调入口。
    static void MessageHandlerStatic(int msgType, char* msg);
    // SDK 静态通知回调入口，转发到用户设置的通知回调。
    static void NotifyHandlerStatic(sNotifyMsg* pNotify, void* pUserData);

    // 数据处理
    // 解析 SDK 帧数据并更新标记点、刚体、速度和加速度缓存。
    void ProcessFrameData(const sFrameOfMocapData* data);
    // 将四元数转换为 roll/pitch/yaw，供刚体快照显示使用。
    void QuaternionToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw);
};
