#ifndef MOTIONREQUEST_H
#define MOTIONREQUEST_H

#include "config/MotionConfig.h"

// UI 线程投递给规划线程的最新运动请求。
// sequenceId 用来区分“同一线程周期下的新任务”和“旧任务的尾波”。
struct MotionRequest
{
    MotionConfig config;
    qint64 sequenceId = 0;
    bool active = false;
    bool restartRequested = false;
};

#endif // MOTIONREQUEST_H
