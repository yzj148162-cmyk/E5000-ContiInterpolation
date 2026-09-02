/*
 * 文件总览：
 * - NokovPoseCalculator 根据 3 个标记点计算刚体位置和姿态。
 * - 标记点顺序由动捕软件中的刚体 marker 顺序预先确认，本类直接按输入顺序建立配准点对应。
 */

#ifndef NOKOVPOSECALCULATOR_H
#define NOKOVPOSECALCULATOR_H

#include "NokovMinimalClient.h"

#include <QVector>
#include <QVector3D>

class NokovPoseCalculator
{
public:
    static constexpr int REQUIRED_MARKER_COUNT = 3;

    struct Result
    {
        QVector3D positionMm;
        QVector3D eulerDeg;
        QVector<MarkerPoint> orderedMarkers;
    };

    // 输入当前帧标记点并输出稳定排序后的刚体位姿。
    bool update(const QVector<MarkerPoint>& markers, Result& result);
    // 当前流程直接使用动捕给出的 marker 顺序，始终视为已完成顺序确认。
    bool hasRoleAssignment() const;
    // 保留接口兼容旧调用；当前不维护历史角色分配。
    void resetRoleAssignment();
    // 对已按动捕刚体顺序排列的 3 个点直接计算位置和姿态。
    static bool calculateFromOrderedMarkers(const QVector<MarkerPoint>& orderedMarkers,
                                            Result& result);

private:
    // 直接使用动捕软件中配置的刚体 marker 顺序。
    static QVector<MarkerPoint> orderByCaptureSequence(const QVector<MarkerPoint>& markers);
};

#endif // NOKOVPOSECALCULATOR_H
