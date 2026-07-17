#ifndef RIGIDBODYFRAME_H
#define RIGIDBODYFRAME_H

#include <QObject>
#include <QVector3D>
#include <QVector>

class NokovRigidBodyFrame : public QObject
{
    Q_OBJECT
public:
    static constexpr int REQUIRED_POINT_COUNT = 8;
    // 结构参数常量
    static constexpr float SIDE_LEN = 121.83f;       // 五边形边长 (mm)
    static constexpr float DIAGONAL = 197.13f;       // 对角线长 (mm)
    static constexpr float LAYER_HEIGHT = 159.88f;   // 上下层圆心垂直距离 (mm)
    // 质心相对于几何中心的偏移（在局部坐标系中）
    static const QVector3D COM_OFFSET;

    // 构造函数，传入8个点的全局坐标（顺序任意）
    explicit NokovRigidBodyFrame(const QVector<QVector3D>& points);

    // 获取计算结果
    QVector3D origin() const { return m_origin; }           // 质心全局坐标
    QVector3D xAxis() const { return m_xAxis; }             // 局部x轴（单位向量）
    QVector3D yAxis() const { return m_yAxis; }             // 局部y轴（单位向量）
    QVector3D zAxis() const { return m_zAxis; }             // 局部z轴（单位向量）
    QVector3D eulerAngles() const { return m_eulerAngles; } // XYZ定轴欧拉角（弧度），顺序：roll, pitch, yaw

    // 若计算成功返回true，否则false
    bool isValid() const { return m_valid; }

private:
    QVector3D m_origin;
    QVector3D m_xAxis, m_yAxis, m_zAxis;
    QVector3D m_eulerAngles;
    bool m_valid;

    // 静态成员变量，存储上一帧的有效数据
    static QVector3D s_lastOrigin;
    static QVector3D s_lastXAxis, s_lastYAxis, s_lastZAxis;
    static QVector3D s_lastEulerAngles;
    static bool s_lastValid;

    // 辅助函数：三点拟合空间圆圆心
    static QVector3D circleCenter3D(const QVector3D& a, const QVector3D& b, const QVector3D& c);
    // 辅助函数：从旋转矩阵（由三轴构成）计算XYZ定轴欧拉角
    static QVector3D rotationMatrixToEuler(const QVector3D& x, const QVector3D& y, const QVector3D& z);
};

#endif // RIGIDBODYFRAME_H
