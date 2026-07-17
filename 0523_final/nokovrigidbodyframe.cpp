#include "nokovrigidbodyframe.h"
#include <QtMath>
#include <algorithm>

const QVector3D NokovRigidBodyFrame::COM_OFFSET = QVector3D(-0.377f, 0.48f, -15.835f);

// 静态成员变量初始化
QVector3D NokovRigidBodyFrame::s_lastOrigin;
QVector3D NokovRigidBodyFrame::s_lastXAxis(1, 0, 0);
QVector3D NokovRigidBodyFrame::s_lastYAxis(0, 1, 0);
QVector3D NokovRigidBodyFrame::s_lastZAxis(0, 0, 1);
QVector3D NokovRigidBodyFrame::s_lastEulerAngles;
bool NokovRigidBodyFrame::s_lastValid = false;

NokovRigidBodyFrame::NokovRigidBodyFrame(const QVector<QVector3D>& points)
    : m_valid(false)
{
    if (points.size() != 8) {
        // 如果点数不是8，使用上一帧数据
        if (s_lastValid) {
            m_origin = s_lastOrigin;
            m_xAxis = s_lastXAxis;
            m_yAxis = s_lastYAxis;
            m_zAxis = s_lastZAxis;
            m_eulerAngles = s_lastEulerAngles;
            m_valid = true;
        }
        return;
    }

    // 1. 按z坐标排序，区分上下层（假设z轴向上，刚体下层在下，上层在上）
    QVector<QVector3D> sorted = points;
    std::sort(sorted.begin(), sorted.end(),
              [](const QVector3D& a, const QVector3D& b) { return a.z() < b.z(); });

    QVector<QVector3D> lower = sorted.mid(0, 5);
    QVector<QVector3D> upper = sorted.mid(5, 3);

    // 2. 下层圆心：五点平均
    QVector3D centerLow(0, 0, 0);
    for (const auto& p : lower) centerLow += p;
    centerLow /= 5.0f;

    // 3. 上层圆心：三点拟合空间圆
    QVector3D centerUp = circleCenter3D(upper[0], upper[1], upper[2]);

    // 4. 修正上层圆心，使两圆心距离精确等于层高
    QVector3D dir = centerUp - centerLow;
    float len = dir.length();
    if (len > 1e-6f) {
        dir /= len;
        centerUp = centerLow + dir * LAYER_HEIGHT;
    } else {
        centerUp = centerLow + QVector3D(0, 0, LAYER_HEIGHT);
    }

    // 5. 几何中心
    QVector3D geoCenter = (centerLow + centerUp) / 2.0f;

    // 6. 确定局部坐标轴方向
    // z轴：从下层圆心指向上层圆心
    m_zAxis = (centerUp - centerLow).normalized();

    // 识别上层固定点（钝角顶点）：找出最长边所对的顶点
    float d01 = (upper[0] - upper[1]).lengthSquared();
    float d12 = (upper[1] - upper[2]).lengthSquared();
    float d20 = (upper[2] - upper[0]).lengthSquared();

    int fixIdx = -1;
    if (d01 > d12 && d01 > d20) fixIdx = 2;
    else if (d12 > d01 && d12 > d20) fixIdx = 0;
    else fixIdx = 1;
    QVector3D fixPoint = upper[fixIdx];

    // y轴：从几何中心指向固定点的水平投影
    QVector3D yRaw = fixPoint - geoCenter;
    QVector3D yProj = yRaw - QVector3D::dotProduct(yRaw, m_zAxis) * m_zAxis;
    if (yProj.lengthSquared() < 1e-6f) {
        yProj = QVector3D(1, 0, 0);
        if (qAbs(QVector3D::dotProduct(yProj, m_zAxis)) > 0.999f)
            yProj = QVector3D(0, 1, 0);
    }
    m_yAxis = yProj.normalized();

    // x轴：右手定则
    m_xAxis = QVector3D::crossProduct(m_yAxis, m_zAxis).normalized();

    // 7. 计算质心全局坐标
    QVector3D offsetGlobal = COM_OFFSET.x() * m_xAxis +
                             COM_OFFSET.y() * m_yAxis +
                             COM_OFFSET.z() * m_zAxis;
    m_origin = geoCenter + offsetGlobal;

    // 8. 计算欧拉角
    m_eulerAngles = rotationMatrixToEuler(m_xAxis, m_yAxis, m_zAxis);

    m_valid = true;
    
    // 更新上一帧数据
    s_lastOrigin = m_origin;
    s_lastXAxis = m_xAxis;
    s_lastYAxis = m_yAxis;
    s_lastZAxis = m_zAxis;
    s_lastEulerAngles = m_eulerAngles;
    s_lastValid = true;
}

QVector3D NokovRigidBodyFrame::circleCenter3D(const QVector3D& a, const QVector3D& b, const QVector3D& c)
{
    QVector3D u = b - a;
    QVector3D v = c - a;
    QVector3D n = QVector3D::crossProduct(u, v);

    float uu = QVector3D::dotProduct(u, u);
    float uv = QVector3D::dotProduct(u, v);
    float vv = QVector3D::dotProduct(v, v);

    float det = uu * vv - uv * uv;
    if (qFuzzyIsNull(det)) {
        return (a + b + c) / 3.0f;
    }

    float alpha = ( (uu/2.0f) * vv - (vv/2.0f) * uv ) / det;
    float beta  = ( uu * (vv/2.0f) - uv * (uu/2.0f) ) / det;

    return a + alpha * u + beta * v;
}

QVector3D NokovRigidBodyFrame::rotationMatrixToEuler(const QVector3D& x, const QVector3D& y, const QVector3D& z)
{
    // 旋转矩阵 R = [x y z] (列向量为局部坐标轴在global中的表示)
    // 计算 XYZ 定轴欧拉角： R = Rz(ψ) * Ry(θ) * Rx(φ)
    float r11 = x.x();  float r12 = y.x();  float r13 = z.x();
    float r21 = x.y();  float r22 = y.y();  float r23 = z.y();
    float r31 = x.z();  float r32 = y.z();  float r33 = z.z();

    float phi, theta, psi;

    // 处理万向锁情况
    if (qAbs(r31) > 1.0f - 1e-6f) {
        // 当 cosβ = 0 时，β = ±90°
        theta = -qAsin(r31);  // 实际上 r31 = -sinβ
        phi = 0.0f;
        psi = qAtan2(r21, r11);
        // 注意：此处公式可能需根据具体符号调整，但通用解法
    } else {
        theta = qAtan2(-r31, qSqrt(r11*r11 + r21*r21));
        phi = qAtan2(r32, r33);
        psi = qAtan2(r21, r11);
    }

    return QVector3D(phi, theta, psi);
}
