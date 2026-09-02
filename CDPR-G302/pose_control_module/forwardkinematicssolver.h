/*
 * 文件总览：
 * - ForwardKinematicsSolver 从 0525 的 fp_kine/kine 逻辑抽出独立正运动学求解模块。
 * - 只根据绳长反解实际末端位姿，供绘图/诊断使用，不写入 MainWindow 的末端位姿缓存。
 */

#ifndef FORWARDKINEMATICSSOLVER_H
#define FORWARDKINEMATICSSOLVER_H

#include <vector>

class ForwardKinematicsSolver
{
public:
    struct Request {
        std::vector<std::vector<double>> anchorPos;
        std::vector<std::vector<double>> contactPointLocal;
        std::vector<double> cableLength;
        std::vector<int> excludedCableIndices;
        double pulleyRadius = 0.0;
        std::vector<double> initialPose;
        std::vector<double> poseLowerBounds;
        std::vector<double> poseUpperBounds;
        bool keepRotation = true;
    };

    struct Result {
        bool success = false;
        std::vector<double> pose;
        int equationCount = 0;
    };

    Result solve(const Request& request);
    void setInitialPose(const std::vector<double>& pose);
    std::vector<double> initialPose() const;

private:
    std::vector<double> lastPose;
};

#endif // FORWARDKINEMATICSSOLVER_H
