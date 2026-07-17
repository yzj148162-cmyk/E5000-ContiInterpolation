#ifndef FORCESIMULATIONMODEL_H
#define FORCESIMULATIONMODEL_H

#include <QTimer>
#include <QElapsedTimer>
#include <QVector3D>
#include <QMutex>
#include <QDebug>
#include <QDir>
#include <Windows.h>

#include <MatrixFun.h>
#include <optimization.h>
#include <nlopt.h>
#include <nlopt.hpp>
#include <cdprdynmodel.h>

# pragma execution_character_set("utf-8")

using namespace alglib;

class ForceSimulationModel : public QObject
{
    Q_OBJECT
public:
    // 分别输入平台坐标系下的绳索接点位置(mm),末端平台重力(N)（-157N）,绳索长度和锚点座位置
    ForceSimulationModel();
    ForceSimulationModel(double _ctrlCycleMs, std::vector<std::vector<double>> endEffectorPos, double _weight,
                              std::vector<double> curCableLen, std::vector<std::vector<double>> _curAnchorPos);
    // endEffectorPos:平台坐标系下的绳索接点位置（结构参数）。顺序与下面函数update中的_curAnchorPos顺序一致
    ForceSimulationModel(double _ctrlCycleMs, std::vector<std::vector<std::vector<double>>> endEffectorPos,bool _useCam,std::string _sta);// 输入量皆为固定参数，变量于update中输入
    ~ForceSimulationModel();
    void startTimer();
    void stopTimer();
//    void stopThread();// 终止

    // 设置优化算法中绳索力范围
    void setCableForceMinMax(double min, std::vector<std::vector<double>> max);
    void setPulleyRadius(double radius);
    void setMotorTorqueLimitNm(double limitNm);
    // 设置运行模式。目前有：清华末端动平台模式，第三方传入期望力与力矩模式
    void setMode(std::string modeName);
    // 启用并设置动力学模型（注意：单位m,rad(定轴欧拉角XYZ，word里是动轴ZYX，两者等价),kg,N）
    void setDynPara(std::vector<double> mass, std::vector<std::vector<std::vector<double>>> inertiaLocal,
                    std::vector<std::vector<std::vector<double>>> anchorsGlobal,std::vector<std::vector<std::vector<double>>> contactPointsLocal,
                    std::vector<std::vector<double>> nr, std::vector<double> massAgent, std::vector<std::vector<std::vector<double>>> inertiaLocalAgent,
                    std::vector<std::vector<double>> Imq, std::vector<std::vector<double>> Fvq,
                    std::vector<std::vector<double>> Fcq,
                    std::vector<std::vector<double>> md, std::vector<std::vector<double>> dd, std::vector<std::vector<double>> kd,
                    std::string _sta);
    void setDynImpCurDesTraj(std::vector<std::vector<double>> pos, std::vector<std::vector<double>> vel,
                             std::vector<std::vector<double>> acc);// 动力学阻抗当前时刻期望轨迹
    bool useDyn = false;
    // 更新输入量。其中_curPlatformPose为各平台的实际位姿，用于代替运动学kine()的理论位姿
    // _curAnchorPos：第一层为各个平台，第二层顺序为各个平台的各个绳索对应的锚点座（绳索顺序应与ui中定义的一致），第三层为锚点座坐标
    // curCableLen用不上了
    void update(std::vector<std::vector<std::vector<double>>> _curAnchorPos, std::vector<std::vector<double>> _curPlatformPose,
                std::vector<std::vector<double>> _expPlatformForce, std::vector<std::vector<double>> _curPlatformForce, double _curTimeStamp);// 力矩输入为NM，所以乘1000
    // 在不用动捕获取实际位姿，而是通过运动学进行计算实际位姿
    void updateNoCam(std::vector<std::vector<std::vector<double>>> _curAnchorPos, std::vector<std::vector<double>> _curCableLen,
                     std::vector<std::vector<double>> _expPlatformForce, std::vector<std::vector<double>> _curPlatformForce, double _curTimeStamp);
    // 读取当前各绳索实际力，用于在没有末端六维力传感器的情况下，计算末端受力情况
    std::vector<std::vector<double>> solvePoseFromCableLengths(std::vector<std::vector<std::vector<double>>> _curAnchorPos,
                                                               std::vector<std::vector<double>> _curCableLen,
                                                               bool keepRotation = true);
    void updateCurCableForce(std::vector<std::vector<double>> f);

    // === para ===
    // 声明为静态，从而使得其能在该类的所有对象之间通用，目的是为了在优化算法方程式function1_fvec中定义的GravityCompensationThread对象能够根据类成员函数trajPlan()更新参数与变量
    // 若不声明为静态类，在其它类成员函数中更新的参数与变量并不会反映到function1_fvec中定义的GravityCompensationThread对象上，毕竟在function1_fvec中是新定义了一个对象
    // 静态类成员的初始化需要在类外进行（且不能放在头文件中，因此放在了.cpp中）
    static int cableNum;// 基于输入的向量endEffector_e确定
    // 平台坐标系下的绳索接点位置（结构参数）
    static std::vector<std::vector<double>> endEffector_e;// = {
    std::vector<std::vector<std::vector<double>>> endEffector_eTemp;
                                                    //{220,220,200,1},
                                                    //{-220, 220, 200, 1},
                                                    //{-220,-220 ,200,1},
                                                    //{220,-220 , 200,1},
                                                    //{220, 0, -200,1},
                                                    //{0 , 220, -200,1},
                                                    //{ -220,0, -200,1},
                                                    //{0 ,-220 ,-200,1} };

    // 末端平台or平台位姿。第一层为各个平台，第二层为其位姿
    std::vector<std::vector<double>> curPlatformPose,curPlatformVel;
    // 绳索长度
    static std::vector<double> curCableLen;// = { 1873.92501450418, 1873.92501450418, 1873.92501450418, 1873.92501450418, 2689.68853401281, 2494.20296086746, 2689.68853401281, 2494.20296086746 };
    static double pulleyRadius;
    std::vector<std::vector<double>> curCableLenTemp;
    // 锚点座位置。顺序为第一个平台的各个绳索对应的锚点座（绳索顺序应与ui中定义的一致），然后第二个平台的各个绳索对应的锚点座
    static std::vector<std::vector<double>> curAnchorPos;// = {{1680.88760800011,	1303.88760800011,	149.900000000000,1},{-1680.88760800011,	1303.88760800011,	149.900000000000,1},
                                           //{-1680.88760800011, -1303.88760800011,	149.900000000000,1},
                                           //{1680.88760800011, -1303.88760800011,	149.900000000000,1},
                                           //{1752.50000000000,	0,	2410.40000000000,1},
                                           //{0,	1375.50000000000,	2410.40000000000,1},
                                           //{-1752.50000000000,	2.14619351550574e-13,	2410.40000000000,1},
                                           //{-1.68450167222718e-13, -1375.50000000000,	2410.40000000000,1}}
    std::vector<std::vector<std::vector<double>>> curAnchorPosTemp;
    std::vector<std::vector<double>> expPlatformForce;// 各平台/末端的期望力及力矩(NM)
    std::vector<std::vector<double>> curPlatformForce;// 各平台/末端的实际力及力矩(NM)
    double curTimeStamp = 0.0;// 记录从启动开始的运行时间
    std::vector<double> weight;

    static std::vector<double> endForce_e;// 末端平台所受力和力矩
    static std::vector<double> cableForce_e;// 绳索拉力
    static std::vector<std::vector<double>> jacoTrans;// 雅可比矩阵
    std::vector<std::vector<double>> homePlatformPose;// 各平台的初始位姿
    std::vector<std::vector<std::vector<double>>> homeCableVecTemp;// 各平台的初始绳长向量
    std::vector<std::vector<double>> curCableVecTemp, expCableVecTemp;// 当前平台的当前绳长向量、期望绳长向量
    std::vector<std::vector<std::vector<double>>> homeRotT;// 各平台初始旋转矩阵的转置
    static std::vector<std::vector<double>> curContactPos;// 当前平台的接点坐标（在世界坐标系）

    std::vector<std::vector<double>> endCableForce;// 各末端的各绳索实际力（基于力传感器）
    std::vector<double> curEndCableForce;
    double optCableForce = 0.0;

    bool isFirst = true;
    bool optErr = false;
    bool useEndRot = true;// 是否考虑末端姿态。若为false，则姿态取0
    bool compTor = false;// 是否补偿力矩
    double motorTorqueLimitNm = 45.0;
    bool useSim = false;
    std::vector<std::vector<double>> lastSimVel;
    std::vector<std::vector<double>> lastSimPos;

    bool isUpdate = false;// 仅在输入被更新时，才会进行运算
private:
    QTimer *timer = nullptr;
    void threadLoop();
    bool isFirstLoop = true;
    double ctrlCycleMs;

    mutable QMutex _dataMutex;

    bool useCam = false;
    std::string sta = "equ";// 控制策略
    std::vector<std::vector<double>> impLastPlatformPose,impLastPlatformVel;
    double impLastTimeS = 0.0;// 上一次执行阻抗的时间
    int mode = -1;// -1：未设定； 0：错误指令 1：清华末端动平台模式 2：第三方传入期望力与力矩模式

    // 已知绳长和锚点座位置，计算平台对应的位姿（正运动学实时解算）。
    // 涉及到alglib优化算法库，通过约束“计算得到的平台位姿所对应的绳长与已知绳长差别最小”，实现平台位姿的计算
    std::vector<double> kine();

    std::vector<double> forceDistribution(std::vector<double> pose);// 基于平台位姿进行绳索力分配
    std::vector<std::vector<double>> getEndEffector_g(std::vector<double> pose);// 基于平台位姿，求各绳索接点的位置

//    bool calCableVecTheta(std::vector);

    typedef struct
    {
        double a, b;
    }my_constraint_data;

    std::vector<std::vector<double>> getContactPose(std::vector<double> PlatformPose);
    bool calGCCableVec = true;
    double cableMinF = 2.0;
    std::vector<std::vector<double>> cableMaxF;
    std::vector<double> curCableMaxF;

    std::vector<std::vector<double>> endForceByCable_a;// 各末端平台实际受绳索的作用力
    std::vector<std::vector<double>> lastEndForceByCable_a;// 上一次各末端平台实际受绳索的作用力

    std::vector<CdprDynModel> cdprDynModelVec;
signals:
    void displayInfoSignal(std::string info, std::string type);
    void update3DViewerSignal(QVector<QVector3D> targetPos, QVector<QVector3D> trajPos, QVector<QVector3D> anchorPos,
                              QVector<QVector3D> cableEndPos);
    void gcStartSignal();
    void gcCableForceResultSignal(std::vector<double> gcCableForce, std::vector<std::vector<double>> PlatformPose, std::vector<std::vector<double>> PlatformForce);// 各平台的各绳索的力分配(N)、各平台位姿与各平台实际受力
};

#endif // FORCESIMULATIONMODEL_H
