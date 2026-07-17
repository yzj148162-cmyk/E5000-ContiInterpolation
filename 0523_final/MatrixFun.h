#pragma once

#include <math.h>
#include <vector>
#include <algorithm>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <QDebug>

#define PI 3.14159265358979323846

namespace MatrixFun {
    struct CableLengthResult {
        double idealLength = 0.0;
        Eigen::Vector3d tangentPoint = Eigen::Vector3d::Zero();
        int layerIndex = -1;
        bool valid = false;
    };

    Eigen::MatrixXd vecToMatrix(const std::vector<std::vector<double>>& vec);// std::vector<std::vector<double>> 转 MatrixXd
    std::vector<Eigen::Vector3d> vecvecToVecv3d(const std::vector<std::vector<double>>& input);// std::vector<std::vector<double>>转eigen std::vector<Vector3d>
    Eigen::VectorXd vecToVxd(const std::vector<double>& input);

    // 向量叉乘
    double vecCross2D(const std::vector<double> vecA, const std::vector<double> vecB);// 二维向量（可用于判断向量a到向量b的转动方向是顺时针还是逆时针。当输出为正，则b在a的逆时针方向）
	std::vector<double> vecCross(const std::vector<double> vecA, const std::vector<double> vecB);
	// 向量点乘(n维)
	double vecDot(const std::vector<double> vecA, const std::vector<double> vecB);
    // 求两向量夹角(rad)
    double vecTheta(const std::vector<double> vecA, const std::vector<double> vecB);
	// 向量相加（相减）
	std::vector<double> vecPlus(const std::vector<double> vecA, const std::vector<double> vecB);
    std::vector<double> vecDiff(const std::vector<double> vecA, const std::vector<double> vecB);// A-B
	std::vector<std::vector<double>> vecPlus(const std::vector<std::vector<double>> vecA, const std::vector<std::vector<double>> vecB);// 列向量
	std::vector<std::vector<double>> vecDiff(const std::vector<std::vector<double>> vecA, const std::vector<std::vector<double>> vecB);// 列向量
	// 向量2范数
    double norm2(const std::vector<double> vec);
    int inferCableLayerIndex(const Eigen::Vector3d& contactPoint, const Eigen::Vector3d& anchorPoint);
    CableLengthResult cableLengthCalculate(const Eigen::Vector3d& contactPoint, const Eigen::Vector3d& anchorPoint,
                                           double pulleyRadius, int layerIndex = -1);
    CableLengthResult cableLengthCalculate(const std::vector<double>& contactPoint, const std::vector<double>& anchorPoint,
                                           double pulleyRadius, int layerIndex = -1);
    double cableLengthWithPulley(const std::vector<double>& contactPoint, const std::vector<double>& anchorPoint, double pulleyRadius);
    // 列向量转行向量
    std::vector<double> col2row(const std::vector<std::vector<double>> vec);

	// 矩阵乘法
	std::vector<std::vector<double>> matrixMulti(std::vector<std::vector<double>> matrixA, std::vector<std::vector<double>> matrixB);

	// 矩阵乘法升级版(模板函数)：mat1(m, n) * mat2(n, p) => result(m, p)
	// 注意：模板函数必须将声明和定义放在一起，否则使用它时会报错Link2019
	template<typename _Tp>
	std::vector<std::vector<_Tp>> matrix_mul(const std::vector<std::vector<_Tp>>& mat1, const std::vector<std::vector<_Tp>>& mat2) {
		std::vector<std::vector<_Tp>> result;
		int m1 = static_cast<int>(mat1.size()), n1 = static_cast<int>(mat1[0].size());
		int m2 = static_cast<int>(mat2.size()), n2 = static_cast<int>(mat2[0].size());
		if (n1 != m2) {
			// fprintf(stderr, "mat dimension dismatch\n");
			return result;
		}

		// 初始化（必要，否则报错：超出范围）
		// 行数
		result.resize(m1);
		for (int i = 0; i < m1; ++i) {
			// 列数
			result[i].resize(n2, (_Tp)0);
		}

		for (int y = 0; y < m1; ++y) {
			for (int x = 0; x < n2; ++x) {
				for (int t = 0; t < n1; ++t) {
					result[y][x] += mat1[y][t] * mat2[t][x];
				}
			}
		}

		return result;
	}

    // 生成mxm的单位矩阵
    std::vector<std::vector<double>> eye(const int m);

    /*四元数转欧拉角定轴XYZ
    输入：x,y,z,w　为四元数
    输出：roll(x)，pitch(y)，yaw(z)欧拉角
    **/
    void toEulerAngleStaticXYZ(const double x,const double y,const double z,const double w, double& rollX, double& pitchY, double& yawZ);

    // 欧拉角定轴XYZ转四元数
    void eulerStaticXYZtoQuaternion(const double rollX, const double pitchY, const double yawZ, double& x,double& y,double& z,double& w);

    // 五次多项式插值，返回位置、速度、加速度的轨迹和时间序列
    // 输出结构：第一层：位置,速度,加速度,时间  第二层：变量分类，数量取决于pos0.size()（如x,y,z,Rx,Ry,Rz,则六个）  第三层：各个变量基于时间的规划
    std::vector<std::vector<std::vector<double>>> quinPolyInter(std::vector<double> pos0, std::vector<double> vec0, std::vector<double> acc0,
                                                                std::vector<double> post, std::vector<double> vect, std::vector<double> acct, double t, double dt);

    std::vector<std::vector<double>> getTrans(std::vector<double> pose);// 求齐次变换矩阵（定轴欧拉角XYZ）
    std::vector<std::vector<double>> getRots(std::vector<double> pose);// 求旋转矩阵（定轴欧拉角XYZ）

    std::vector<double> rots2EulerXYZ(std::vector<std::vector<double>> rots);// 旋转矩阵转姿态角（定轴欧拉角XYZ）

    // DH法：输入DH参数，输出各个关节和末端到基座的齐次变换矩阵（前三个输入量都是结构参数，只有最后的关节角是变量）
    // 例：下标为0的元素，是第一个关节坐标系在基座坐标系下的表示。最后一个元素，是末端坐标系在基座坐标系下的表示
    // 注意：基座一般是第一个关节。当然为了建模方便，可以找一个固定端作为基座，输出的DH自然也会多一组
    // 注意：当某关节旋转时，其自身坐标系不会绕自身的Z轴旋转，而是影响下一个关节
    // 比如，当按照上述找一个固定端作为基座，则当第一个关节旋转时，该关节的x轴不会旋转，因此基座与该关节的DH参数中的关节角为定值
    std::vector<std::vector<std::vector<double>>> dhG_T_J(std::vector<double> jointTwistedThetaVec, std::vector<double> armLenVec,
                                                          std::vector<double> armDisVec, std::vector<double> jointTheta);

    // 已知某平面上三点在世界坐标系下的表示，求该平面的法向量
    // 其中O是OA和OB的交点，A、B为其它两点。输出的法向量按照右手定则，从OA到OB（OA到OB逆时针转动，大拇指方向即法向量
    std::vector<double> points2nor(std::vector<double> pointO,std::vector<double> pointA,std::vector<double> pointB);

    // 已知某坐标系轴的方向向量在世界坐标系下的表示，求该坐标系的旋转矩阵（定轴欧拉角XYZ）
    // 注意：以某个轴进行运算，必然缺失该轴方向上的旋转角度
    // 因此如果想要求得两坐标系之间/刚体的姿态角，必须至少使用两个轴，并分别搭配rots2EulerXYZ，弥补缺失
    // 以XY为例，分别使用rots2EulerXYZ，可得{0,b1,c1}和{a1,0,c1}，最终可组合得到完整姿态角{a1,b1,c1}
    std::vector<std::vector<double>> axisXDir2Rots(std::vector<double> axisXDirInG);// X轴
    std::vector<std::vector<double>> axisYDir2Rots(std::vector<double> axisYDirInG);// Y轴
    std::vector<std::vector<double>> axisZDir2Rots(std::vector<double> axisZDirInG);// Z轴

    // 以下三个函数，能够生成一条曲线。详见803程序（对应JFR论文，不是霄元）
    // ---------------------- 自然三次样条：求二阶导 M ----------------------
    // 输入：参数 t（长度 n），样本值 x（长度 n）
    // 输出：各节点处的二阶导 M（长度 n），自然样条边界条件 M[0]=M[n-1]=0
    // 采用三对角方程（Thomas 算法）求解
    std::vector<double> computeSecondDerivatives(const  std::vector<double> &t, const  std::vector<double> &x);
    // ---------------------- 在给定参数 s 处评估标量样条值 ----------------------
    // 使用已知节点 t、样本 x 与二阶导 M，在 s 处评价样条（区间插值）
    double splineEvalScalar(const std::vector<double> &t, const std::vector<double> &x, const std::vector<double> &M, double s);
    // ---------------------- 对 x,y,z 分量分别插值，返回三维点 ----------------------
    Eigen::Vector3d splineEvalVec3(const std::vector<double> &t, const std::vector<Eigen::Vector3d> &pts,
                                   const std::vector<double> &Mx, const std::vector<double> &My, const std::vector<double> &Mz, double s);

    std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat);//矩阵求逆

	// =========================== 求伪逆 ===========================
	template<typename _Tp>
	bool pinv(const std::vector<std::vector<_Tp>>& src, std::vector<std::vector<_Tp>>& dst, _Tp tolerance)
	{
		std::vector<std::vector<_Tp>> D, U, Vt;
		if (svd(src, D, U, Vt) != 0) {
			// fprintf(stderr, "singular value decomposition fail\n");
			return false;
		}

		int m = src.size();
		int n = src[0].size();

		std::vector<std::vector<_Tp>> Drecip, DrecipT, Ut, V;

		transpose(Vt, V);
		transpose(U, Ut);

		if (m < n)
			std::swap(m, n);

		Drecip.resize(n);
		for (int i = 0; i < n; ++i) {
			Drecip[i].resize(m, (_Tp)0);

			if (D[i][0] > tolerance)
				Drecip[i][i] = 1.0f / D[i][0];
		}

		if (src.size() < src[0].size())
			transpose(Drecip, DrecipT);
		else
			DrecipT = Drecip;

		std::vector<std::vector<_Tp>> tmp = matrix_mul(V, DrecipT);
		dst = matrix_mul(tmp, Ut);

		return true;
	}

	// 矩阵转置
	template<typename _Tp>
	void transpose(const std::vector<std::vector<_Tp>>& matrixA, std::vector<std::vector<_Tp>>& matrixAT) {
		matrixAT.resize(matrixA[0].size());
		for (int i = 0; i < matrixA[0].size(); ++i) {
			matrixAT[i].resize(matrixA.size());
		}

		for (int i = 0; i < matrixA.size(); i++) {
			for (int j = 0; j < matrixA[0].size(); j++) {
				matrixAT[j][i] = matrixA[i][j];
			}
		}
	}
	
	// ================================= 矩阵奇异值分解 =================================
//	template<typename _Tp>
//	static void JacobiSVD(std::vector<std::vector<_Tp>>& At,
//		std::vector<std::vector<_Tp>>& _W, std::vector<std::vector<_Tp>>& Vt)
//	{
//    double minval = __FLT_MIN__;
//    _Tp eps = (_Tp)(__FLT_EPSILON__ * 2.0);
//		const int m = At[0].size();
//		const int n = _W.size();
//		const int n1 = m; // urows
//		std::vector<double> W(n, 0.);

//		for (int i = 0; i < n; i++) {
//			double sd{ 0. };
//			for (int k = 0; k < m; k++) {
//				_Tp t = At[i][k];
//				sd += (double)t*t;
//			}
//			W[i] = sd;

//			for (int k = 0; k < n; k++)
//				Vt[i][k] = 0;
//			Vt[i][i] = 1;
//		}

//		int max_iter = std::max(m, 30);
//		for (int iter = 0; iter < max_iter; iter++) {
//			bool changed = false;
//			_Tp c, s;

//			for (int i = 0; i < n - 1; i++) {
//				for (int j = i + 1; j < n; j++) {
//					_Tp *Ai = At[i].data(), *Aj = At[j].data();
//					double a = W[i], p = 0, b = W[j];

//					for (int k = 0; k < m; k++)
//						p += (double)Ai[k] * Aj[k];

//          if (fabs(p) <= eps * sqrt((double)a*b))//edit if (std::abs(p) <= eps * std::sqrt((double)a*b))
//						continue;

//					p *= 2;
//          double beta = a - b, gamma = hypot((double)p, beta);// edit
//					if (beta < 0) {
//						double delta = (gamma - beta)*0.5;
//            s = (_Tp)sqrt(delta / gamma);// edit
//						c = (_Tp)(p / (gamma*s * 2));
//					}
//					else {
//            c = (_Tp)sqrt((gamma + beta) / (gamma * 2));// edit
//						s = (_Tp)(p / (gamma*c * 2));
//					}

//					a = b = 0;
//					for (int k = 0; k < m; k++) {
//						_Tp t0 = c * Ai[k] + s * Aj[k];
//						_Tp t1 = -s * Ai[k] + c * Aj[k];
//						Ai[k] = t0; Aj[k] = t1;

//						a += (double)t0*t0; b += (double)t1*t1;
//					}
//					W[i] = a; W[j] = b;

//					changed = true;

//					_Tp *Vi = Vt[i].data(), *Vj = Vt[j].data();

//					for (int k = 0; k < n; k++) {
//						_Tp t0 = c * Vi[k] + s * Vj[k];
//						_Tp t1 = -s * Vi[k] + c * Vj[k];
//						Vi[k] = t0; Vj[k] = t1;
//					}
//				}
//			}

//			if (!changed)
//				break;
//		}

//		for (int i = 0; i < n; i++) {
//			double sd{ 0. };
//			for (int k = 0; k < m; k++) {
//				_Tp t = At[i][k];
//				sd += (double)t*t;
//			}
//      W[i] = sqrt(sd);//edit
//		}

//		for (int i = 0; i < n - 1; i++) {
//			int j = i;
//			for (int k = i + 1; k < n; k++) {
//				if (W[j] < W[k])
//					j = k;
//			}
//			if (i != j) {
//				std::swap(W[i], W[j]);

//				for (int k = 0; k < m; k++)
//					std::swap(At[i][k], At[j][k]);

//				for (int k = 0; k < n; k++)
//					std::swap(Vt[i][k], Vt[j][k]);
//			}
//		}

//		for (int i = 0; i < n; i++)
//			_W[i][0] = (_Tp)W[i];

//		//srand(time(nullptr));

//		for (int i = 0; i < n1; i++) {
//			double sd = i < n ? W[i] : 0;

//			for (int ii = 0; ii < 100 && sd <= minval; ii++) {
//				// if we got a zero singular value, then in order to get the corresponding left singular vector
//				// we generate a random vector, project it to the previously computed left singular vectors,
//				// subtract the projection and normalize the difference.
//				const _Tp val0 = (_Tp)(1. / m);
//				for (int k = 0; k < m; k++) {
//					// 声明静态局部变量，从而使其只会被初始化一次并在函数结束后仍存在
//					static long seed;
//					seed += 1;
//					// returns a pseudo-random number 0 through 32767.
//					// Parameter holdrand is random seed, which is modified on each call.
//					/*unsigned int rng = (int)rands_(&seed);*/

//					//unsigned int rng = (int)rands_(&seed) % 4294967295;
//					//_Tp val = (rng & 256) != 0 ? val0 : -val0;
//					//At[i][k] = val;

//					//unsigned int rng = 1;
//					unsigned int rng = rand() % 4294967295; // 2^32 - 1
//					_Tp val = (rng & 256) != 0 ? val0 : -val0;
//					At[i][k] = val;
//				}
//				for (int iter = 0; iter < 2; iter++) {
//					for (int j = 0; j < i; j++) {
//						sd = 0;
//						for (int k = 0; k < m; k++)
//							sd += At[i][k] * At[j][k];
//						_Tp asum = 0;
//						for (int k = 0; k < m; k++) {
//							_Tp t = (_Tp)(At[i][k] - sd * At[j][k]);
//							At[i][k] = t;
//							asum += fabs_(t);// edit
//						}
//						asum = asum > eps * 100 ? 1 / asum : 0;
//						for (int k = 0; k < m; k++)
//							At[i][k] *= asum;
//					}
//				}

//				sd = 0;
//				for (int k = 0; k < m; k++) {
//					_Tp t = At[i][k];
//					sd += (double)t*t;
//				}
//        sd = sqrt(sd);//edit
//			}

//			_Tp s = (_Tp)(sd > minval ? 1 / sd : 0.);
//			for (int k = 0; k < m; k++)
//				At[i][k] *= s;
//		}
//	}

//	template<typename _Tp>
//	int svd(const std::vector<std::vector<_Tp>>& matSrc,
//		std::vector<std::vector<_Tp>>& matD, std::vector<std::vector<_Tp>>& matU, std::vector<std::vector<_Tp>>& matVt)
//	{
//		int m = matSrc.size();
//		int n = matSrc[0].size();
//		for (const auto& sz : matSrc) {
//			if (n != sz.size()) {
//				//fprintf(stderr, "matrix dimension dismatch\n");
//				return -1;
//			}
//		}

//		bool at = false;
//		if (m < n) {
//			std::swap(m, n);
//			at = true;
//		}

//		matD.resize(n);
//		for (int i = 0; i < n; ++i) {
//			matD[i].resize(1, (_Tp)0);
//		}
//		matU.resize(m);
//		for (int i = 0; i < m; ++i) {
//			matU[i].resize(m, (_Tp)0);
//		}
//		matVt.resize(n);
//		for (int i = 0; i < n; ++i) {
//			matVt[i].resize(n, (_Tp)0);
//		}
//		std::vector<std::vector<_Tp>> tmp_u = matU, tmp_v = matVt;

//		std::vector<std::vector<_Tp>> tmp_a, tmp_a_;
//		if (!at)
//			transpose(matSrc, tmp_a);
//		else
//			tmp_a = matSrc;

//		if (m == n) {
//			tmp_a_ = tmp_a;
//		}
//		else {
//			tmp_a_.resize(m);
//			for (int i = 0; i < m; ++i) {
//				tmp_a_[i].resize(m, (_Tp)0);
//			}
//			for (int i = 0; i < n; ++i) {
//				tmp_a_[i].assign(tmp_a[i].begin(), tmp_a[i].end());
//			}
//		}
//		JacobiSVD(tmp_a_, matD, tmp_v);

//		if (!at) {
//			transpose(tmp_a_, matU);
//			matVt = tmp_v;
//		}
//		else {
//			transpose(tmp_v, matU);
//			matVt = tmp_a_;
//		}

//		return 0;
//	}
}
