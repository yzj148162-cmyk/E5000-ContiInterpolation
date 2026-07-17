#include "MatrixFun.h"

#include <algorithm>
#include <cmath>

Eigen::MatrixXd MatrixFun::vecToMatrix(const std::vector<std::vector<double>>& vec) {
    if (vec.empty()) return Eigen::MatrixXd(0, 0);

    int rows = vec.size();
    int cols = vec[0].size();

    // 1. 这种方法要求必须使用 RowMajor（行优先）的 Matrix
    // 或者让 Eigen 自己处理赋值时的布局转换
    Eigen::MatrixXd mat(rows, cols);

    for (int i = 0; i < rows; ++i) {
        // 将 std::vector 的数据指针映射为 Eigen 的行向量
        // Map<const RowVectorXd>(指针, 列数)
        mat.row(i) = Eigen::Map<const Eigen::RowVectorXd>(vec[i].data(), cols);
    }

    return mat;
}

std::vector<Eigen::Vector3d> MatrixFun::vecvecToVecv3d(const std::vector<std::vector<double>>& input) {
    std::vector<Eigen::Vector3d> result;

    // 1. 【重要】预分配内存，提升性能
    result.reserve(input.size());

    for (const auto& row : input) {
        // 2. 安全检查：确保只有 3 维数据才转换
        if (row.size() != 3) {
            continue;
        }

        // 3. 核心转换
        // 方法 A：构造函数（笨拙但直观）
        // result.emplace_back(row[0], row[1], row[2]);

        // 方法 B：使用 Map（优雅，直接映射内存）
        // 将 std::vector 的数据指针直接映射为 Vector3d 并拷贝进去
        result.emplace_back(Eigen::Map<const Eigen::Vector3d>(row.data()));
    }

    return result;
}

Eigen::VectorXd MatrixFun::vecToVxd(const std::vector<double>& input){
    return Eigen::Map<const Eigen::VectorXd>(input.data(), input.size());
}

double MatrixFun::vecCross2D(const std::vector<double> vecA, const std::vector<double> vecB) {
    if (vecA.size() == 2 && vecB.size() == 2) {
        return vecA[0] * vecB[1] - vecA[1] * vecB[0];
    }
    else {
        return 0;
    }
}

std::vector<double> MatrixFun::vecCross(const std::vector<double> vecA, const std::vector<double> vecB) {
	if (vecA.size() == 3 && vecB.size() == 3) {
		std::vector<double> result;
		result.push_back(vecA[1] * vecB[2] - vecA[2] * vecB[1]);
		result.push_back(vecA[2] * vecB[0] - vecA[0] * vecB[2]);
		result.push_back(vecA[0] * vecB[1] - vecA[1] * vecB[0]);
		return result;
	}
	else {
		return {};
	}
}

double MatrixFun::vecDot(const std::vector<double> vecA, const std::vector<double> vecB) {
	if (vecA.size() == vecB.size() && vecA.size() != 0) {
		double result(0.0);
		for (int i = 0; i < vecA.size(); ++i) {
			result += vecA[i]*vecB[i];
		}
		return result;
	}
	else
		return {};
}

double MatrixFun::vecTheta(const std::vector<double> vecA, const std::vector<double> vecB){
    double result = acos(vecDot(vecA,vecB)/(norm2(vecA)*norm2(vecB)));
    if(std::isnan(result))
        return 0.0;
    else
        return result;
}

std::vector<double>  MatrixFun::vecPlus(const std::vector<double> vecA, const std::vector<double> vecB) {
	if (vecA.size() == vecB.size()) {
		std::vector<double> result;
		result.resize(vecA.size());
		for (int i = 0; i < vecA.size(); ++i) {
			result[i] = vecA[i] + vecB[i];
		}
		return result;
	}
	else {
		return {};
	}
}

std::vector<double>  MatrixFun::vecDiff(const std::vector<double> vecA, const std::vector<double> vecB) {
	if (vecA.size() == vecB.size()) {
		std::vector<double> result;
		result.resize(vecA.size());
		for (int i = 0; i < vecA.size(); ++i) {
			result[i] = vecA[i] - vecB[i];
		}
		return result;
	}
	else {
		return {};
	}
}

std::vector<std::vector<double>> MatrixFun::vecPlus(const std::vector<std::vector<double>> vecA, const std::vector<std::vector<double>> vecB) {
	if (vecA.size() == vecB.size()) {
		std::vector<std::vector<double>> result(vecA.size());
		for (int i = 0; i < vecA.size(); ++i) {
			if (vecA[i].size() == vecB[i].size() && vecA[i].size() == 1) {
				result[i].push_back(vecA[i][0]+ vecB[i][0]);
			}
			else
				return {};
		}
		return result;
	}
	else {
		return {};
	}
}

std::vector<std::vector<double>> MatrixFun::vecDiff(const std::vector<std::vector<double>> vecA, const std::vector<std::vector<double>> vecB) {
	if (vecA.size() == vecB.size()) {
		std::vector<std::vector<double>> result(vecA.size());
		for (int i = 0; i < vecA.size(); ++i) {
			if (vecA[i].size() == vecB[i].size() && vecA[i].size() == 1) {
				result[i].push_back(vecA[i][0] - vecB[i][0]);
			}
			else
				return {};
		}
		return result;
	}
	else {
		return {};
	}
}

std::vector<double> MatrixFun::col2row(const std::vector<std::vector<double> > vec){
    std::vector<double> result;
    for(int i=0;i<vec.size();++i){
        result.push_back(vec[i][0]);
    }
    return result;
}

double MatrixFun::norm2(const std::vector<double> vec) {
	double result = 0;
	for (int i = 0; i < vec.size(); ++i) {
		result += pow(vec[i],2);
	}
	return sqrt(result);
}

int MatrixFun::inferCableLayerIndex(const Eigen::Vector3d& contactPoint, const Eigen::Vector3d& anchorPoint)
{
    return anchorPoint.z() >= contactPoint.z() ? 1 : 0;
}

MatrixFun::CableLengthResult MatrixFun::cableLengthCalculate(
        const Eigen::Vector3d& contactPoint,
        const Eigen::Vector3d& anchorPoint,
        double pulleyRadius,
        int layerIndex)
{
    CableLengthResult result;
    result.tangentPoint = anchorPoint;
    result.layerIndex = (layerIndex == 0 || layerIndex == 1)
            ? layerIndex
            : inferCableLayerIndex(contactPoint, anchorPoint);

    const Eigen::Vector3d directCable = anchorPoint - contactPoint;
    const double directLength = directCable.norm();
    result.idealLength = std::isfinite(directLength) ? directLength : 0.0;

    if (!std::isfinite(result.idealLength) || pulleyRadius <= 0.0) {
        result.valid = std::isfinite(result.idealLength);
        return result;
    }

    const double r = pulleyRadius;
    const int index = result.layerIndex;
    const double theta = (index == 1)
            ? atan2(std::abs(anchorPoint.x() - contactPoint.x()),
                    std::abs(anchorPoint.z() - contactPoint.z()))
            : atan2(std::abs(anchorPoint.y() - contactPoint.y()),
                    std::abs(anchorPoint.z() - contactPoint.z()));

    Eigen::Vector3d circleCenter = Eigen::Vector3d::Zero();
    if (index == 1) {
        circleCenter.x() = anchorPoint.x() > 0.0 ? anchorPoint.x() - r * sin(theta)
                                                 : anchorPoint.x() + r * sin(theta);
        circleCenter.y() = anchorPoint.y() > 0.0 ? anchorPoint.y() - r * cos(theta)
                                                 : anchorPoint.y() + r * cos(theta);
        circleCenter.z() = anchorPoint.z();
    } else {
        circleCenter.x() = anchorPoint.x() > 0.0 ? anchorPoint.x() - r * cos(theta)
                                                 : anchorPoint.x() + r * cos(theta);
        circleCenter.y() = anchorPoint.y() > 0.0 ? anchorPoint.y() - r * sin(theta)
                                                 : anchorPoint.y() + r * sin(theta);
        circleCenter.z() = anchorPoint.z();
    }

    const Eigen::Vector3d vecOA = contactPoint - circleCenter;
    const Eigen::Vector3d vecOB = anchorPoint - circleCenter;
    const double normOA = vecOA.norm();
    const double normOB = vecOB.norm();
    if (!std::isfinite(normOA) || !std::isfinite(normOB) ||
            normOA <= 1e-12 || normOB <= 1e-12) {
        result.valid = false;
        return result;
    }

    const double cosAOB = std::clamp(vecOA.dot(vecOB) / (normOA * normOB), -1.0, 1.0);
    const double cosAOE = std::clamp(r / normOA, -1.0, 1.0);
    const double thetaAOB = acos(cosAOB);
    const double thetaAOE = acos(cosAOE);
    const double thetaBOE = 2.0 * PI - thetaAOB - thetaAOE;
    if (!std::isfinite(thetaBOE)) {
        result.valid = false;
        return result;
    }

    const double offset = r + r * cos(thetaBOE - PI / 2.0);
    Eigen::Vector3d tangentPoint = Eigen::Vector3d::Zero();
    if (index == 1) {
        tangentPoint.z() = circleCenter.z() + r * cos(thetaBOE - PI / 2.0);
        tangentPoint.x() = anchorPoint.x() > 0.0 ? anchorPoint.x() - offset * sin(theta)
                                                 : anchorPoint.x() + offset * sin(theta);
        tangentPoint.y() = anchorPoint.y() > 0.0 ? anchorPoint.y() - offset * cos(theta)
                                                 : anchorPoint.y() + offset * cos(theta);
    } else {
        tangentPoint.z() = circleCenter.z() - r * cos(thetaBOE - PI / 2.0);
        tangentPoint.x() = anchorPoint.x() > 0.0 ? anchorPoint.x() - offset * cos(theta)
                                                 : anchorPoint.x() + offset * cos(theta);
        tangentPoint.y() = anchorPoint.y() > 0.0 ? anchorPoint.y() - offset * sin(theta)
                                                 : anchorPoint.y() + offset * sin(theta);
    }

    const double idealLength = (contactPoint - tangentPoint).norm() + r * thetaBOE;
    if (!std::isfinite(idealLength)) {
        result.valid = false;
        return result;
    }

    result.idealLength = idealLength;
    result.tangentPoint = tangentPoint;
    result.valid = true;
    return result;
}

MatrixFun::CableLengthResult MatrixFun::cableLengthCalculate(
        const std::vector<double>& contactPoint,
        const std::vector<double>& anchorPoint,
        double pulleyRadius,
        int layerIndex)
{
    if (contactPoint.size() < 3 || anchorPoint.size() < 3) {
        return CableLengthResult();
    }

    return cableLengthCalculate(Eigen::Vector3d(contactPoint[0], contactPoint[1], contactPoint[2]),
                                Eigen::Vector3d(anchorPoint[0], anchorPoint[1], anchorPoint[2]),
                                pulleyRadius,
                                layerIndex);
}

double MatrixFun::cableLengthWithPulley(const std::vector<double>& contactPoint, const std::vector<double>& anchorPoint, double pulleyRadius) {
    return cableLengthCalculate(contactPoint, anchorPoint, pulleyRadius).idealLength;
}

std::vector<std::vector<double>> MatrixFun::matrixMulti(std::vector<std::vector<double>> matrixA, std::vector<std::vector<double>> matrixB) {
	std::vector<std::vector<double>> result;

	// 初始化（必要，否则报错：超出范围）
	// 行数
	result.resize(matrixA.size());
	for (int i = 0; i < matrixA.size(); ++i) {
		// 列数
		result[i].resize(matrixB[i].size());
	}

	// i代表第i行
	for (int i = 0; i < matrixA.size(); ++i) {
		// j代表第j列
		for (int j = 0; j < matrixB[i].size(); ++j) {
			for (int a = 0; a < matrixA[i].size(); ++a) {
				/*result[i][j] += matrixA[i * matrixA[i].size() + a - 1] * matrixB[(a - 1) * matrixB[i].size() + j];*/
				result[i][j] += matrixA[i][a] * matrixB[a][j];
			}
		}
	}
	return result;
}

std::vector<std::vector<double>> MatrixFun::eye(const int m){
    std::vector<std::vector<double>> result(m,std::vector<double>(m,0.0));
    for(int i=0;i<m;++i){
        for(int ii=0;ii<m;++ii){
            if(i == ii){
                result[i][ii] = 1.0;
            }
        }
    }
    return result;
}

void MatrixFun::toEulerAngleStaticXYZ(const double x,const double y,const double z,const double w, double& rollX, double& pitchY, double& yawZ){
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    rollX = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitchY = copysign(PI / 2, sinp); // use 90 degrees if out of range
    else
        pitchY = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yawZ = atan2(siny_cosp, cosy_cosp);
//    return yaw;
}

void MatrixFun::eulerStaticXYZtoQuaternion(const double rollX, const double pitchY, const double yawZ, double &x, double &y, double &z, double &w){
    const double deg2rad = PI / 180.0;
    double alpha = rollX  * deg2rad;  // X
    double beta  = pitchY * deg2rad;  // Y
    double gamma = yawZ   * deg2rad;  // Z

    Eigen::Quaterniond q =
            Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(beta,  Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());
    q.normalize();

    w = q.w();
    x = q.x();
    y = q.y();
    z = q.z();
}

std::vector<std::vector<std::vector<double>>> MatrixFun::quinPolyInter(std::vector<double> pos0, std::vector<double> vec0, std::vector<double> acc0,
                                                                       std::vector<double> post, std::vector<double> vect, std::vector<double> acct, double t, double dt) {
    std::vector<std::vector<std::vector<double>>> result(4);

  if (pos0.size() != vec0.size() || pos0.size() != acc0.size() || pos0.size() != post.size() || pos0.size() != vect.size() || pos0.size() != acct.size()) {
    return {};
  }
  for (int i = 0; i < 4; ++i)
    result[i].resize(pos0.size());

  std::vector<double> a0(pos0), a1(vec0), a2(acc0), a3, a4, a5;
  for (int i = 0; i < pos0.size(); ++i) {
    a3.push_back((20.0 * (post[i] - pos0[i]) - (8.0*vect[i] + 12.0*vec0[i])*t + (acct[i] - 3.0*acc0[i])*pow(t, 2.0)) / (2.0*pow(t, 3.0)));
    a4.push_back((-30.0*(post[i] - pos0[i]) + (14.0*vect[i] + 16.0*vec0[i])*t - (2.0*acct[i] - 3.0*acc0[i])*pow(t, 2.0)) / (2.0*pow(t, 4.0)));
    a5.push_back((12.0*(post[i] - pos0[i]) - (6.0*vect[i] + 6.0*vec0[i])*t + (acct[i] - acc0[i])*pow(t, 2.0)) / (2.0*pow(t, 5.0)));
  }

  double k = 0.0;
  for (int step = 0; step <= t / dt; ++step) {
    for (int i = 0; i < pos0.size(); ++i) {
      result[3][i].push_back(k);
      result[0][i].push_back(a0[i] + a1[i] * k + a2[i] * pow(k, 2.0) + a3[i] * pow(k, 3.0) + a4[i] * pow(k, 4.0) + a5[i] * pow(k, 5.0));
      result[1][i].push_back(a1[i] + 2.0*a2[i] * k + 3.0*a3[i] * pow(k, 2.0) + 4.0*a4[i] * pow(k, 3.0) + 5.0*a5[i] * pow(k, 4.0));
      result[2][i].push_back(2.0*a2[i] + 6.0*a3[i]*k + 12.0*a4[i]* pow(k, 2.0) + 20.0*a5[i]* pow(k, 3.0));
    }
    k += dt;
  }

  return result;
}

std::vector<std::vector<double>> MatrixFun::getTrans(std::vector<double> pose){
    std::vector<std::vector<double>> Rotx = {{ 1,0,0,0 },
                       { 0,cos(pose[3]),-sin(pose[3]),0 },
                       { 0,sin(pose[3]),cos(pose[3]),0 },
                       { 0,0,0,1 } };
    std::vector<std::vector<double>> Roty = {{cos(pose[4]),0,sin(pose[4]),0 },
                       { 0,1,0,0 },
                       { -sin(pose[4]),0,cos(pose[4]),0},
                       { 0,0,0,1 } };
    std::vector<std::vector<double>> Rotz = {{ cos(pose[5]),-sin(pose[5]),0,0 },
                       { sin(pose[5]),cos(pose[5]),0,0 },
                       { 0,0,1,0 },
                       { 0,0,0,1 } };
    std::vector<std::vector<double>> Txyz = {{ 1,0,0,pose[0] },
                       { 0,1,0,pose[1] },
                       { 0,0,1,pose[2] },
                       { 0,0,0,1 } };
    //Txyz不断右乘Rz\Ry\Rx，即生成最终的齐次变换矩阵
    return MatrixFun::matrixMulti(MatrixFun::matrixMulti(MatrixFun::matrixMulti(Txyz, Rotz), Roty), Rotx);
}

std::vector<std::vector<double>> MatrixFun::getRots(std::vector<double> pose){
    std::vector<std::vector<double>> Rotx = {{1,0,0},
                       {0,cos(pose[3]),-sin(pose[3])},
                       {0,sin(pose[3]),cos(pose[3])}};
    std::vector<std::vector<double>> Roty = {{cos(pose[4]),0,sin(pose[4])},
                       {0,1,0},
                       {-sin(pose[4]),0,cos(pose[4])}};
    std::vector<std::vector<double>> Rotz = {{cos(pose[5]),-sin(pose[5]),0},
                       {sin(pose[5]),cos(pose[5]),0},
                       {0,0,1}};
    //Txyz不断右乘Rz\Ry\Rx，即生成最终的齐次变换矩阵
    return MatrixFun::matrixMulti(MatrixFun::matrixMulti(Rotz, Roty), Rotx);
}

std::vector<double> MatrixFun::rots2EulerXYZ(std::vector<std::vector<double> > rots){
    std::vector<double> result(3,0.0);

    result[0] = atan2(rots[2][1],rots[2][2]);//X
    result[1] = atan2(-rots[2][0],sqrt(pow(rots[2][1],2)+pow(rots[2][2],2)));//Y
    result[2] = atan2(rots[1][0],rots[0][0]);//Z

    return result;
}

std::vector<std::vector<std::vector<double>>> MatrixFun::dhG_T_J(std::vector<double> jointTwistedThetaVec, std::vector<double> armLenVec,
                                                                 std::vector<double> armDisVec, std::vector<double> jointTheta){
    std::vector<std::vector<double>> dhPara(jointTwistedThetaVec.size(),std::vector<double>(4,0.0));
    for(int i=0;i<dhPara.size();++i){
        dhPara[i] = {jointTwistedThetaVec[i],armLenVec[i],armDisVec[i],jointTheta[i]};// 扭角，杆长，杆距，关节转角
    }
    std::vector<std::vector<std::vector<double>>> G_T_J(jointTwistedThetaVec.size(),std::vector<std::vector<double>>(4,std::vector<double>(4,0.0)));// 各个关节到基座的齐次变换矩阵
    std::vector<std::vector<double>> tmpLastG_T_J = {{1.0,0.0,0.0,0.0},
                                                     {0.0,1.0,0.0,0.0},
                                                     {0.0,0.0,1.0,0.0},
                                                     {0.0,0.0,0.0,1.0}};
    for(int i=0;i<jointTwistedThetaVec.size();++i){
        std::vector<std::vector<double>> tmp = {{cos(dhPara[i][3]),-cos(dhPara[i][0])*sin(dhPara[i][3]),sin(dhPara[i][0])*sin(dhPara[i][3]),dhPara[i][1]*cos(dhPara[i][3])},
                                                {sin(dhPara[i][3]),cos(dhPara[i][0])*cos(dhPara[i][3]),-sin(dhPara[i][0])*cos(dhPara[i][3]),dhPara[i][1]*sin(dhPara[i][3])},
                                                {0.0,sin(dhPara[i][0]),cos(dhPara[i][0]),dhPara[i][2]},
                                                {0.0,0.0,0.0,1.0}};
        tmpLastG_T_J = MatrixFun::matrix_mul(tmpLastG_T_J,tmp);
        G_T_J[i] = tmpLastG_T_J;
    }
    return G_T_J;
}

std::vector<double> MatrixFun::points2nor(std::vector<double> pointO,std::vector<double> pointA,std::vector<double> pointB){
    return vecCross(vecDiff(pointA,pointO),vecDiff(pointB,pointO));
}

std::vector<std::vector<double>> MatrixFun::axisXDir2Rots(std::vector<double> axisXDirInG){
    Eigen::Vector3d VectorBefore(1.0, 0.0, 0.0);
    Eigen::Vector3d VectorAfter(axisXDirInG[0], axisXDirInG[1], axisXDirInG[2]);
    Eigen::Matrix3d rotationMatrix = Eigen::Quaterniond::FromTwoVectors(VectorBefore, VectorAfter).toRotationMatrix();
    std::vector<std::vector<double>> result(3,std::vector<double>(3));
    for(int i=0;i<3;++i)
        for(int ii=0;ii<3;++ii)
            result[i][ii] = rotationMatrix(i,ii);
    return result;
}

std::vector<std::vector<double>> MatrixFun::axisYDir2Rots(std::vector<double> axisYDirInG){
    Eigen::Vector3d VectorBefore(0.0, 1.0, 0.0);
    Eigen::Vector3d VectorAfter(axisYDirInG[0], axisYDirInG[1], axisYDirInG[2]);
    Eigen::Matrix3d rotationMatrix = Eigen::Quaterniond::FromTwoVectors(VectorBefore, VectorAfter).toRotationMatrix();
    std::vector<std::vector<double>> result(3,std::vector<double>(3));
    for(int i=0;i<3;++i)
        for(int ii=0;ii<3;++ii)
            result[i][ii] = rotationMatrix(i,ii);
    return result;
}

std::vector<std::vector<double>> MatrixFun::axisZDir2Rots(std::vector<double> axisZDirInG){
    Eigen::Vector3d VectorBefore(0.0, 0.0, 1.0);
    Eigen::Vector3d VectorAfter(axisZDirInG[0], axisZDirInG[1], axisZDirInG[2]);
    Eigen::Matrix3d rotationMatrix = Eigen::Quaterniond::FromTwoVectors(VectorBefore, VectorAfter).toRotationMatrix();
    std::vector<std::vector<double>> result(3,std::vector<double>(3));
    for(int i=0;i<3;++i)
        for(int ii=0;ii<3;++ii)
            result[i][ii] = rotationMatrix(i,ii);
    return result;
}

std::vector<double> MatrixFun::computeSecondDerivatives(const  std::vector<double> &t, const  std::vector<double> &x) {
    int n = (int)t.size();
    std::vector<double> M(n, 0.0);
    if (n <= 2) return M; // 点太少，返回全 0

    int m = n - 2; // 待求解的 M[1..n-2] 个未知数
    std::vector<double> a(m), b(m), c(m), d(m);

    // 构造三对角矩阵和右端项
    for (int i = 1; i <= n-2; ++i) {
        double h1 = t[i] - t[i-1];
        double h2 = t[i+1] - t[i];
        // 防止参数退化
        if (h1 <= 0) h1 = 1e-9;
        if (h2 <= 0) h2 = 1e-9;
        a[i-1] = h1;
        b[i-1] = 2.0*(h1 + h2);
        c[i-1] = h2;
        double dd = ((x[i+1]-x[i]) / h2 - (x[i]-x[i-1]) / h1);
        d[i-1] = 6.0 * dd;
    }

    // Thomas 算法：消元
    for (int i = 1; i < m; ++i) {
        double w = a[i] / b[i-1];
        b[i] -= w * c[i-1];
        d[i] -= w * d[i-1];
    }

    // 回代求解
    std::vector<double> sol(m, 0.0);
    if (m > 0) {
        sol[m-1] = d[m-1] / b[m-1];
        for (int i = m-2; i >= 0; --i) sol[i] = (d[i] - c[i] * sol[i+1]) / b[i];
    }

    // 把解填回 M（边界 M0 = Mn-1 = 0）
    for (int i = 1; i <= n-2; ++i) M[i] = sol[i-1];
    M[0] = 0.0; M[n-1] = 0.0;
    return M;
}

double MatrixFun::splineEvalScalar(const std::vector<double> &t, const std::vector<double> &x, const std::vector<double> &M, double s) {
    int n = (int)t.size();
    if (s <= t.front()) return x.front();
    if (s >= t.back()) return x.back();

    // 找到区间 i，使得 t[i] <= s <= t[i+1]
    int idx = int(lower_bound(t.begin(), t.end(), s) - t.begin());
    if (idx == 0) idx = 0;
    if (idx >= n) idx = n-1;
    if (t[idx] > s) --idx;
    if (idx >= n-1) idx = n-2;

    double h = t[idx+1] - t[idx];
    if (h <= 0) return x[idx];
    double A = (t[idx+1] - s) / h;
    double B = (s - t[idx]) / h;

    // 样条插值公式（Hermite 形式）
    double term1 = A * x[idx] + B * x[idx+1];
    double term2 = ((A*A*A - A) * M[idx] + (B*B*B - B) * M[idx+1]) * (h*h) / 6.0;
    return term1 + term2;
}

// ---------------------- 对 x,y,z 分量分别插值，返回三维点 ----------------------
Eigen::Vector3d MatrixFun::splineEvalVec3(const std::vector<double> &t, const std::vector<Eigen::Vector3d> &pts,
                                          const std::vector<double> &Mx, const std::vector<double> &My, const std::vector<double> &Mz, double s) {
    int n = (int)pts.size();
    std::vector<double> xs(n), ys(n), zs(n);
    for (int i = 0; i < n; ++i) { xs[i] = pts[i].x(); ys[i] = pts[i].y(); zs[i] = pts[i].z(); }

    double vx = splineEvalScalar(t, xs, Mx, s);
    double vy = splineEvalScalar(t, ys, My, s);
    double vz = splineEvalScalar(t, zs, Mz, s);
    return Eigen::Vector3d(vx, vy, vz);
}

std::vector<std::vector<double>> MatrixFun::inv(std::vector<std::vector<double>> a){
    std::vector<std::vector<double>> b(3,std::vector<double>(3));

    int i, j, k;
    double max, temp;
    // 定义一个临时矩阵t
    std::vector<std::vector<double>> t(3,std::vector<double>(3));
    // 将a矩阵临时存放在矩阵t[n][n]中
    for (i = 0; i < 3; i++){
        for (j = 0; j < 3; j++){
            t[i][j] = a[i][j];
        }
    }
    // 初始化B矩阵为单位矩阵
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            b[i][j] = (i == j) ? (double)1 : 0;
        }
    }
    // 进行列主消元，找到每一列的主元
    for (i = 0; i < 3; i++)
    {
        max = t[i][i];
        // 用于记录每一列中的第几个元素为主元
        k = i;
        // 寻找每一列中的主元元素
        for (j = i + 1; j < 3; j++)
        {
            if (fabs(t[j][i]) > fabs(max))
            {
                max = t[j][i];
                k = j;
            }
        }
        //cout<<"the max number is "<<max<<endl;
        // 如果主元所在的行不是第i行，则进行行交换
        if (k != i)
        {
            // 进行行交换
            for (j = 0; j < 3; j++)
            {
                temp = t[i][j];
                t[i][j] = t[k][j];
                t[k][j] = temp;
                // 伴随矩阵B也要进行行交换
                temp = b[i][j];
                b[i][j] = b[k][j];
                b[k][j] = temp;
            }
        }
        if (fabs(t[i][i]) < 1e-6)
        {
            // cout << "\nthe matrix does not exist inverse matrix\n";
//            break;
            return {{}};
        }
        // 获取列主元素
        temp = t[i][i];
        // 将主元所在的行进行单位化处理
        //cout<<"\nthe temp is "<<temp<<endl;
        for (j = 0; j < 3; j++)
        {
            t[i][j] = t[i][j] / temp;
            b[i][j] = b[i][j] / temp;
        }
        for (j = 0; j < 3; j++)
        {
            if (j != i)
            {
                temp = t[j][i];
                //消去该列的其他元素
                for (k = 0; k < 3; k++)
                {
                    t[j][k] = t[j][k] - temp * t[i][k];
                    b[j][k] = b[j][k] - temp * b[i][k];
                }
            }

        }

    }

    return b;
}
