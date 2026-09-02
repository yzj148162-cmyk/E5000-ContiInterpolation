#include "forwardkinematicssolver.h"

#include "MatrixFun.h"
#include "optimization.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace alglib;

namespace {

constexpr std::array<double, 6> kDefaultInitialPose = {
    0.0, 0.0, 400.0, 0.0, 0.0, 0.0
};

bool hasValidPose6(const std::vector<double>& pose)
{
    if(pose.size() < 6){
        return false;
    }
    for(int i=0; i<6; ++i){
        if(!std::isfinite(pose[i])){
            return false;
        }
    }
    return true;
}

bool isExcludedCableIndex(int cableIndex, const std::vector<int>& excludedCableIndices)
{
    return std::find(excludedCableIndices.begin(),
                     excludedCableIndices.end(),
                     cableIndex) != excludedCableIndices.end();
}

std::vector<std::vector<double>> contactPointGlobal(
        const std::vector<double>& pose,
        const std::vector<std::vector<double>>& contactPointLocal)
{
    std::vector<std::vector<double>> rotx = {
        {1, 0, 0, 0},
        {0, std::cos(pose[3]), -std::sin(pose[3]), 0},
        {0, std::sin(pose[3]), std::cos(pose[3]), 0},
        {0, 0, 0, 1}
    };
    std::vector<std::vector<double>> roty = {
        {std::cos(pose[4]), 0, std::sin(pose[4]), 0},
        {0, 1, 0, 0},
        {-std::sin(pose[4]), 0, std::cos(pose[4]), 0},
        {0, 0, 0, 1}
    };
    std::vector<std::vector<double>> rotz = {
        {std::cos(pose[5]), -std::sin(pose[5]), 0, 0},
        {std::sin(pose[5]), std::cos(pose[5]), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    std::vector<std::vector<double>> txyz = {
        {1, 0, 0, pose[0]},
        {0, 1, 0, pose[1]},
        {0, 0, 1, pose[2]},
        {0, 0, 0, 1}
    };

    std::vector<std::vector<double>> transform =
            MatrixFun::matrixMulti(MatrixFun::matrixMulti(MatrixFun::matrixMulti(txyz, rotz), roty), rotx);
    std::vector<std::vector<double>> result(contactPointLocal.size());
    for(int cableIndex=0; cableIndex<static_cast<int>(contactPointLocal.size()); ++cableIndex){
        std::vector<std::vector<double>> local(4, std::vector<double>(1, 1.0));
        for(int axis=0; axis<3 && axis<static_cast<int>(contactPointLocal[cableIndex].size()); ++axis){
            local[axis][0] = contactPointLocal[cableIndex][axis];
        }
        const std::vector<std::vector<double>> global = MatrixFun::matrixMulti(transform, local);
        result[cableIndex].resize(4, 1.0);
        for(int axis=0; axis<4; ++axis){
            result[cableIndex][axis] = global[axis][0];
        }
    }
    return result;
}

struct ResidualContext {
    const ForwardKinematicsSolver::Request* request = nullptr;
};

void cableLengthResidual(const real_1d_array& x, real_1d_array& fi, void* ptr)
{
    const ResidualContext* context = static_cast<const ResidualContext*>(ptr);
    if(!context || !context->request){
        for(int i=0; i<fi.length(); ++i){
            fi[i] = 0.0;
        }
        return;
    }

    const ForwardKinematicsSolver::Request& request = *context->request;
    const std::vector<double> pose = {x[0], x[1], x[2], x[3], x[4], x[5]};
    const std::vector<std::vector<double>> globalContact =
            contactPointGlobal(pose, request.contactPointLocal);

    int fiIndex = 0;
    for(int cableIndex=0;
        cableIndex<static_cast<int>(request.cableLength.size()) && fiIndex<fi.length();
        ++cableIndex){
        if(isExcludedCableIndex(cableIndex, request.excludedCableIndices)){
            continue;
        }
        if(cableIndex >= static_cast<int>(globalContact.size()) ||
                cableIndex >= static_cast<int>(request.anchorPos.size())){
            continue;
        }
        const double theoreticalLength =
                MatrixFun::cableLengthCalculate(globalContact[cableIndex],
                                                request.anchorPos[cableIndex],
                                                request.pulleyRadius).idealLength;
        fi[fiIndex] = request.cableLength[cableIndex] - theoreticalLength;
        ++fiIndex;
    }
    for(; fiIndex<fi.length(); ++fiIndex){
        fi[fiIndex] = 0.0;
    }
}

real_1d_array makeInitialArray(const std::vector<double>& requestInitial,
                               const std::vector<double>& lastPose)
{
    std::array<double, 6> initialPose = kDefaultInitialPose;
    const std::vector<double>& source = hasValidPose6(requestInitial) ? requestInitial : lastPose;
    if(hasValidPose6(source)){
        for(int i=0; i<6; ++i){
            initialPose[i] = source[i];
        }
    }

    real_1d_array x;
    x.setcontent(6, initialPose.data());
    return x;
}

real_1d_array makeBoundsArray(const std::vector<double>& requestBounds,
                              const std::array<double, 6>& defaultBounds)
{
    std::array<double, 6> bounds = defaultBounds;
    if(requestBounds.size() >= bounds.size()){
        for(int i=0; i<static_cast<int>(bounds.size()); ++i){
            if(std::isfinite(requestBounds[i])){
                bounds[i] = requestBounds[i];
            }
        }
    }

    real_1d_array result;
    result.setcontent(6, bounds.data());
    return result;
}

} // namespace

ForwardKinematicsSolver::Result ForwardKinematicsSolver::solve(const Request& request)
{
    Result result;
    const int cableCount = std::min({static_cast<int>(request.anchorPos.size()),
                                     static_cast<int>(request.contactPointLocal.size()),
                                     static_cast<int>(request.cableLength.size())});
    if(cableCount <= 0){
        return result;
    }

    std::vector<int> excludedCableIndices = request.excludedCableIndices;
    for(int cableIndex=0; cableIndex<cableCount; ++cableIndex){
        if(!isExcludedCableIndex(cableIndex, excludedCableIndices)){
            ++result.equationCount;
        }
    }
    if(!excludedCableIndices.empty() && result.equationCount != 6){
        excludedCableIndices.clear();
        result.equationCount = cableCount;
    }
    if(result.equationCount < 6){
        return result;
    }

    real_1d_array x = makeInitialArray(request.initialPose, lastPose);
    real_1d_array bndl = makeBoundsArray(
                request.poseLowerBounds,
                std::array<double, 6>{-3100.0, -3100.0, -40.0,
                                      -3.14, -3.14, -3.14});
    real_1d_array bndu = makeBoundsArray(
                request.poseUpperBounds,
                std::array<double, 6>{3100.0, 3100.0, 3100.0,
                                      3.14, 3.14, 3.14});
    minlmstate state;
    minlmreport report;
    minlmcreatev(result.equationCount, x, 0.0001, state);
    minlmsetbc(state, bndl, bndu);
    minlmsetcond(state, 0.000001, 10);

    Request effectiveRequest = request;
    effectiveRequest.excludedCableIndices = excludedCableIndices;
    ResidualContext context;
    context.request = &effectiveRequest;
    minlmoptimize(state, cableLengthResidual, nullptr, &context);
    minlmresults(state, x, report);

    result.pose.resize(6);
    for(int i=0; i<6; ++i){
        result.pose[i] = x[i];
    }
    if(!request.keepRotation){
        for(int i=3; i<6; ++i){
            result.pose[i] = 0.0;
        }
    }
    result.success = hasValidPose6(result.pose);
    if(result.success){
        lastPose = result.pose;
    }
    return result;
}

void ForwardKinematicsSolver::setInitialPose(const std::vector<double>& pose)
{
    if(hasValidPose6(pose)){
        lastPose.assign(pose.begin(), pose.begin() + 6);
    }
}

std::vector<double> ForwardKinematicsSolver::initialPose() const
{
    return lastPose;
}
