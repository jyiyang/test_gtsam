#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

#include "PoseFusion.hpp"

using namespace gtsam;
using namespace std;

PoseFusion::PoseFusion() :
factorGraph_(NonlinearFactorGraph()),
initialEstimate_()
{
    priorNoise_ = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    odometryNoise_ = noiseModel::Diagonal::Sigmas((Vector(6) << 0.05, 0.05, 0.05, 0.02, 0.02, 0.02).finished());
}

PoseFusion::~PoseFusion() {

}

void PoseFusion::addMVGPose(const Eigen::Matrix4f& T, const int key) {
    // Add priors
    Pose3 priorPose(T.cast<double>());
    factorGraph_.add(PriorFactor<Pose3>(key, priorPose, priorNoise_));
    // Use priors as initial estimate
    initialEstimate_.insert(key, priorPose);
}

void PoseFusion::addEFPose(const Eigen::Matrix4f& T, const int key1, const int key2) {
    Pose3 odometry(T.cast<double>());
    factorGraph_.add(BetweenFactor<Pose3>(key1, key2, odometry, odometryNoise_));
}

void PoseFusion::solveFactorGraph() {
    LevenbergMarquardtOptimizer optimizer(factorGraph_, initialEstimate_);
    results_ = optimizer.optimize();
}
