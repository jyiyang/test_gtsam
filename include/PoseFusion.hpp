#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <Eigen/Core>

class PoseFusion {
public:
    PoseFusion();
    ~PoseFusion();

    void addMVGPose(const Eigen::Matrix4f& T, const int key);
    void addEFPose(const Eigen::Matrix4f& T, const int key1, const int key2);

    void solveFactorGraph();

private:
    int numPoses_;
    gtsam::NonlinearFactorGraph factorGraph_;
    gtsam::Values initialEstimate_;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise_;
    gtsam::Values results_;
};
