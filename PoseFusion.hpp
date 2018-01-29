#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <Eigen/Core>

class PoseFusion {
public:
    PoseFusion();
    ~PoseFusion();

private:
    int numPoses_;
    NonlinearFactorGraph factorGraph_;
    Eigen::Matrix4f relativeTrans_;
}
