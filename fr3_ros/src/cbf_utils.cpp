#include <fr3_ros/cbf_utils.h>

namespace fr3_ros { 

std::tuple<double, Eigen::Matrix<double, 1, 14>> end_effector_box_cbf(const Eigen::Matrix<double, 7, 1>& q,
                                                                      const Eigen::Matrix<double, 7, 1>& dq,
                                                                      Eigen::Matrix<double, 6, 7>& jacobian,
                                                                      Eigen::Matrix<double, 6, 7>& djacobian,
                                                                      const Eigen::Matrix<double, 3, 1>& p_measured,
                                                                      Eigen::Matrix<double, 3, 1>& normalVec,
                                                                      const double& d_max,
                                                                      const double& alpha) {
    // get positional Jacobians
    Eigen::Matrix<double, 3, 7> Jp = jacobian.topLeftCorner(3, 7);
    Eigen::Matrix<double, 3, 7> dJp = djacobian.topLeftCorner(3, 7);

    // compute control barrier function
    double cbf = -normalVec.transpose() * Jp * dq + alpha * (d_max - normalVec.transpose() * p_measured);
    
    // compute ∂h/∂x
    Eigen::Matrix<double, 1, 14> dcbf_dq;
    dcbf_dq << -normalVec.transpose() * (dJp + alpha * Jp), -normalVec.transpose() * Jp;

    return {cbf, dcbf_dq};
}

}  // namespace fr3_ros