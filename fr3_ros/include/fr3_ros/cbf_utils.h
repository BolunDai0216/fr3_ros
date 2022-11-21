#pragma once

#include <tuple>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace fr3_ros {

/**
 * @brief This function computes h and ∂h/∂x for a end-effector box CBF.      
 *
 * @param q          : measured joint angles. 
 * @param dq         : measured joint velocities.
 * @param jacobian   : end-effector Jacobian.
 * @param djacobian  : time derivate of end-effector Jacobian.
 * @param p_measured : measured end-effector position.
 * @param normalVec  : normal vector of the obstacle plane.
 * @param d_max      : max distance to the obstacle plane measured in the base frame.
 * @param alpha      : relative degree one CBF constaint gain.
 */
std::tuple<double, Eigen::Matrix<double, 1, 14>> end_effector_box_cbf(const Eigen::Matrix<double, 7, 1>& q,
                                                                      const Eigen::Matrix<double, 7, 1>& dq,
                                                                      Eigen::Matrix<double, 6, 7>& jacobian,
                                                                      Eigen::Matrix<double, 6, 7>& djacobian,
                                                                      const Eigen::Matrix<double, 3, 1>& p_measured,
                                                                      Eigen::Matrix<double, 3, 1>& normalVec,
                                                                      const double& d_max,
                                                                      const double& alpha);

}  // namespace fr3_ros