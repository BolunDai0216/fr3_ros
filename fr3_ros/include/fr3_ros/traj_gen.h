#pragma once

#include <tuple>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace fr3_ros { 

void computePoly(const Eigen::Matrix<double, 3, 1>& p_start,
                 const Eigen::Matrix<double, 3, 1>& p_end,
                 const double& T,
                 Eigen::Matrix<double, 6, 1>& poly_x,
                 Eigen::Matrix<double, 6, 1>& poly_y,
                 Eigen::Matrix<double, 6, 1>& poly_z);

void computePolyTargets(const double& t,
                        const double& T,
                        const Eigen::Matrix<double, 6, 1>& poly_x,
                        const Eigen::Matrix<double, 6, 1>& poly_y,
                        const Eigen::Matrix<double, 6, 1>& poly_z,
                        Eigen::Matrix<double, 3, 1>& p_target,
                        Eigen::Matrix<double, 3, 1>& v_target,
                        Eigen::Matrix<double, 3, 1>& a_target);

}  // namespace fr3_ros