#pragma once

#include <algorithm>
#include <vector>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "fr3_ros/controlLogs.h"

namespace fr3_ros {

struct LogDataType{
  Eigen::Matrix<double, 7, 1>  q;
  Eigen::Matrix<double, 7, 1> q_dot;
  Eigen::Matrix<double, 7, 1> q_des;
  Eigen::Matrix<double, 7, 1> q_dot_des;
  Eigen::Matrix<double, 7, 7> M;
  Eigen::Matrix<double, 7, 7> C;
  Eigen::Matrix<double, 7, 1> G;
  Eigen::Matrix<double, 6, 7> J;
  Eigen::Matrix<double, 6, 7> J_dot;
  Eigen::Matrix<double, 6, 7> J_aux;
  Eigen::Matrix<double, 7, 7> M_aux;
  Eigen::Matrix<double, 7, 1> torque_cmd;
};

ros::Publisher registerLogPublisher(ros::NodeHandle& node_handle);

void publishLogMsgs(LogDataType *data, ros::Publisher *pub);

/**
 * @brief This function saturates the torque to comply with the maximum torque change rate
 *
 * @param tau_d_calculated : commanded torque without gravity.
 * @param tau_J_d          : desired link-side joint torque sensor signals without gravity.
 * @return tau_d_saturated : saturated torque without gravity.
 */ 
Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                               const Eigen::Matrix<double, 7, 1>& tau_J_d);

/**
 * @brief This function computes the axis-angle representation of the error rotation matrix
 *
 * @param R_target    : target orientation represented as a rotation matrix.
 * @param R_measured  : measured orientation represented as a rotation matrix.
 * @return rotvec_err : angle-axis representation of R_target @ R_measured.T
 */                                
Eigen::Vector3d computeRotVecError(const Eigen::Matrix<double, 3, 3>& R_target,
                                   const Eigen::Matrix<double, 3, 3>& R_measured);

/**
 * @brief This function computes α, dα, ddα for smooth trajectory generation
 *
 * @param t        : clock time in current trajectory segement.
 * @param T        : entire duration planned for current trajectory segement.
 * @return alpha   : sin(β) or 1 if t > T.
 * @return dalpha  : (π²/4T)cos(β)sin(πt/T) or 0 if t > T.
 * @return ddalpha : (π³/4T)cos(πt/T)cos(β) - (π⁴/16T²)sin²(πt/T)sin(β) or 0 if t > T.
 */                          
std::tuple<double, double, double> getAlphas(const double& t, const double& T); 

/**
 * @brief This function reads the joint space PD gain from rosparam
 *
 * @param k_gains_ : P gains read from rosparam.
 * @param d_gains_ : D gains read from rosparam.
 * @param Kp       : 7x7 diagonal matrix where the P gains are wrote into. 
 * @param Kd       : 7x7 diagonal matrix where the D gains are wrote into.
 */
inline void readJointPDGains(std::vector<double>& k_gains_,
                             std::vector<double>& d_gains_,
                             Eigen::Matrix<double, 7, 7>& Kp,
                             Eigen::Matrix<double, 7, 7>& Kd) {
  Eigen::Map<Eigen::Matrix<double, 7, 1>> k_gains_array(k_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> d_gains_array(d_gains_.data());

  Kp = k_gains_array.array().matrix().asDiagonal();
  Kd = d_gains_array.array().matrix().asDiagonal();
}

/**
 * @brief This function reads the task space PD gain from rosparam
 *
 * @param k_gains_ : P gains read from rosparam.
 * @param d_gains_ : D gains read from rosparam.
 * @param Kp       : 6x6 diagonal matrix where the P gains are wrote into. 
 * @param Kd       : 6x6 diagonal matrix where the D gains are wrote into.
 */
inline void readTaskPDGains(std::vector<double>& k_gains_,
                            std::vector<double>& d_gains_,
                            Eigen::Matrix<double, 6, 6>& Kp,
                            Eigen::Matrix<double, 6, 6>& Kd) {
  Eigen::Map<Eigen::Matrix<double, 6, 1>> k_gains_array(k_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> d_gains_array(d_gains_.data());

  Kp = k_gains_array.array().matrix().asDiagonal();
  Kd = d_gains_array.array().matrix().asDiagonal();
}

}  // namespace fr3_ros