#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace pin = pinocchio;

namespace fr3_ros {

/**
 * @brief This function computes M(q), M^{-1}(q), C(q, dq) + G(q), and G(q) using measurements from FR3. 
 *        The computed M(q), M^{-1}(q), C(q, dq) + G(q), and G(q) are stored in data.M, data.Minv, data.nle, data.g.         
 *
 * @param model : Pinocchio robot model.
 * @param data  : Pinocchio robot data.
 * @param q     : measured joint angles. 
 * @param dq    : measured joint velocities.
 */
inline void getDynamicsParameter(pinocchio::Model& model,
                                 pinocchio::Data& data,
                                 const Eigen::Matrix<double, 7, 1>& q, 
                                 const Eigen::Matrix<double, 7, 1>& dq) {
  // compute the upper triangle portion of the mass matrix
  pin::crba(model, data, q);

  // fill the lower triangle portion of the mass matrix
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  // compute C + G
  pin::nonLinearEffects(model, data, q, dq);

  // compute G
  pin::computeGeneralizedGravity(model, data, q);

  // compute M^{-1}
  pin::computeMinverse(model, data, q);
}

/**
 * @brief This function updates the Pinocchio model using measurements from FR3.
 *
 * @param model : Pinocchio robot model.
 * @param data  : Pinocchio robot data.
 * @param q     : measured joint angles. 
 * @param dq    : measured joint velocities.
 */ 
inline void updatePinocchioModel(pinocchio::Model& model,
                                 pinocchio::Data& data,
                                 const Eigen::Matrix<double, 7, 1>& q, 
                                 const Eigen::Matrix<double, 7, 1>& dq) {
  // update pinocchio robot model
  pin::forwardKinematics(model, data, q, dq);
  pin::computeJointJacobians(model, data, q); 
  pin::updateFramePlacements(model, data);   
  pin::computeJointJacobiansTimeVariation(model, data, q, dq);                           
}

}  // namespace fr3_ros