#include <cmath>

#include <fr3_ros/controller_utils.h>

namespace fr3_ros { 

ros::Publisher registerLogPublisher(ros::NodeHandle& node_handle) {
  return node_handle.advertise<fr3_ros::controlLogs>("control_logs", 1);
}

void publishLogMsgs(LogDataType *data, ros::Publisher *pub) {  
  fr3_ros::controlLogs log_msg;
  
  for(int i=0; i<7; i++) {
    log_msg.q.push_back(data->q[i]);
    log_msg.q_dot.push_back(data->q_dot[i]);
    log_msg.q_des.push_back(data->q_des[i]);
    log_msg.q_dot_des.push_back(data->q_dot_des[i]);
  } 

  for(int i=0; i < data->M.size(); i++) {
    log_msg.M.push_back(data->M.data()[i]);
    log_msg.M_aux.push_back(data->M_aux.data()[i]);
  }

  for(int i=0; i < data->C.size(); i++)
    log_msg.C.push_back(data->C.data()[i]);

  for(int i=0; i < data->G.size(); i++)
    log_msg.G.push_back(data->G.data()[i]);

  for(int i=0; i < data->J.size(); i++) {
    log_msg.J.push_back(data->J.data()[i]);
    log_msg.J_dot.push_back(data->J_dot.data()[i]);
    log_msg.J_aux.push_back(data->J_aux.data()[i]);
  } 

  for(int i=0; i < data->torque_cmd.size(); i++)
    log_msg.torque_cmd.push_back(data->torque_cmd.data()[i]);
  
  for(int i=0; i<3; i++) {
    log_msg.p.push_back(data->p[i]);
    log_msg.p_des.push_back(data->p_des[i]);
  }
  
  for(int i=0; i<6; i++) {
    log_msg.P_dot.push_back(data->P_dot[i]);
    log_msg.P_dot_des.push_back(data->P_dot_des[i]);
    log_msg.P_ddot_cmd.push_back(data->P_ddot_cmd[i]);
  }

  log_msg.header.stamp = ros::Time::now();
  pub->publish(log_msg);
}

Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                               const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  const double delta_tau_max_{1.0};
  
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  
  return tau_d_saturated;
}

Eigen::Vector3d computeRotVecError(const Eigen::Matrix<double, 3, 3>& R_target,
                                   const Eigen::Matrix<double, 3, 3>& R_measured) {
  Eigen::Matrix<double, 3, 3> R_error = R_target * R_measured.transpose();
  Eigen::AngleAxisd AngleAxisErr(R_error);
  Eigen::Vector3d rotvec_err = AngleAxisErr.axis() * AngleAxisErr.angle();

  return rotvec_err;
}

std::tuple<double, double, double> getAlphas(const double& t, const double& T) {
  double alpha, dalpha, ddalpha;

  if (t <= T){
    double sin_ = std::sin(M_PI * t / T);
    double cos_ = std::cos(M_PI * t / T);
    double beta = (M_PI / 4) * (1 - cos_);
    double _sin = std::sin(beta);
    double _cos = std::cos(beta);
    double T2 = T * T;

    alpha = _sin;
    dalpha = (M_PI * M_PI / (4 * T)) * _cos * sin_;
    ddalpha = (std::pow(M_PI, 3) / (4 * T2)) * cos_ * _cos - (std::pow(M_PI, 4) / (16 * T2)) * sin_ * sin_ * _sin;
  } else {
    alpha = 1.0;
    dalpha = 0.0;
    ddalpha = 0.0;
  }

  return {alpha, dalpha, ddalpha};
}

}  // namespace fr3_ros