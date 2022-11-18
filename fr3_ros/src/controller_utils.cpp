#include <fr3_ros/controller_utils.h>

namespace fr3_ros { 

ros::Publisher registerLogPublisher(ros::NodeHandle& node_handle) {
  return node_handle.advertise<fr3_ros::controlLogs>("control_logs", 1);
}

void publishLogMsgs(LogDataType *data, ros::Publisher *pub) {  
  fr3_ros::controlLogs log_msg;
  
  for(int i=0; i<7; i++)
    log_msg.q.push_back(data->q[i]);

  for(int i=0; i<7; i++)
    log_msg.q_dot.push_back(data->q_dot[i]);

  for(int i=0; i<7; i++)
    log_msg.q_des.push_back(data->q_des[i]);

  for(int i=0; i<7; i++)
    log_msg.q_dot_des.push_back(data->q_dot_des[i]);

  for(int i=0; i < data->M.size(); i++)
    log_msg.M.push_back(data->M.data()[i]);

  for(int i=0; i < data->M_aux.size(); i++)
    log_msg.M_aux.push_back(data->M_aux.data()[i]);

  for(int i=0; i < data->C.size(); i++)
    log_msg.C.push_back(data->C.data()[i]);

  for(int i=0; i < data->G.size(); i++)
    log_msg.G.push_back(data->G.data()[i]);

  for(int i=0; i < data->J.size(); i++)
    log_msg.J.push_back(data->J.data()[i]);

  for(int i=0; i < data->J_dot.size(); i++)
    log_msg.J_dot.push_back(data->J_dot.data()[i]);

  for(int i=0; i < data->J_aux.size(); i++)
    log_msg.J_aux.push_back(data->J_aux.data()[i]);

  for(int i=0; i < data->torque_cmd.size(); i++)
    log_msg.torque_cmd.push_back(data->torque_cmd.data()[i]);

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

}