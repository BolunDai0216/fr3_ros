#include <cmath>
#include <iostream>

#include <fr3_ros/traj_gen.h>

namespace fr3_ros { 

void computePoly(const Eigen::Matrix<double, 3, 1>& p_start,
                 const Eigen::Matrix<double, 3, 1>& p_end,
                 const double& T,
                 Eigen::Matrix<double, 6, 1>& poly_x,
                 Eigen::Matrix<double, 6, 1>& poly_y,
                 Eigen::Matrix<double, 6, 1>& poly_z) {
    // define linear equations
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> bx, by, bz;

    A << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
         std::pow(T, 5), std::pow(T, 4), std::pow(T, 3), std::pow(T, 2), T, 1.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         5*std::pow(T, 4), 4*std::pow(T, 3), 3*std::pow(T, 2), 2*T, 1.0, 0.0,
         0.0, 0.0, 0.0, 2.0, 0.0, 0.0,
         20*std::pow(T, 3), 12*std::pow(T, 2), 6*T, 2.0, 0.0, 0.0;

    bx << p_start(0, 0), p_end(0, 0), 0.0, 0.0, 0.0, 0.0;
    by << p_start(1, 0), p_end(1, 0), 0.0, 0.0, 0.0, 0.0;
    bz << p_start(2, 0), p_end(2, 0), 0.0, 0.0, 0.0, 0.0;

    // solve linear equations
    poly_x = A.colPivHouseholderQr().solve(bx);
    poly_y = A.colPivHouseholderQr().solve(by);
    poly_z = A.colPivHouseholderQr().solve(bz);
}

void computePolyJerk(const Eigen::Matrix<double, 3, 1>& p_start,
                     const Eigen::Matrix<double, 3, 1>& p_end,
                     const double& T,
                     Eigen::Matrix<double, 8, 1>& poly_x,
                     Eigen::Matrix<double, 8, 1>& poly_y,
                     Eigen::Matrix<double, 8, 1>& poly_z) {
    // define linear equations
    Eigen::Matrix<double, 8, 8> A;
    Eigen::Matrix<double, 8, 1> bx, by, bz;

    A << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
         std::pow(T, 7), std::pow(T, 6), std::pow(T, 5), std::pow(T, 4), std::pow(T, 3), std::pow(T, 2), T, 1.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         7*std::pow(T, 6), 6*std::pow(T, 5), 5*std::pow(T, 4), 4*std::pow(T, 3), 3*std::pow(T, 2), 2*T, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0,
         42*std::pow(T, 5), 30*std::pow(T, 4), 20*std::pow(T, 3), 12*std::pow(T, 2), 6*T, 2.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 0.0, 0.0,
         210*std::pow(T, 4), 120*std::pow(T, 3), 60*std::pow(T, 2), 24*T, 6.0, 0.0, 0.0, 0.0;

    bx << p_start(0, 0), p_end(0, 0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    by << p_start(1, 0), p_end(1, 0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    bz << p_start(2, 0), p_end(2, 0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // solve linear equations
    poly_x = A.colPivHouseholderQr().solve(bx);
    poly_y = A.colPivHouseholderQr().solve(by);
    poly_z = A.colPivHouseholderQr().solve(bz);
}

void computePolyTargets(const double& t,
                        const double& T,
                        const Eigen::Matrix<double, 6, 1>& poly_x,
                        const Eigen::Matrix<double, 6, 1>& poly_y,
                        const Eigen::Matrix<double, 6, 1>& poly_z,
                        Eigen::Matrix<double, 3, 1>& p_target,
                        Eigen::Matrix<double, 3, 1>& v_target,
                        Eigen::Matrix<double, 3, 1>& a_target) {
    Eigen::Matrix<double, 1, 6> p_mat, v_mat, a_mat;
    double _t = t <= T ? t : T;  // tenary operator

    p_mat << std::pow(_t, 5), std::pow(_t, 4), std::pow(_t, 3), std::pow(_t, 2), _t, 1.0;
    v_mat << 5*std::pow(_t, 4), 4*std::pow(_t, 3), 3*std::pow(_t, 2), 2*_t, 1.0, 0.0;
    a_mat << 20*std::pow(_t, 3), 12*std::pow(_t, 2), 6*_t, 2.0, 0.0, 0.0;

    p_target << p_mat * poly_x, p_mat * poly_y, p_mat * poly_z;
    v_target << v_mat * poly_x, v_mat * poly_y, v_mat * poly_z;
    a_target << a_mat * poly_x, a_mat * poly_y, a_mat * poly_z;
}

void computePolyJerkTargets(const double& t,
                            const double& T,
                            const Eigen::Matrix<double, 8, 1>& poly_x,
                            const Eigen::Matrix<double, 8, 1>& poly_y,
                            const Eigen::Matrix<double, 8, 1>& poly_z,
                            Eigen::Matrix<double, 3, 1>& p_target,
                            Eigen::Matrix<double, 3, 1>& v_target,
                            Eigen::Matrix<double, 3, 1>& a_target) {
    Eigen::Matrix<double, 1, 8> p_mat, v_mat, a_mat;
    double _t = t <= T ? t : T;  // tenary operator

    p_mat << std::pow(_t, 7), std::pow(_t, 6), std::pow(_t, 5), std::pow(_t, 4), std::pow(_t, 3), std::pow(_t, 2), _t, 1.0;
    v_mat << 7*std::pow(_t, 6), 6*std::pow(_t, 5), 5*std::pow(_t, 4), 4*std::pow(_t, 3), 3*std::pow(_t, 2), 2*_t, 1.0, 0.0;
    a_mat << 42*std::pow(_t, 5), 30*std::pow(_t, 4), 20*std::pow(_t, 3), 12*std::pow(_t, 2), 6*_t, 2.0, 0.0, 0.0;

    p_target << p_mat * poly_x, p_mat * poly_y, p_mat * poly_z;
    v_target << v_mat * poly_x, v_mat * poly_y, v_mat * poly_z;
    a_target << a_mat * poly_x, a_mat * poly_y, a_mat * poly_z;
}

}  // namespace fr3_ros