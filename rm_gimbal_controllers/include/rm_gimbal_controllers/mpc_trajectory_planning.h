//
// Created by xuncheng on 2026/4/11.
//

#ifndef RM_WS_MPC_TRAJERTORY_PLANNING_H
#define RM_WS_MPC_TRAJERTORY_PLANNING_H
#include <osqp.h>
#include <Eigen/Sparse>

namespace rm_gimbal_controllers
{
class MPC_Trajectory_Planning
{
public:
  // Eigen -> OSQP 转换函数
  OSQPCscMatrix* eigen2osqp(const Eigen::SparseMatrix<double>& mat)
  {
    Eigen::SparseMatrix<double> mat_csc = mat;
    mat_csc.makeCompressed();

    OSQPInt m = mat_csc.rows();
    OSQPInt n = mat_csc.cols();
    OSQPInt nz = mat_csc.nonZeros();

    std::vector<OSQPInt> inner(mat_csc.innerIndexPtr(), mat_csc.innerIndexPtr() + nz);
    std::vector<OSQPInt> outer(mat_csc.outerIndexPtr(), mat_csc.outerIndexPtr() + n + 1);

    OSQPCscMatrix* osqp_mat = OSQPCscMatrix_new(m, n, nz, mat_csc.valuePtr(), inner.data(), outer.data());

    return osqp_mat;
  }

  void initMatrix(Eigen::Vector2d x_self)
  {
    A << 1.0, 0.001, 0.0, 1.0;
    B << 0.0, 0.001;
    f << 0.0, 0.0;
    Q << 10e6, 0.0, 0.0, 0.0;
    R << 1.0;
    I << 1.0, 0.0, 0.0, 1.0;
    for (int i = 0; i < (N + 1); i++)
    {
      P_des.block(nx * i, nx * i, nx, nx) = Q;
      P_des.block(nu * i + nx * (N + 1), nu * i + nx * N, nu, nu) = R;
    }
    //转化成稀疏矩阵
    P_osqp = P_des.sparseView();
    P_csc = eigen2osqp(P_osqp);

    //初始状态不可以更改
    A_(0, 0) = 1;
    A_(1, 1) = 1;
    l_(0) = x_self(0);
    u_(0) = x_self(0);
    l_(1) = x_self(1);
    u_(1) = x_self(1);
    x = x_self;
    for (int i = 0; i < N; i++)
    {
      //动力学解算
      A_.block(nx * i + 2, nx * i, nx, nx) = -A;
      A_.block(nx * i + 2, nx * (N + 1) + nu * i, nx, nu) = -B;
      A_.block(nx * i + 2, nx * (i + 1), 2, 2) = I;
      //速度限制
      A_(nx * N + 2 + i, nx * (i + 1) + 1) = 1;
      l_(nx * N + 2 + i) = -max_vel_;
      u_(nx * N + 2 + i) = max_vel_;
      //加速度限制
      A_(nx * N + 2 + N + i, nx * (N + 1) + nu * i) = 1;
      l_(nx * N + 2 + N + i) = -max_acc_;
      u_(nx * N + 2 + N + i) = max_acc_;
    }
    A_osqp = A_.sparseView();
    A_csc = eigen2osqp(A_osqp);
  }
  void osqpSolver(Eigen::VectorXd bullet_pos_des)
  {
    for (int i = 0; i < N + 1; i++)
    {
      q_osqp(nx * i) = -Q(0, 0) * bullet_pos_des(i);
    }
    OSQPSettings* settings = OSQPSettings_new();
    osqp_set_default_settings(settings);

    OSQPSolver* solver = NULL;
    osqp_setup(&solver, P_csc, q_osqp.data(), A_csc, l_.data(), u_.data(), N * 4 + 2, (nx + nu) * (N + 1), settings);

    osqp_solve(solver);

    OSQPSolution sol = OSQPSolution();
    osqp_get_solution(solver, &sol);
    pos_des_ = sol.x[3];
    vel_des_ = sol.x[4];
    acc_des_ = sol.x[2];
    OSQPSettings_free(settings);
    osqp_cleanup(solver);
  };
  double getPosDes()
  {
    return pos_des_;
  }
  double getVelDes()
  {
    return vel_des_;
  }
  double getAccDes()
  {
    return acc_des_;
  }

private:
  //自身状态和控制量
  Eigen::Vector2d x;
  Eigen::VectorXd u;
  //状态变化矩阵
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  //常数扰动
  Eigen::Vector2d f;
  //权重矩阵
  Eigen::Matrix2d Q;
  Eigen::MatrixXd R;
  Eigen::Matrix2d I;
  double pos_des_ = 0.0;
  double vel_des_ = 0.0;
  double acc_des_ = 0.0;
  double max_acc_ = 40;
  double max_vel_ = 8;
  int N = 200;
  int nx = 2;
  int nu = 1;
  int num = (nx + nu) * (N + 1);
  Eigen::MatrixXd P_des = Eigen::MatrixXd::Zero(num, num);
  Eigen::VectorXd q_osqp = Eigen::VectorXd::Zero(num);
  Eigen::SparseMatrix<double> P_osqp;
  Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(N * 4 + 2, num);
  Eigen::VectorXd l_ = Eigen::VectorXd::Zero(N * 4 + 2);
  Eigen::VectorXd u_ = Eigen::VectorXd::Zero(N * 4 + 2);
  Eigen::SparseMatrix<double> A_osqp;
  const OSQPCscMatrix* A_csc;
  const OSQPCscMatrix* P_csc;
};
}  // namespace rm_gimbal_controllers
#endif  // RM_WS_MPC_TRAJERTORY_PLANNING_H
