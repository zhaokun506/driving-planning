#include "reference_line/reference_line_smoother.h"
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Eigen"

//使用：将类的初始化参数 传递给 成员变量
ReferenceLineSmoother::ReferenceLineSmoother(
    const ReferenceLineSmootherConfig &config)
    : config_(config) {}

void ReferenceLineSmoother::Smooth(const ReferenceLine &raw_reference_line,
                                   ReferenceLine &smoothed_reference_line) {
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<std::pair<double, double>> smoothed_point2d;
  for (const auto point : raw_reference_line.reference_points()) {
    raw_point2d.emplace_back(std::make_pair(point.x, point.y));
  }
  DiscretePointsSmooth(raw_point2d, &smoothed_point2d);
  std::vector<ReferencePoint> smoothed_points;
  for (const auto pt : smoothed_point2d) {
    ReferencePoint rp;
    rp.x = pt.first;
    rp.y = pt.second;
    smoothed_points.push_back(rp);
  }
  smoothed_reference_line.set_reference_points(smoothed_points);
}

//
bool ReferenceLineSmoother::DiscretePointsSmooth(
    const std::vector<std::pair<double, double>> &raw_point2d,
    std::vector<std::pair<double, double>> *smoothed_point2d) {
  int n = raw_point2d.size();

  //初始化A1,A2,A3，f,lb,ub矩阵
  //平滑代价系数矩阵，x'A1'A1x, (n-2)
  Eigen::SparseMatrix<double> A1(2 * n, 2 * n);
  //路径长度代价矩阵 x'A2'A2x,(n-1)
  Eigen::SparseMatrix<double> A2(2 * n, 2 * n);
  //参考线偏离代价矩阵 x'A3'A3x,单位阵
  Eigen::SparseMatrix<double> A3(2 * n, 2 * n);

  Eigen::SparseMatrix<double> H(2 * n, 2 * n); //必须是稀疏矩阵
  Eigen::VectorXd f = Eigen::VectorXd::Zero(2 * n);
  Eigen::SparseMatrix<double> A(2 * n, 2 * n);
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(2 * n);
  Eigen::VectorXd ub = Eigen::VectorXd::Zero(2 * n);
  Eigen::VectorXd qp_solution = Eigen::VectorXd::Zero(2 * n);

  A.setIdentity();

  //赋值f,lb,ub;
  // MatrixXd下标从(0,0)开始,(1,2)表示第1行第2列
  for (int i = 0; i < n; i++) {
    f(2 * i) = raw_point2d[i].first;
    f(2 * i + 1) = raw_point2d[i].second;

    lb(2 * i) = f(2 * i) + config_.x_lower_bound;
    lb(2 * i + 1) = f(2 * i + 1) + config_.y_lower_bound;

    ub(2 * i) = f(2 * i) + config_.x_upper_bound;
    ub(2 * i + 1) = f(2 * i + 1) + config_.y_upper_bound;
  }

  //赋值A1
  for (int j = 0; j < n - 2; j++) {
    A1.insert(2 * j, 2 * j) = 1;
    A1.insert(2 * j, 2 * j + 2) = -2;
    A1.insert(2 * j, 2 * j + 4) = 1;
    A1.insert(2 * j + 1, 2 * j + 1) = 1;
    A1.insert(2 * j + 1, 2 * j + 3) = -2;
    A1.insert(2 * j + 1, 2 * j + 5) = 1;
  }
  //赋值A2
  for (int k = 0; k < n - 1; k++) {
    A2.insert(2 * k, 2 * k) = 1;
    A2.insert(2 * k, 2 * k + 2) = -1;
    A2.insert(2 * k + 1, 2 * k + 1) = 1;
    A2.insert(2 * k + 1, 2 * k + 3) = 1;
  }

  A3.setIdentity();

  // H = 2 * (config_.weight_smooth * (A1.transpose().dot(A1)) +
  //          config_.weight_path_length * (A2.transpose().dot(A2)) +
  //          config_.weight_ref_deviation * A3);
  H = 2 * (config_.weight_smooth * A1.transpose() * A1 +
           config_.weight_path_length * A2.transpose() * A2 +
           config_.weight_ref_deviation * A3);

  f = -2 * config_.weight_ref_deviation * f;

  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(2 * n);
  solver.data()->setNumberOfConstraints(2 * n);
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(f);
  solver.data()->setLinearConstraintsMatrix(A);
  solver.data()->setLowerBound(lb);
  solver.data()->setUpperBound(ub);

  if (!solver.initSolver())
    return 0;
  if (!solver.solve())
    return 0;
  qp_solution = solver.getSolution();

  (*smoothed_point2d).resize(n);
  for (int i = 0; i < n; i++) {
    (*smoothed_point2d)[i].first = qp_solution(2 * i);
    (*smoothed_point2d)[i].second = qp_solution(2 * i + 1);
  }
}

/*
    使用二次规划
    0.5x'Hx + f'x = min
    lb < Ax < ub
    % 二次规划形式

% H1 = w_cost_smooth*(A1'*A1) + w_cost_length*(A2'*A2) + w_cost_ref*(A3'*A3)
% f = -2 * w_cost_ref * referenceline_init
% x'H1x + f'x = 0.5 * x'(2H1)*x + f'x
% A1 = [1  0 -2  0  1  0
%       0  1  0 -2  0  1
%             1  0 -2  0  1  0
%             0  1  0 -2  0  1
%                   ...............

%A2 = [1  0 -1  0
%      0  1  0 -1
%            1  0 -1  0
%            0  1  0 -1
%                  ...........
%A3 为单位矩阵


f'=w_cost_ref*h'
*/
