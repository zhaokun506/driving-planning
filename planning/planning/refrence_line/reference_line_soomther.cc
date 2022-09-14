#include "reference_line_soomther.h"
#include "eigen3/Eigen/Dense"
#include "osqpEigen/OsqpEigen.h"
//使用：将类的初始化参数 传递给 成员变量
ReferenceLineSmoother::ReferenceLineSmoother(const ReferenceLineSmootherConfig &config) : config_(config)
{
}

void ReferenceLineSmoother::Smooth(const ReferenceLine &raw_reference_line, ReferenceLine *const smoothed_reference_line)
{
}

//
void ReferenceLineSmoother::DiscretePointsSmooth(const std::vector<std::pair<double, double>> &raw_point2d,
                                                 const std::vector<double> &bounds,
                                                 std::vector<std::pair<double, double>> *ptr_smoothed_point2d)
{
    int n = raw_point2d.size();

    //初始化A1,A2,A3，f,lb,ub矩阵
    Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(2 * n - 4, 2 * n); //平滑代价系数矩阵，x'A1'A1x
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(2 * n - 2, 2 * n); //路径长度代价矩阵 x'A2'A2x
    Eigen::MatrixXd A3 = Eigen::MatrixXd::Identity(2 * n, 2 * n); //参考线偏离代价矩阵 x'A3'A3x,单位阵

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2 * n, 2 * n);
    Eigen::MatrixXd f = Eigen::MatrixXd::Zero(2 * n, 1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd lb = Eigen::MatrixXd::Zero(2 * n, 1);
    Eigen::MatrixXd ub = Eigen::MatrixXd::Zero(2 * n, 1);

    Eigen::VectorXd qp_solution = Eigen::MatrixXd::Zero(2 * n);
    //赋值f,lb,ub;
    // MatrixXd下标从(0,0)开始,(1,2)表示第1行第2列
    for (int i = 0; i < n; i++)
    {
        f(2 * i) = raw_point2d[i].first;
        f(2 * i + 1) = raw_point2d[i].second;

        lb(2 * i) = f(2 * i + 1) + config_.x_lower_bound;
        lb(2 * i + 1) = f(2 * i + 1) + config_.y_lower_bound;

        ub(2 * i) = f(2 * i + 1) + config_.x_upper_bound;
        ub(2 * i + 1) = f(2 * i + 1) + config_.y_upper_bound;
    }
    //赋值A2
    for (int j = 0; j < n - 2; j++)
    {
        A2(2 * j, 2 * j) = 1;
        A2(2 * j, 2 * j + 2) = -2;
        A2(2 * j, 2 * j + 4) = 1;
        A2(2 * j + 1, 2 * j + 1) = 1;
        A2(2 * j + 1, 2 * j + 3) = -2;
        A2(2 * j + 1, 2 * j + 5) = 1;
    }
    //赋值A3
    for (int k = 0; k < n - 1; k++)
    {
        A3(2 * k, 2 * k) = 1;
        A3(2 * k, 2 * k + 2) = -1;
        A3(2 * k + 1, 2 * k + 1) = 1;
        A3(2 * k + 1, 2 * k + 3) = 1;
    }

    H = 2 * (config_.weight_smooth * (A1.transpose * A1) + config_.weight_path_length * (A2.transpose * A2) + config_.weight_ref_deviation * A3);

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(2 * n);
    solver.data()->setNumberOfConstraints(1);
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(f);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);

    if (!solver.initSolve())
        return 0;
    if (!solver.solve())
        return 0;
    qp_solution = solver.getSolution();

    for (int i = 0; i < n; i++)
    {
        ptr_smoothed_point2d[i]->first = qp_solution(2 * i);
        ptr_smoothed_point2d[i]->second = qp_solution(2 * i + 1);
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
