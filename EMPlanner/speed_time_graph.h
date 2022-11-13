//此类用于速度规划功能模块的实现

#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Eigen"
#include <algorithm>
#include <float.h>
#include <vector>

#include "EMPlanner_config.h"
#include "localization_estimate.h"
#include "perception_obstacle.h"
#include "reference_line.h"
#include "reference_line_provider.h"

#include "path_time_graph.h"

#include "trajectory.h"

class STPoint {
public:
  double t;
  double s;
  double ds_dt;
  double dds_dt;

  double cost2start;
  double pre_mincost_row;
};

class STLine {
public:
  STPoint left_point;
  STPoint right_point;
};

class SpeedTimeGraph {
public:
  SpeedTimeGraph(ReferenceLine planning_path,
                 EMPlannerConfig emplaner_conf); //传入规划好的路径
  ~SpeedTimeGraph();
  // 1.基于规划的轨迹，初始化坐标轴
  void InitSAxis(const ReferenceLine planning_path);

  //计算速度规划的
  void CalcStartCondition();
  // 2.计算障碍物的ST位置
  void SetDynamicObstaclesSL(const std::vector<ObstacleInfo> dynamic_obstacles);

  void GenerateSTGraph(const std::vector<ObstacleInfo> dynamic_obstacles);
  // 3.采样
  void CreateSmaplePoint(int row, int col);
  // 4.动态规划
  void SpeedDynamicPlanning();
  double CalcDpCost(STPoint &point_s, STPoint &point_e);
  double CalcObsCost(STPoint &point_s, STPoint &point_e);
  double CalcCollisionCost(double w_cost_obs, double min_dis);

  // 5.二次规划
  void GenerateCovexSpace();
  int FindDpMatchIndex(double t);
  void SpeedQuadraticProgramming();
  // 6.st加密
  void SpeedQpInterpolation(int interpolation_num);

  // 7.path和speed的合并
  void PathAndSpeedMerge(int n, double cur_t);

  const Trajectory trajectory() const;
  const std::vector<ReferencePoint>
  xy_virtual_obstacles() const; //虚拟障碍物的xy坐标

private:
  EMPlannerConfig emplaner_conf_;
  ReferenceLine planning_path_;

  std::vector<ReferencePoint> planning_path_points_; //规划的参考线
  std::vector<SLPoint> sl_planning_path_;            //转换的sl坐标

  std::vector<SLPoint> sl_dynamic_obstacles_; //障碍物信息应该有ID
  SLPoint plan_start_;                        //规划起点
  std::vector<SLPoint> sl_virtual_obstacles_; //虚拟障碍物的sl坐标
  std::vector<ReferencePoint> xy_virtual_obstacles_; //虚拟障碍物的xy坐标

  std::vector<STLine> st_obstacles;
  std::vector<std::vector<STPoint>> sample_points_;

  std::vector<STPoint> dp_speed_points_;
  std::vector<STPoint> dp_speed_points_dense_;

  //凸空间
  Eigen::VectorXd convex_s_lb_;
  Eigen::VectorXd convex_s_ub_;
  Eigen::VectorXd convex_ds_dt_lb_;
  Eigen::VectorXd convex_ds_dt_ub_;

  std::vector<STPoint> qp_speed_points_;

  std::vector<TrajectoryPoint> trajectory_points_;
  Trajectory trajectory_;
};
