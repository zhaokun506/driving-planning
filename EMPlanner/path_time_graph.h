//该类用于维护EMPlaner的st图和sl图。
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

// SL图点坐标
class SLPoint {
private:
  /* data */
public:
  SLPoint();
  ~SLPoint();

  int index; //对应参考线的索引
  double s;
  double ds_dt;
  double dds_dt;
  double l;
  double dl_dt;
  double ddl_dt;
  double dddl_dt;
  double dl_ds;
  double ddl_ds;
  double dddl_ds;

  double cost2start =
      INT_MAX; //起点到该点的cost  这些是对于采样点来说的，其他的用不上
  int pre_mincost_row; //最小cost前一列的行号
};

class PathTimeGraph {
private:
  /* data */
public:
  // 1.初始化以参考线和车辆位置信息构建，sl坐标系。对应index2s
  PathTimeGraph(const ReferenceLine &reference_line,
                const ReferencePoint &host_project_point,
                EMPlannerConfig emplaner_conf);
  ~PathTimeGraph();

  //将计算vector<point>的sl
  // 1.计算投影点
  // 2.XY2XL
  //自有信息：reference_line_,
  // host_project_point_。//此函数其他地方也可以使用，因此在其中改变其私有变量☆☆☆☆☆☆☆
  void
  Cartesian2Frenet(const ReferenceLine &reference_line,           //参考线
                   const std::vector<SLPoint> &sl_reference_line, //参考线的sl值
                   const std::vector<TrajectoryPoint> &points_wcs, //待转换的点
                   const std::vector<ReferencePoint> &match_points,
                   const std::vector<ReferencePoint>
                       &project_points, //待转换点在参考线上的投影点
                   std::vector<SLPoint> &points_fcs);

  //自然坐标系转笛卡尔坐标系
  void
  Frenet2Cartesian(const ReferenceLine &reference_line,           //参考线
                   const std::vector<SLPoint> &sl_reference_line, //参考线的sl值
                   const std::vector<SLPoint> &points_fcs, //待转换的点
                   std::vector<ReferencePoint> &points_wcs);

  // 1.根据参考线和自车投影点生成s轴
  // 公用函数其他模块也可以调用。不能使用类内的相关变量
  void InitSAxis(const ReferenceLine &reference_line,
                 const ReferencePoint &host_project_point,
                 std::vector<SLPoint> &sl_reference_line)

      // 2.自车sl，记录自车位置。自车投影
      void SetHostSl(const LocalizationInfo host_local);
  // 3.障碍物sl，记录障碍物位置。障碍物投影
  void SetStaticObstaclesSl(const std::vector<ObstacleInfo> static_obstacles);

  // 4.dp主算法相关生成采样点
  void CreateSamplingPoint(const int row, const int col, const double sample_s,
                           const double sample_l); //对SamplePoints进行操作
  double CalcPathCost(SLPoint point1, SLPoint point2, EMPlannerConfig conf);
  void CalcQuinticCoeffient(const SLPoint &point1, const SLPoint &point2,
                            std::vector<double> &QuinticCoeffient);
  void PathDynamicPlanning();
  // 5.dp路径插值
  void DpPathInterpolation(int interpolation_num, double ds);

  //二次规划相关
  void
  GenerateConvexSpace(double static_obs_length,
                      double static_obs_width); //根据dp_path,输出l_minx,l_max
  int FindNearIndex(std::vector<SLPoint> dp_path_points_dense, double s);
  void PathQuadraticProgramming();
  void QpPathInterpolation(int interpolation_num, double ds);

  void GeneratePlaningPath();
  void CalcProjPoint(const SLPoint sl_point,
                     std::vector<SLPoint> sl_reference_line,
                     const ReferenceLine reference_line,
                     ReferencePoint &project_point);

  const ReferenceLine planning_path() const;

private:
  EMPlannerConfig config_;

  ReferenceLine reference_line_;      //参考线，世界坐标系
  ReferencePoint host_project_point_; //车辆在参考线的投影点
  LocalizationEstimate localization_; //车辆的定位信息，世界坐标系
  std::vector<SLPoint> sl_reference_line_; // l均为0，s由车辆定位决定
  SLPoint sl_host_;                        //自车的sl
  std::vector<SLPoint> sl_static_obstacles_;        //静态障碍物的sl
  std::vector<std::vector<SLPoint>> sample_points_; //采样点矩阵
  std::vector<SLPoint> dp_path_points_;             //动态规划路径点
  std::vector<SLPoint> dp_path_points_dense_; //动态规划加密路径点
  std::vector<SLPoint> qp_path_points_;       //二次规划路径点
  std::vector<SLPoint> qp_path_points_dense_; //增密后的二次规划路径点

  ReferenceLine planning_path_; //转换后的世界坐标系轨迹

  Eigen::VectorXd l_min_;
  Eigen::VectorXd l_max_;
};
