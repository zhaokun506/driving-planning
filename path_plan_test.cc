#include "EMPlanner/EMPlanner.h"
#include "localization/localization_estimate.h"
#include "perception/perception_obstacle.h"
#include "plot/plot.h"
#include "reference_line/reference_line_provider.h"
#include "routing/routing_path.h"
#include <memory>
#include <unistd.h>

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char const *argv[]) {
  std::vector<double> x = {0};
  std::vector<double> y = {0};

  plt::plot(x, y);

  //构造路由模块指针
  std::unique_ptr<RoutingPath> routing_path =
      std::make_unique<RoutingPath>(); //注意是make_unique

  //定位信息指针
  std::unique_ptr<LocalizationEstimate> localization =
      std::make_unique<LocalizationEstimate>();
  //障碍物信息
  std::unique_ptr<PerceptionObstacle> perception =
      std::make_unique<PerceptionObstacle>();
  //构造全局路径

  routing_path->CreatePath();
  //创建一个静态障碍物
  perception->AddStaticObstacle(0, 30, -0.5, 0,
                                0); //此代码执行错误，待查找原因

  auto obstacle_info = perception->static_obstacles();

  auto routing_path_points = routing_path->routing_path_points();

  LocalizationInfo localization_info;

  ReferenceLine reference_line; //当前参考线
  std::vector<ReferencePoint> rp;
  rp.clear();
  reference_line.set_reference_points(rp);

  ReferenceLine pre_reference_line; //上一时刻参考线

  Trajectory trajectory;
  Trajectory pre_trajectory;
  uint64_t time = 0;

  localization->UpdateLocalizationInfo(0, trajectory);
  localization_info = localization->localization_info();

  // 2.参考线生成,参考新默认-30m~150m
  std::unique_ptr<ReferenceLineProvider> reference_line_provider =
      std::make_unique<ReferenceLineProvider>();
  pre_reference_line = reference_line;
  //传参应该是数据类型，而不是类的对象
  reference_line_provider->Provide(routing_path_points, localization_info,
                                   pre_reference_line, reference_line);
  auto raw_reference_line = reference_line_provider->raw_reference_line();

  EMPlannerConfig emplanner_config;
  std::unique_ptr<EMPlanner> em_planner =
      std::make_unique<EMPlanner>(emplanner_config);

  TrajectoryPoint plan_start_point;
  Trajectory stitch_trajectory;

  std::vector<ObstacleInfo> virtual_obs;

  em_planner->CalPlaningStartPoint(pre_trajectory, localization_info,
                                   &plan_start_point, &stitch_trajectory);

  em_planner->Plan(0, plan_start_point, raw_reference_line, localization_info,
                   obstacle_info, &trajectory, virtual_obs);

  std::unique_ptr<Plot> plot = std::make_unique<Plot>();

  plot->PlotRoutingPath(routing_path_points, "k");
  plot->PlotReferenceLine(reference_line, "y");

  plot->PlotSLPath(em_planner->sl_graph_->dp_path_points(), "r");
  plot->PlotSLPath(em_planner->sl_graph_->dp_path_points_dense(), "g");

  for (const auto obs : obstacle_info) {
    plot->PlotObs(obs, "k");
  }

  plt::show();
  return 0;
}