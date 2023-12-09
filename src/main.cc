/*主函数入口
1.routing自定义一条ReferenceLine
  location，选择轨迹前进n个点作为定位点
  preceotion,选取前进方向范围内的障碍物，作为识别的障碍物信息

2.调用参考线平滑模块，对参考线进行平滑
3.调用EMPlanner算法进行Planning
4.
*/

#include "EMPlanner/EMPlanner.h"
#include "localization/localization_estimate.h"
#include "perception/perception_obstacle.h"
#include "reference_line/reference_line_provider.h"
#include "routing/routing_path.h"
#include <memory>
#include <unistd.h>


int main(int argc, char const *argv[]) {

 


  //构造配置
  std::unique_ptr<EMPlannerConfig> config = std::make_unique<EMPlannerConfig>();

  //构造路由模块指针
  std::unique_ptr<RoutingPath> routing_path = std::make_unique<RoutingPath>();

  //定位信息指针
  std::unique_ptr<LocalizationEstimate> localization =
      std::unique_ptr<LocalizationEstimate>();
  //障碍物信息
  std::unique_ptr<PerceptionObstacle> perception =
      std::unique_ptr<PerceptionObstacle>();
  //构造全局路径

  routing_path->CreatePath();
  //创建一个静态障碍物
  perception->AddStaticObstacle(0, 400, 20, 0, 0);
  auto routing_path_points = routing_path->routing_path_points();

  LocalizationInfo localization_info;
  ReferenceLine reference_line;     //当前参考线
  ReferenceLine pre_reference_line; //上一时刻参考线

  Trajectory trajectory;
  Trajectory pre_trajectory;
  uint64_t time = 0;

  while (1) {
    //每10ms次循环执行一次
    if (time % 10 - 0 < 1e-10) {

      //线程1 10ms
      // 1.更新车辆信息，根据轨迹更新位置。取规划轨迹+10ms的位置
      localization->UpdateLocalizationInfo(time, trajectory); //根据
      localization_info = localization->localization_info();
      // 2.障碍物信息
      perception->UpdateObstacleInfo();
    }

    //每100次循环执行一次
    if (time % 100 < 1e-10) {

      //线程2 100ms
      // 2.参考线生成,参考新默认-30m~150m
      std::unique_ptr<ReferenceLineProvider> reference_line_provider =
          std::unique_ptr<ReferenceLineProvider>();
      pre_reference_line = reference_line;
      //传参应该是数据类型，而不是类的对象
      reference_line_provider->Provide(routing_path_points, localization_info,
                                       pre_reference_line, reference_line);
      // 2.规划器规划
      std::unique_ptr<EMPlanner> em_planner =
          std::make_unique<EMPlanner>(*config); //规划器初始化
      TrajectoryPoint plan_start;
      Trajectory stitch_trajectory;
      pre_trajectory = trajectory;
      em_planner->CalPlaningStartPoint(pre_trajectory, localization_info,
                                       plan_start, stitch_trajectory);
      std::vector<ReferencePoint> xy_virtual_obstacles;
      em_planner->Plan(time, plan_start, reference_line, localization,
                       perception, trajectory, xy_virtual_obstacles);
      perception->UpdateVirtualObstacle(
          std::vector<ReferencePoint> xy_virtual_obstacles);
    }
    time++;
    std::usleep(1000); //延时1ms
  }

  return 0;
}
