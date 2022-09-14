/*主函数入口
1.routing自定义一条ReferenceLine
  location，选择轨迹前进n个点作为定位点
  preceotion,选取前进方向范围内的障碍物，作为识别的障碍物信息

2.调用参考线平滑模块，对参考线进行平滑
3.调用EMPlanner算法进行Planning

4.

*/

#include <memory>
#include "reference_line_provider.h"
#include "EMPlanner.h"

int main(int argc, char const *argv[])
{
  //构造配置

  //构造全局路径
  std::shared_ptr<RoutingPath> routing_path = std::make_shared<RoutingPath>();
  routing_path->GetRoutingPathFromCSV();
  while (1)
  {

    // 1.参考线生成,参考新默认-30m~150m
    std::shared_ptr<ReferenceLineProvider> reference_line_provider = std::make_shared<ReferenceLineProvider>();
    reference_line_provider->Provide((*routing_path), );
    // 2.规划器规划,
    // dp采样数默认，row必须是11，col必须是6。dl1m, ds15m
    // dp增密数，60个，间隔1m
    std::shared_ptr<EMPlanner> em_planner = std::make_shared<EMPlanner>();
    em_planner->Plan();

    // 3.车辆位置和障碍物信息
  }

  return 0;
}
