#pragma once

#include "common/pnc_point.h"
#include <math.h>
#include <vector>

class PerceptionObstacle {

public:
  PerceptionObstacle(/* args */);
  ~PerceptionObstacle() = default;
  const std::vector<ObstacleInfo> static_obstacles() const;
  const std::vector<ObstacleInfo> dynamic_obstacles() const;
  void UpdateObstacleInfo(
      int time,
      LocalizationInfo localization_info); //对障碍物列表的状态进行更新
  void UpdateVirtualObstacle(std::vector<ReferencePoint> xy_virtual_obstacles);

  void AddStaticObstacle(int id, double init_x, double init_y,
                         double init_heading, double init_v);
  void AddDynamicObstacle(int id, double init_x, double init_y,
                          double init_heading, double init_v);
  void FilterAndOutObstacleInfo(
      LocalizationInfo localization_info); //根据车辆的位置输出障碍物信息

private:
  /* data */
  std::vector<ObstacleInfo> static_obstacles_;
  std::vector<ObstacleInfo> dynamic_obstacles_;
};
