/*
    位置轨迹的数据类型
*/
#pragma once

#include "EMPlanner/trajectory.h"
#include <math.h>

class LocalizationEstimate {
private:
  /* data */
  LocalizationInfo localization_info_;

public:
  LocalizationEstimate(/* args */);
  ~LocalizationEstimate() = default;

  void UpdateLocalizationInfo(u_int64_t time, Trajectory trajectory);
  const LocalizationInfo localization_info() const;
};
