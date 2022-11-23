#include "localization_estimate.h"

LocalizationEstimate::LocalizationEstimate(/* args */) {
  localization_info_.t = 0;

  localization_info_.x = 0;
  localization_info_.y = 0;

  localization_info_.v = 0;
  localization_info_.vx = 0;
  localization_info_.vy = 0;

  localization_info_.a = 0;
  localization_info_.ax = 0;
  localization_info_.ay = 0;

  localization_info_.heading = 0;
}

const LocalizationInfo LocalizationEstimate::localization_info() const {

  return localization_info_;
}
//根据当前时间和当前规划的轨迹，更新定位信息，假设控制完全跟踪规划
void LocalizationEstimate::UpdateLocalizationInfo(u_int64_t time,
                                                  Trajectory trajectory) {

  if (time == 0) {
    localization_info_.x = 0;
    localization_info_.y = 0;
    localization_info_.t = 0;

    localization_info_.v = 0;
    localization_info_.vx = 0;
    localization_info_.vy = 0;

    localization_info_.a = 0;
    localization_info_.ax = 0;
    localization_info_.ay = 0;

    localization_info_.heading = 0;
    localization_info_.kappa = 0;

  } else {
    auto trajectory_points = trajectory.trajectory_points();
    int i = 0;
    for (i = 0; i < trajectory_points.size() - 1; i++) {
      if (trajectory_points[i].t <= time && trajectory_points[i + 1].t > time)
        break;
    }

    //        localization_info_ = trajectory_points[i];
    localization_info_.t = time;

    localization_info_.x = trajectory_points[i].x;
    localization_info_.y = trajectory_points[i].y;

    localization_info_.v = trajectory_points[i].v;
    localization_info_.vx =
        trajectory_points[i].v * cos(trajectory_points[i].heading);
    localization_info_.vy =
        trajectory_points[i].v * sin(trajectory_points[i].heading);

    localization_info_.a = trajectory_points[i].a;
    localization_info_.ax =
        trajectory_points[i].a * cos(trajectory_points[i].heading);
    localization_info_.ay =
        trajectory_points[i].a * sin(trajectory_points[i].heading);

    localization_info_.heading = trajectory_points[i].heading;
    localization_info_.kappa = trajectory_points[i].kappa;
  }
}