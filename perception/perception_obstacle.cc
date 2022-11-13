#include "perception_obstacle.h"

const std::vector<ObstacleInfo> PerceptionObstacle::static_obstacles() const {
  return static_obstacles_;
}

const std::vector<ObstacleInfo> PerceptionObstacle::dynamic_obstacles() const {
  return dynamic_obstacles_;
}

void PerceptionObstacle::UpdateObstacleInfo(int time,
                                            LocalizationInfo LocalizationInfo) {

  //静态障碍物
  ObstacleInfo obs0;
  obs0.ID = 0;
  obs0.x = 400;
  obs0.y = 20;
  obs0.v = 0;

  //静态障碍物1
}

void PerceptionObstacle::AddStaticObstacle(int id, double init_x, double init_y,
                                           double init_heading, double init_v) {
  ObstacleInfo obs;
  if (static_obstacles_.empty()) {
    obs.ID = 0;
  } else {
    obs.ID = static_obstacles_.size() - 1;
  }
  obs.init_x = init_x;
  obs.init_y = init_y;
  obs.init_heading = init_heading;

  obs.x = init_x;
  obs.y = init_y;
  obs.heading = init_heading;
  obs.v = 0;

  static_obstacles_.push_back(obs);
}

PerceptionObstacle::PerceptionObstacle(/* args */) {}

PerceptionObstacle::~PerceptionObstacle() {}
