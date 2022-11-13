#pragma once
#include "common/pnc_point.h"
#include <vector>

/*
　　类成员变量的初始化
    1.类成员不可以在定义时初始化
    2、const类型的成员必须在初始化化列表进行初始化；一般变量在在初始化列表中初始化的顺序决定于定义的顺序
    尝试运行以上的例子，可以发现，在用T():name(),name2():{}的形式进行初始化的时候，
    初始化的顺序是与类中本来类的变量位置的顺序有关，与T():name(),name2():{}的形式中变量名的顺序无关。
    3、static类型的成员变量需要在类外进行初始化
*/



class Trajectory {

public:
  Trajectory(/* args */);
  ~Trajectory();
  const std::vector<TrajectoryPoint> trajectory_points() const;
  void
  set_trajectory_points(const std::vector<TrajectoryPoint> &trajectory_points);

private:
  /* data */
  std::vector<TrajectoryPoint> trajectory_points_;
};

