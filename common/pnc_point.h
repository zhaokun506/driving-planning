
/*
routing
全局的xy变量
*/
#pragma once

class MapPoint {
public:
  /* data */
  double x;
  double y;
};

class ReferencePoint : public MapPoint {
public:
  /* data */
  double index;
  double heading; //方向角
  double kappa;   //曲率
  double dkappa;  //曲率的导数
};

class TrajectoryPoint : public ReferencePoint {
  /* data */
public:
  double v;  //速度
  double vx; //车身坐标系
  double vy;
  double a; //加速度
  double ax;
  double ay;
  double t; //时间
};

/*障碍物的数据类型*/
class ObstacleInfo : public TrajectoryPoint {
public:
  int ID;
  double init_x;
  double init_y;
  double init_heading;

  double v;
  double heading;
};

class LocalizationInfo : public TrajectoryPoint {
public:
};
