
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

// SL图点坐标
class SLPoint {
private:
  /* data */
public:
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

  double cost2start; //起点到该点的cost  这些是对于采样点来说的，其他的用不上
  int pre_mincost_row; //最小cost前一列的行号
};

class STPoint {
public:
  double t;
  double s;
  double ds_dt;
  double dds_dt;

  double cost2start;
  double pre_mincost_row;
};

class STLine {
public:
  STPoint left_point;
  STPoint right_point;
};