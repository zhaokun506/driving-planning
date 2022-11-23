#include "path_time_graph.h"

PathTimeGraph::PathTimeGraph(const ReferenceLine &reference_line,
                             const EMPlannerConfig &emplaner_conf) {
  reference_line_ = reference_line;
  config_ = emplaner_conf;

  //初始化根据参考线，以自车的投影点为原点，生产sl
  InitSAxis(reference_line, &sl_reference_line_);
}

void PathTimeGraph::InitSAxis(const ReferenceLine &reference_line,
                              std::vector<SLPoint> *sl_reference_line) {

  //初始化根据参考线，以自车的投影点为原点，生成sl
  auto reference_points = reference_line.reference_points();
  auto host_match_index = reference_line.match_point_index();
  auto host_project_point = reference_line.host_project_point();
  int len = reference_points.size();
  (*sl_reference_line).resize(len);

  // 首先计算以path起点为坐标原点,各个点的s[i]
  (*sl_reference_line)[0].s = 0;
  for (int i = 1; i < len - 1; i++) {
    (*sl_reference_line)[i].s =
        sqrt(pow(reference_points[i].x - reference_points[i - 1].x, 2) +
             pow(reference_points[i].y - reference_points[i - 1].y, 2)) +
        (*sl_reference_line)[i - 1].s;
  }
  //再计算以轨迹起点到frenet_path的坐标原点的弧长，记为s0，再用s[i] - s0
  //就是最终的结果
  double s_host_match = (*sl_reference_line)[host_match_index].s;

  ReferencePoint host_match_point = reference_points[host_match_index];
  ReferencePoint host_match_point_next = reference_points[host_match_index + 1];
  std::pair<double, double> vector_project_match =
      std::make_pair(host_project_point.x - host_match_point.x,
                     host_project_point.y - host_match_point.y);
  std::pair<double, double> vector_match_next =
      std::make_pair(host_match_point_next.x - host_match_point.x,
                     host_match_point_next.y - host_match_point.y);
  double s0 = 0;
  if (vector_project_match.first * vector_match_next.first +
          vector_project_match.second * vector_match_next.second >
      0)
    s0 = s_host_match + sqrt(pow(vector_project_match.first, 2) +
                             pow(vector_project_match.second, 2));
  else
    s0 = s_host_match - sqrt(pow(vector_project_match.first, 2) +
                             pow(vector_project_match.second, 2));

  for (int i = 0; i < len; i++) {
    (*sl_reference_line)[i].s = (*sl_reference_line)[i].s - s0;
  }
}

/*-------------------------------------设置自车位置和障碍物部分-------------------------------------------------------------*/
void PathTimeGraph::SetStartPointSl(TrajectoryPoint plan_start_point) {
  //计算投影
  //坐标转换
  std::vector<MapPoint> map_points;

  map_points.resize(1);
  map_points[0].x = plan_start_point.x;
  map_points[0].y = plan_start_point.y;

  //定义变量，用于存储计算结果
  std::vector<ReferencePoint> start_match_points, start_project_points;

  ReferenceLineProvider::FindMatchAndProjectPoint(reference_line_, map_points,
                                                  0, 3, start_match_points,
                                                  start_project_points);
  std::vector<TrajectoryPoint> start_point_vec;
  start_point_vec.push_back(plan_start_point);
  std::vector<SLPoint> points_fcs;
  Cartesian2Frenet(reference_line_, sl_reference_line_, start_point_vec,
                   start_match_points, start_project_points, points_fcs);
  sl_plan_start_ = points_fcs.front();

  //调试
  sl_plan_start_.s = 0;
  sl_plan_start_.l = 0;
  sl_plan_start_.dl_ds = 0;
  sl_plan_start_.ddl_ds = 0;
}

void PathTimeGraph::SetStaticObstaclesSl(
    const std::vector<ObstacleInfo> static_obstacles) {
  std::vector<MapPoint> map_points;
  int size = static_obstacles.size();

  map_points.resize(size);
  for (int i = 0; i < size; i++) {
    map_points[i].x = static_obstacles[i].x;
    map_points[i].y = static_obstacles[i].y;
  }
  //定义变量，用于存储计算结果
  std::vector<ReferencePoint> obstacles_match_points, obstacles_project_points;

  ReferenceLineProvider::FindMatchAndProjectPoint(reference_line_, map_points,
                                                  0, 3, obstacles_match_points,
                                                  obstacles_project_points);

  //此部分比较妥协，暂时使用这种赋值循环，解决障碍物信息和参数不一致得到的问题
  std::vector<TrajectoryPoint> traj_points;

  traj_points.resize(size);
  for (int i = 0; i < size; i++) {
    traj_points[i].x = static_obstacles[i].x;
    traj_points[i].y = static_obstacles[i].y;
    traj_points[i].vx = static_obstacles[i].vx;
    traj_points[i].vy = static_obstacles[i].vy;
    traj_points[i].ax = static_obstacles[i].ax;
    traj_points[i].ay = static_obstacles[i].ay;
  }
  // std::vector<SLPoint> *obstacles_points_fcs;
  Cartesian2Frenet(reference_line_, sl_reference_line_, traj_points,
                   obstacles_match_points, obstacles_project_points,
                   sl_static_obstacles_);

  //调试
  sl_static_obstacles_.resize(1);
  sl_static_obstacles_[0].s = 30;
  sl_static_obstacles_[0].l = -0.5;
  sl_static_obstacles_[0].dl_ds = 0;
  sl_static_obstacles_[0].ddl_ds = 0;
}
//需要加深理解，引用，指针作为参数，对原来变量的影响
//--------------------------------------坐标变换-------------------------------------------------
void PathTimeGraph::Cartesian2Frenet(
    const ReferenceLine &reference_line,             //参考线
    const std::vector<SLPoint> &sl_reference_line,   //参考线的sl值
    const std::vector<TrajectoryPoint> &points_wcs,  //待转换的点
    const std::vector<ReferencePoint> &match_points, //匹配点
    const std::vector<ReferencePoint>
        &project_points,              //待转换点在参考线上的投影点
    std::vector<SLPoint> &points_fcs) //转换点的SL
{

  //计算s的值，根据匹配点和投影点，匹配点的s值已经求得，存在于sl_reference_line
  auto reference_line_points = reference_line.reference_points();
  int size = points_wcs.size();
  points_fcs.resize(size);
  for (int i = 0; i < size; i++) {
    // 1.求s，可以写成一个子函数
    //此处是否可以用eigen解决vector2d向量的问题，
    //求match->project
    auto vector_porj2mactch =
        std::make_pair<double, double>(project_points[i].x - match_points[i].x,
                                       project_points[i].y - match_points[i].y);
    //求match->next macth向量
    std::pair<double, double> vector_next2match =
        std::make_pair<double, double>(0, 0);
    if (match_points[i].index < reference_line_points.size() - 1) {
      vector_next2match = std::make_pair<double, double>(
          reference_line_points[match_points[i].index + 1].x -
              match_points[i].x,
          reference_line_points[match_points[i].index + 1].y -
              match_points[i].y);
    } else {
      vector_next2match = std::make_pair<double, double>(
          match_points[i].x -
              reference_line_points[match_points[i].index - 1].x,
          match_points[i].y -
              reference_line_points[match_points[i].index - 1].y);
    }
    //判断投影点在匹配的前还是后
    if (vector_porj2mactch.first * vector_next2match.first +
            vector_porj2mactch.second * vector_next2match.second >
        0) {
      points_fcs[i].s =
          sl_reference_line[(match_points[i].index)].s +
          sqrt(vector_porj2mactch.first * vector_porj2mactch.first +
               vector_porj2mactch.second + vector_porj2mactch.second);
    } else {
      points_fcs[i].s =
          sl_reference_line[(match_points[i].index)].s -
          sqrt(vector_porj2mactch.first * vector_porj2mactch.first +
               vector_porj2mactch.second + vector_porj2mactch.second);
    }

    // 2.求l
    // l=(o_h-o_r)*n_r,h为待求点,r为参考线投影点,o为原点
    auto vector_oh = std::make_pair<double, double>(1.0 * points_wcs[i].x,
                                                    1.0 * points_wcs[i].y);
    auto vector_nr = std::make_pair<double, double>(
        -sin(project_points[i].heading), cos(project_points[i].heading));
    auto vector_or = std::make_pair<double, double>(1.0 * project_points[i].x,
                                                    1.0 * project_points[i].y);

    points_fcs[i].l = (vector_oh.first - vector_nr.first) * vector_or.first +
                      (vector_oh.second - vector_nr.second) * vector_or.second;

    // 3.求ds/dt,dl/dt, dl/ds*
    auto vector_vh = std::make_pair<double, double>(1.0 * points_wcs[i].vx,
                                                    1.0 * points_wcs[i].vy);
    auto vector_tr = std::make_pair<double, double>(
        cos(project_points[i].heading), sin(project_points[i].heading));
    points_fcs[i].dl_dt =
        vector_vh.first * vector_nr.first + vector_vh.second * vector_nr.second;
    points_fcs[i].ds_dt = (vector_vh.first * vector_tr.first +
                           vector_vh.second * vector_tr.second) /
                          (1 - project_points[i].kappa * points_fcs[i].l);
    if (abs(points_fcs[i].ds_dt) < 1e-6)
      points_fcs[i].dl_ds = 0;
    else
      points_fcs[i].dl_ds = points_fcs[i].dl_dt / points_fcs[i].ds_dt;
    // 3.求dds/dt,ddl/dt, ddl/ds*
    auto vector_ah = std::make_pair<double, double>(1.0 * points_wcs[i].ax,
                                                    1.0 * points_wcs[i].ay);
    points_fcs[i].ddl_dt = vector_ah.first * vector_nr.first +
                           vector_ah.first * vector_nr.first -
                           project_points[i].kappa *
                               (1 - project_points[i].kappa * points_fcs[i].l *
                                        pow(points_fcs[i].ds_dt, 2));
    points_fcs[i].dds_dt =
        (1 / (1 - project_points[i].kappa * points_fcs[i].l)) *
        ((vector_ah.first * vector_tr.first +
          vector_ah.second * vector_tr.second) +
         2 * project_points[i].kappa * points_fcs[i].dl_ds *
             pow(points_fcs[i].ds_dt, 2));
    if (abs(points_fcs[i].dds_dt) < 10e-5) {
      points_fcs[i].ddl_ds = 0;
    } else {
      points_fcs[i].ddl_ds = points_fcs[i].ddl_dt / points_fcs[i].dds_dt;
    }
  }
}

//自然坐标系转笛卡尔坐标系
void PathTimeGraph::Frenet2Cartesian(
    const ReferenceLine &reference_line,           //参考线
    const std::vector<SLPoint> &sl_reference_line, //参考线的sl值
    const std::vector<SLPoint> &points_fcs,        //待转换的点
    std::vector<ReferencePoint> *points_wcs) {}

/*
注：形参是声明的引用，注意这个引用没有初始化，在调用函数时实现了初始化，做到了真正意义变量传递
引用和指针做形参是很常见的问题，但是它们在做参数的时候是有区别的
指针他是一个变量，有具体的值，他的值是一个地址。
而引用是对一个变量的引用，是变量的别名，并且在引用的时候必须要初始化。
*/

/*----------------------------------------路径动态规划部分开始------------------------------------------------------------------------------------*/

void PathTimeGraph::PathDynamicPlanning() {
  int rows = sample_points_.size();
  int cols = sample_points_[0].size();
  // 1.计算起点到第一列所有点的cost
  for (int i = 0; i < rows; i++) {
    sample_points_[i][0].cost2start =
        CalcPathCost(sl_plan_start_, sample_points_[i][0]);
  }

  // 2.动态规划主结构
  for (int j = 1; j < cols; j++) //第j列
  {
    for (int i = 0; i < rows; i++) //第i行
    {
      //计算起点到ij点的cost
      sample_points_[i][j].cost2start = DBL_MAX;
      for (int k = 0; k < rows; k++) {
        //计算前一列每一点到当前点的cost，结合前一列点的cost2start，获得起点到当前点的cost，通过比较获取最小值
        double cost =
            sample_points_[k][j - 1].cost2start +
            CalcPathCost(sample_points_[k][j - 1], sample_points_[i][j]);
        if (cost < sample_points_[i][j].cost2start) {
          sample_points_[i][j].cost2start = cost;
          sample_points_[i][j].pre_mincost_row = k; //记录最优路径上一列的行号
        }
      }
    }
  }

  // 3.回溯路径
  std::vector<SLPoint> dp_path_points; // rows-1个+起点
  dp_path_points.resize(cols + 1);
  dp_path_points[0] = sl_plan_start_; //第一个点是起点
  //找到最后一列
  dp_path_points[cols] = sample_points_[0][cols - 1];
  for (int i = 1; i < rows; i++) {
    if (sample_points_[i][cols - 1].cost2start <
        dp_path_points[cols].cost2start)
      dp_path_points[cols] = sample_points_[i][cols - 1];
  }

  for (int j = cols - 1; j > 0; j--) {
    int pre_mincost_row = dp_path_points[j + 1].pre_mincost_row;
    dp_path_points[j] = sample_points_[pre_mincost_row][j - 1];
  }
  dp_path_points_ = dp_path_points; //赋值私有变量
}

double PathTimeGraph::CalcPathCost(SLPoint point_satrt, SLPoint point_end) {
  //计算两点之间五次多项式连接的代价
  // 1.计算五次多项式系数
  std::vector<double> qc;
  CalcQuinticCoeffient(point_satrt, point_end, &qc);
  //中间取10个点计算cost，加上终点11个
  //插值
  SLPoint quintic_path_point;
  double cost_smooth = 0;
  double cost_ref = 0;
  double cost_collision = 0;

  for (int i = 0; i < 10; i++) { //不包含起点包含终点
    //计算第i和点的s,l,dl,ddl,dddl
    quintic_path_point.s =
        point_satrt.s + (i + 1) * (point_end.s - point_satrt.s) / 10;
    quintic_path_point.l = qc[0] + qc[1] * quintic_path_point.s +
                           qc[2] * pow(quintic_path_point.s, 2) +
                           qc[3] * pow(quintic_path_point.s, 3) +
                           qc[4] * pow(quintic_path_point.s, 4) +
                           qc[5] * pow(quintic_path_point.s, 5);
    quintic_path_point.dl_ds = qc[1] + 2 * qc[2] * quintic_path_point.s +
                               3 * qc[3] * pow(quintic_path_point.s, 2) +
                               4 * qc[4] * pow(quintic_path_point.s, 3) +
                               5 * qc[5] * pow(quintic_path_point.s, 4);
    quintic_path_point.ddl_ds = 2 * qc[2] + 6 * qc[3] * quintic_path_point.s +
                                12 * qc[4] * pow(quintic_path_point.s, 2) +
                                20 * qc[5] * pow(quintic_path_point.s, 3);
    quintic_path_point.dddl_ds = 6 * qc[3] + 24 * qc[4] * quintic_path_point.s +
                                 60 * qc[5] * pow(quintic_path_point.s, 2);
    //平滑代价
    cost_smooth = cost_smooth +
                  config_.dp_cost_dl * pow(quintic_path_point.dl_ds, 2) +
                  config_.dp_cost_ddl * pow(quintic_path_point.ddl_ds, 2) +
                  config_.dp_cost_dddl * pow(quintic_path_point.dddl_ds, 2);
    //如果ddl或者切向角过大，则增加cost
    ////此处导致代价都很大
    if (abs(quintic_path_point.ddl_ds) > 0.5 ||
        abs(atan(quintic_path_point.dl_ds)) > 0.4 * M_PI) {
      cost_smooth = cost_smooth + 10e7;
    }
    //参考线代价
    cost_ref = config_.dp_cost_ref * pow(quintic_path_point.l, 2);

    //碰撞代价,这里做了非常简化的质点模型，认为障碍物就是一个点
    //求该点到障碍物的最小距离
    double min_d = DBL_MAX;
    for (const auto &obstacle : sl_static_obstacles_) //遍历静态障碍物
    {
      double dlon = quintic_path_point.s - obstacle.s;
      double dlat = quintic_path_point.l - obstacle.l;
      double square_d = pow(dlon, 2) + pow(dlat, 2);
      min_d = std::min(min_d, square_d);
    }
    // cost_collision =
    //     cost_collision + config_.dp_cost_collision * pow(10, -min_d / 3);
    if (min_d > 25)
      cost_collision = cost_collision + 0;
    else if (min_d < 25 && min_d > 9)
      cost_collision = cost_collision + config_.dp_cost_collision * 500 / min_d;
    else
      cost_collision = cost_collision + config_.dp_cost_collision * 10e5;

    //此处忽略左右判断？？？
  }
  //三个代价不是一个数量级
  return cost_smooth + cost_ref + cost_collision;
}

void PathTimeGraph::CalcQuinticCoeffient(
    const SLPoint &point_s, const SLPoint &point_e,
    std::vector<double> *QuinticCoeffient) {
  /* % l = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
  % l' = a1 + 2 * a2 * s + 3 * a3 * s^2 + 4 * a4 * s^3 + 5 * a5 * s^4
  % l'' = 2 * a2 + 6 * a3 * s + 12 * a4 * s^2 + 20 * a5 * s^3*/
  double start_s = point_s.s;
  double start_s2 = pow(point_s.s, 2);
  double start_s3 = pow(point_s.s, 3);
  double start_s4 = pow(point_s.s, 4);
  double start_s5 = pow(point_s.s, 5);

  double end_s = point_e.s;
  double end_s2 = pow(point_e.s, 2);
  double end_s3 = pow(point_e.s, 3);
  double end_s4 = pow(point_e.s, 4);
  double end_s5 = pow(point_e.s, 5);

  Eigen::MatrixXd A(6, 6);
  A << 1, start_s, start_s2, start_s3, start_s4, start_s5, 0, 1, 2 * start_s,
      3 * start_s2, 4 * start_s3, 5 * start_s4, 0, 0, 2, 6 * start_s,
      12 * start_s2, 20 * start_s3, 1, end_s, end_s2, end_s3, end_s4, end_s5, 0,
      1, 2 * end_s, 3 * end_s2, 4 * end_s3, 5 * end_s4, 0, 0, 2, 6 * end_s,
      12 * end_s2, 20 * end_s3;
  Eigen::VectorXd B(6);
  B << point_s.l, point_s.dl_ds, point_s.ddl_ds, point_e.l, point_e.dl_ds,
      point_e.ddl_ds;

  Eigen::VectorXd coeff(6);
  coeff = A.inverse() * B;
  (*QuinticCoeffient).resize(coeff.size());
  for (int i = 0; i < coeff.size(); i++) {
    (*QuinticCoeffient)[i] = coeff[i];
  }
}

void PathTimeGraph::CreateSamplingPoint(
    const int row, const int col, const double sample_s,
    const double
        sample_l) { /*       0   1   2   3
                           0 x   x   x   x
                           1 x   x   x   x
                      -----2-x---x---x---x----------------reference_line
                           3 x   x   x   x
                           4 x   x   x   x


                   */
  sample_points_.resize(row);
  for (int j = 0; j < col; j++) {
    for (int i = 0; i < row; i++) {
      sample_points_[i].resize(col);
      sample_points_[i][j].s = sl_host_.s + (j + 1) * sample_s;
      sample_points_[i][j].l = ((row + 1) / 2 - (i + 1)) * sample_l;
      sample_points_[i][j].dl_ds = 0;
      sample_points_[i][j].ddl_ds = 0;
    }
  }

} //对sample_points_进行操作

void PathTimeGraph::DpPathInterpolation(int interpolation_num, double ds) {
  std::vector<SLPoint> dp_path_points_dense; // ds1，60个。向前60m
  // //调试增加
  // dp_path_points_[0].s = 0;
  // dp_path_points_[0].l = 0;
  // dp_path_points_[0].dl_ds = 0;
  // dp_path_points_[0].ddl_ds = 0;
  // //调试增加
  int size = dp_path_points_.size();
  int dense_size = dp_path_points_.back().s / ds + 1;
  for (int i = 0; i < size - 1; i++) {
    if (dp_path_points_dense.size() < dense_size) {
      //对s进行采样,依次对每两点间隔ds进行采样
      SLPoint quintic_path_point;
      std::vector<double> qc;
      CalcQuinticCoeffient(dp_path_points_[i], dp_path_points_[i + 1], &qc);
      for (int j = 0; j < dense_size; j++) {
        quintic_path_point.s = dp_path_points_[i].s + ds * j;
        if (quintic_path_point.s < dp_path_points_[i + 1].s) {
          quintic_path_point.l = qc[0] + qc[1] * quintic_path_point.s +
                                 qc[2] * pow(quintic_path_point.s, 2) +
                                 qc[3] * pow(quintic_path_point.s, 3) +
                                 qc[4] * pow(quintic_path_point.s, 4) +
                                 qc[5] * pow(quintic_path_point.s, 5);
          quintic_path_point.dl_ds = qc[1] + 2 * qc[2] * quintic_path_point.s +
                                     3 * qc[3] * pow(quintic_path_point.s, 2) +
                                     4 * pow(quintic_path_point.s, 3) +
                                     5 * qc[5] * pow(quintic_path_point.s, 4);
          quintic_path_point.ddl_ds =
              2 * qc[2] + 6 * qc[3] * quintic_path_point.s +
              12 * qc[4] * pow(quintic_path_point.s, 2) +
              20 * qc[5] * pow(quintic_path_point.s, 3);
          dp_path_points_dense.push_back(quintic_path_point);
        } else
          break;
      }
    } else
      break;
  }
  dp_path_points_dense.push_back(dp_path_points_.back());
  dp_path_points_dense_ = dp_path_points_dense;
}

/*----------------------------------------路径动态规划结束------------------------------------------------------------------------------------*/

void PathTimeGraph::GenerateConvexSpace(double static_obs_length,
                                        double static_obs_width) {
  // 0.初始化l_min,l_max为道路边界
  // 1.找到每个障碍物的s_head,s_end
  // 2.找到dp_path_s 中，与s_head s_end最近的s的编号。
  //如果是向左绕行，则设置l_min
  //如果向右绕行，则设置l_max
  int size = dp_path_points_dense_.size();
  Eigen::VectorXd l_min = Eigen::VectorXd::Ones(size) * -8;
  Eigen::VectorXd l_max = Eigen::VectorXd::Ones(size) * 8;
  for (const auto &obs : sl_static_obstacles_) {
    double obs_s_min = obs.s - static_obs_length / 2;
    double obs_s_max = obs.s + static_obs_length / 2;

    double start_index = FindNearIndex(dp_path_points_dense_, obs_s_min);
    double end_index = FindNearIndex(dp_path_points_dense_, obs_s_max);

    double path_center_l = (dp_path_points_dense_[start_index].l +
                            dp_path_points_dense_[end_index].l) /
                           2;

    if (path_center_l > obs.l) { //左侧绕行
      for (int i = start_index; i <= end_index; i++) {
        l_min(i) = std::max(l_min(i), obs.l + static_obs_width / 2);
      }
    } else { //右侧绕行
      for (int i = start_index; i <= end_index; i++) {
        l_max(i) = std::min(l_max(i), obs.l - static_obs_width / 2);
      }
    }
  }

  l_min_ = l_min;
  l_max_ = l_max;
}

int PathTimeGraph::FindNearIndex(
    const std::vector<SLPoint> &dp_path_points_dense, double obs_s) {
  if (dp_path_points_dense.front().s > obs_s)
    return 0;
  if (dp_path_points_dense.back().s < obs_s)
    return dp_path_points_dense.size() - 1;
  double index = 0;
  for (index = 0; index < dp_path_points_dense.size(); index++) {
    if (dp_path_points_dense[index].s > obs_s)
      break;
  }

  if (dp_path_points_dense[index].s - obs_s >
      obs_s - dp_path_points_dense[index - 1].s)
    return index - 1;
  else
    return index;
}

bool PathTimeGraph::PathQuadraticProgramming() {
  /*
  % 0.5*x'Hx + f'*x = min
  % subject to A*x <= b 凸空间约束
  %            Aeq*x = beq 加加速度约束
  %            lb <= x <= ub;
  % 输入：l_min l_max 点的凸空间
  % w_cost_l 参考线代价
  % w_cost_dl ddl dddl 光滑性代价
  % w_cost_centre 凸空间中央代价
   % w_cost_end_l dl dd1 终点的状态代价(希望path的终点状态为(0, 0, 0))

   % host_d1, d2 host质心到前后轴的距离
   % host_w host的宽度
   % plan_start_l, dl, ddl 规划起点
   % 输出 qp_path_l dl ddl 二次规划输出的曲线
  */

  // H_L H_DL H_DDL H_DDDL Aeq beq A b 初始化

  int n = dp_path_points_dense_.size();
  double ds = dp_path_points_dense_[1].s - dp_path_points_dense_[0].s;
  double d1 = 3; //汽车中心到后前边的距离
  double d2 = 3;
  double w = 1.67;

  //输出初始化
  Eigen::VectorXd qp_path_l = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qp_path_dl = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qp_path_ddl = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qp_path_s = Eigen::VectorXd::Zero(n);

  // Hissen矩阵初始化
  Eigen::SparseMatrix<double> H(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_L(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DL(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDL(3 * n, 3 * n); // n-1,3n
  Eigen::SparseMatrix<double> H_DDDL(3 * n, 3 * n);

  Eigen::SparseMatrix<double> H_CENTRE(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_L_END(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DL_END(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDL_END(3 * n, 3 * n);

  // grident矩阵 f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(3 * n);

  //不等式约束  A*x <= b，dl ddl变化量约束
  Eigen::SparseMatrix<double> A(8 * n, 3 * n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(8 * n);

  //等式约束Aeq*x = beq,连续性约束
  Eigen::SparseMatrix<double> Aeq(2 * n - 2, 3 * n);
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(2 * n - 2);

  // lb ub初始化 lb <= x <= ub;
  Eigen::SparseMatrix<double> A_lu(3 * n, 3 * n);
  A_lu.setIdentity();
  Eigen::VectorXd lb = Eigen::VectorXd::Ones(3 * n) * -DBL_MAX;
  Eigen::VectorXd ub = Eigen::VectorXd::Ones(3 * n) * DBL_MAX;
  //行优先只能进行列操作
  Eigen::SparseMatrix<double> A_merge(8 * n + 2 * n - 2 + 3 * n, 3 * n);
  Eigen::VectorXd lb_merge(8 * n + 2 * n - 2 + 3 * n);
  Eigen::VectorXd ub_merge(8 * n + 2 * n - 2 + 3 * n);
  //---------------------------------------------H
  //生成H_L,H_DL,H_DDL,H_CENTRE
  // matlab矩阵索引从1开始，eigen矩阵索引从
  for (int i = 0; i < n; i++) {
    H_L.insert(3 * i, 3 * i) = 1;
    H_DL.insert(3 * i + 1, 3 * i + 1) = 1;
    H_DDL.insert(3 * i + 2, 3 * i + 2) = 1;
  }
  H_CENTRE = H_L;
  //生成H_DDDL
  Eigen::SparseMatrix<double> H_dddl_sub(1, 6);
  H_dddl_sub.insert(0, 2) = 1;
  H_dddl_sub.insert(0, 5) = -1;

  for (int i = 0; i < n - 1; i++) {
    int row = i;
    int col = 3 * i;
    //关于读操作，稀疏矩阵支持类似于密集矩阵相同的接口来访问块，列，行。
    //但由于性能的原因，稀疏子矩阵的写操作非常受限。当前仅列（列）优先稀疏矩阵的连续列（或行）可写。此外，在编译时必须知道这些信息。
    //  H_DDDL.block(row, col, 1, 6) = H_dddl_sub; //设置矩阵的块
    // H_DDDL.middleRows(row, col) = H_dddl_sub;
    H_DDDL.insert(row, col + 2) = 1;
    H_DDDL.insert(row, col + 5) = -1;
  }

  //% 生成H_L_END H_DL_END H_DDL_END
  H_L_END.insert(3 * n - 3, 3 * n - 3) = 1;
  H_DL_END.insert(3 * n - 2, 3 * n - 2) = 1;
  H_DDL_END.insert(3 * n - 1, 3 * n - 1) = 1;

  H = config_.qp_cost_l * (H_L.transpose() * H_L) +
      config_.qp_cost_dl * (H_DL.transpose() * H_DL) +
      config_.qp_cost_ddl * (H_DDL.transpose() * H_DDL) +
      config_.qp_cost_dddl * (H_DDDL.transpose() * H_DDDL) / ds +
      config_.qp_cost_centre * (H_CENTRE.transpose() * H_CENTRE) +
      config_.qp_cost_end_l * (H_L_END.transpose() * H_L_END) +
      config_.qp_cost_end_dl * (H_DL_END.transpose() * H_DL_END) +
      config_.qp_cost_end_ddl * (H_DDL_END.transpose() * H_DDL_END);
  H = 2 * H;

  //--------------------------------------生成f
  auto centre_line =
      0.5 * (l_min_ + l_max_); //此时centre line 还是60个点,起点不优化
  for (int i = 0; i < n; i++) {
    f(3 * i) = -2 * centre_line(i);
    //避免centreline权重过大影响轨迹平顺性
    if (abs(f(3 * i)) > 0.3)
      f(3 * i) = config_.qp_cost_centre * centre_line(i);
  }
  //% 期望的终点状态
  double end_l_desire = 0;
  double end_dl_desire = 0;
  double end_ddl_desire = 0;
  f(3 * n - 3) = f(3 * n - 3) - 2 * end_l_desire * config_.qp_cost_end_l;
  f(3 * n - 2) = f(3 * n - 2) - 2 * end_dl_desire * config_.qp_cost_end_dl;
  f(3 * n - 1) = f(3 * n - 1) - 2 * end_ddl_desire * config_.qp_cost_end_ddl;

  // Ax<=b
  //生成A 凸空间约束
  Eigen::MatrixXd A_sub(8, 3);

  A_sub << 1, d1, 0, 1, d1, 0, 1, -d2, 0, 1, -d2, 0, -1, -d1, 0, -1, -d1, 0, -1,
      d2, 0, -1, d2, 0;

  double row_index_start = 0;
  for (int i = 0; i < n - 1; i++) {
    int row = 8 * i;
    int col = 3 * i;
    //   A.block(row, col, 8, 3) = A_sub;
    A_merge.insert(row + 0, col + 0) = 1;
    A_merge.insert(row + 1, col + 0) = 1;
    A_merge.insert(row + 2, col + 0) = 1;
    A_merge.insert(row + 3, col + 0) = 1;
    A_merge.insert(row + 5, col + 0) = -1;
    A_merge.insert(row + 6, col + 0) = -1;
    A_merge.insert(row + 7, col + 0) = -1;
    A_merge.insert(row + 8, col + 0) = -1;

    A_merge.insert(row + 0, col + 1) = d1;
    A_merge.insert(row + 1, col + 1) = d1;
    A_merge.insert(row + 2, col + 1) = -d2;
    A_merge.insert(row + 3, col + 1) = -d2;
    A_merge.insert(row + 4, col + 1) = -d1;
    A_merge.insert(row + 5, col + 1) = -d1;
    A_merge.insert(row + 6, col + 1) = d2;
    A_merge.insert(row + 7, col + 1) = d2;
  }

  //生成b
  int front_index = ceil(d1 / ds);
  int back_index = ceil(d2 / ds);
  for (int i = 0; i < n; i++) {
    int index1 = std::min(i + front_index, n - 1); //车头索引
    int index2 = std::max(i - back_index, 0);      //车位索引
    Eigen::VectorXd b_sub(8);
    b_sub << l_max_(index1) - w / 2, l_max_(index1) + w / 2,
        l_max_(index2) - w / 2, l_max_(index2) + w / 2, -l_min_(index1) + w / 2,
        -l_min_(index1) - w / 2, -l_min_(index2) + w / 2,
        -l_min_(index2) - w / 2;
    b.block(8 * i, 0, 8, 1) = b_sub;
  }

  // Aeqx<=beq
  //生成Aeq,连续性约束
  Eigen::MatrixXd Aeq_sub(2, 6);
  Aeq_sub << 1, ds, pow(ds, 2) / 3, -1, 0, pow(ds, 2) / 6, 0, 1, ds / 2, 0, -1,
      ds / 2;

  row_index_start = 8 * n;
  for (int i = 0; i < n - 1; i++) {
    int row = row_index_start + 2 * i;
    int col = 3 * i;
    // Aeq.block(row, col, 2, 6) = A_sub;
    A_merge.insert(row, col) = 1;
    A_merge.insert(row, col + 1) = ds;
    A_merge.insert(row, col + 2) = pow(ds, 2) / 3;
    A_merge.insert(row, col + 3) = -1;
    A_merge.insert(row, col + 5) = pow(ds, 2) / 6;

    A_merge.insert(row + 1, col + 1) = 1;
    A_merge.insert(row + 1, col + 2) = ds / 2;
    A_merge.insert(row + 1, col + 4) = -1;
    A_merge.insert(row + 1, col + 5) = ds / 2;
  }

  // 3n*3n单位矩阵，起点约束，dl，ddl约束
  row_index_start = 8 * n + 2 * n - 2;
  for (int i = 0; i < 3 * n; i++) {
    A_merge.insert(row_index_start + i, i) = 1;
  }

  //% 生成 lb ub 主要是对规划起点做约束
  lb(0) = dp_path_points_dense_[0].l;
  lb(1) = dp_path_points_dense_[0].dl_ds;
  lb(2) = dp_path_points_dense_[0].ddl_ds;
  ub(0) = lb(0);
  ub(1) = lb(1);
  ub(2) = lb(2);

  for (int i = 1; i < n; i++) {
    lb(3 * i + 1) = -2;
    lb(3 * i + 2) = -0.1;
    ub(3 * i + 1) = 2;
    ub(3 * i + 2) = 0.1;
  }

  //汇总约束
  /*
                 -inf< A*x   <= b
  %              -inf< Aeq*x = beq
  %               lb <=  x   <= ub;

  A            b
  Aeq  * x =   beq
  1            ub


  Eigen::MatrixXd A_merge(8 * n + 2 * n - 2 + 3 * n, 3 * n);
  Eigen::MatrixXd lb_merge(8 * n + 2 * n - 2 + 3 * n, 1);
  Eigen::MatrixXd ub_merge(8 * n + 2 * n - 2 + 3 * n, 1);*/

  // A_merge.topRows(8 * n) = A;
  // A_merge.middleRows(8 * n + 1, 8 * n + 2 * n - 2) = Aeq;
  // A_merge.bottomRows(8 * n + 2 * n - 2) = A_lu;

  // A_merge_tp.leftCols(8 * n) = A.transpose();
  // // A_merge_tp.middleRows(8 * n + 1, 8 * n + 2 * n - 2) = Aeq.transpose();
  // A_merge_tp.rightCols(8 * n + 2 * n - 2 + 1) = A_lu.transpose();
  // A_merge = A_merge_tp.transpose();

  ub_merge.block(0, 0, 8 * n, 1) = b;
  ub_merge.block(8 * n, 0, 2 * n - 2, 1) = beq;
  ub_merge.block(8 * n + 2 * n - 2, 0, 3 * n, 1) = ub;

  lb_merge.block(0, 0, 8 * n, 1) = Eigen::MatrixXd::Ones(8 * n, 1) * (-DBL_MAX);
  lb_merge.block(8 * n, 0, 2 * n - 2, 1) =
      Eigen::MatrixXd::Ones(2 * n - 2, 1) * (-DBL_MAX);
  lb_merge.block(8 * n + 2 * n - 2, 0, 3 * n, 1) = lb;

  //-----------------------------------------
  // osqp的调用---------------------------------------------------------------
  OsqpEigen::Solver solver;

  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(3 * n); // A矩阵列数
  solver.data()->setNumberOfConstraints(8 * n + 2 * n - 2 + 3 * n); // A矩阵行数
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(f);
  solver.data()->setLinearConstraintsMatrix(A_merge);
  solver.data()->setLowerBound(lb_merge);
  solver.data()->setUpperBound(ub_merge);

  if (!solver.initSolver())
    return 0;
  if (!solver.solve())
    return 0;
  Eigen::VectorXd qp_solution(3 * n);
  qp_solution = solver.getSolution();
  qp_path_points_ = dp_path_points_dense_;
  for (int i = 0; i < n; i++) {
    qp_path_points_[i].l = qp_solution(3 * i);
    qp_path_points_[i].dl_ds = qp_solution(3 * i + 1);
    qp_path_points_[i].ddl_ds = qp_solution(3 * i + 2);
  }
}
void PathTimeGraph::QpPathInterpolation(int num, double ds) {
  std::vector<SLPoint> qp_path_points_dense;

  qp_path_points_dense[0] = qp_path_points_[0];
  int n_init = qp_path_points_.size();
  ds = qp_path_points_.back().s - qp_path_points_.front().s / (num - 1);
  int index = 0;
  for (int i = 1; i < num; i++) {
    double s = qp_path_points_.front().s + i * ds;
    qp_path_points_dense[i].s = s;
    while (s >= qp_path_points_[index].s) {
      index++;
      if (index == n_init)
        break;
    }
    //% while 循环退出的条件是x <
    // qp_path_s(index)，所以x对应的前一个s的编号是index - 1 后一个编号是index
    int pre = index - 1;
    int cur = index;
    double delta_s = s - qp_path_points_[pre].s;
    qp_path_points_dense[i].l =
        qp_path_points_[pre].l + qp_path_points_[pre].dl_ds * delta_s +
        (1 / 3) * qp_path_points_[pre].ddl_ds * pow(delta_s, 2) +
        (1 / 6) * qp_path_points_[cur].ddl_ds * pow(delta_s, 2);
    qp_path_points_dense[i].dl_ds =
        qp_path_points_[pre].dl_ds +
        0.5 * qp_path_points_[pre].ddl_ds * delta_s +
        0.5 * qp_path_points_[cur].ddl_ds * delta_s;
    qp_path_points_dense[i].ddl_ds =
        qp_path_points_[pre].ddl_ds +
        (qp_path_points_[cur].ddl_ds - qp_path_points_[pre].ddl_ds) * ds /
            (qp_path_points_[cur].s - qp_path_points_[pre].s);
    // % 因为此时x的后一个编号是index 必有x < qp_path_s(index), 在下一个循环中x
    // = x + ds
    //也未必大于 % qp_path_s(index)，这样就进入不了while循环，所以index
    //要回退一位
    index = index - 1;
  }
}

void PathTimeGraph::GeneratePlaningPath() // frenet
{
  std::vector<ReferencePoint> planning_path_points;
  for (int i = 0; i < qp_path_points_dense_.size(); i++) {
    ReferencePoint project_point;
    CalcProjPoint(qp_path_points_dense_[i], sl_reference_line_,
                  reference_line_.reference_points(), project_point);
    Eigen::Vector2d nor(-sin(project_point.heading),
                        cos(project_point.heading));
    Eigen::Vector2d vector_proj(project_point.x, project_point.y);
    Eigen::Vector2d point = vector_proj + qp_path_points_dense_[i].l * nor;
    double heading =
        project_point.heading +
        atan(qp_path_points_dense_[i].dl_ds /
             (1 - project_point.kappa * qp_path_points_dense_[i].l));
    //% 近似认为 kappa' == 0,frenet转cartesian公式，见第一章第三节评论区的链接
    double kappa = ((qp_path_points_dense_[i].dddl_ds +
                     project_point.kappa * qp_path_points_dense_[i].dl_ds *
                         tan(heading - project_point.heading))) *
                   (pow(cos(heading - project_point.heading), 2) /
                        (1 - project_point.kappa * qp_path_points_dense_[i].l) +
                    project_point.kappa) *
                   cos(heading - project_point.heading) /
                   (1 - project_point.kappa * qp_path_points_dense_[i].l);
    planning_path_points[i].x = point(1);
    planning_path_points[i].y = point(2);
    planning_path_points[i].heading = heading;
    planning_path_points[i].kappa = kappa;
  }
  planning_path_.set_reference_points(planning_path_points);
}

void PathTimeGraph::CalcProjPoint(
    const SLPoint sl_point, std::vector<SLPoint> sl_reference_line,
    const std::vector<ReferencePoint> reference_line,
    ReferencePoint &project_point) {
  //   该函数将计算在frenet坐标系下，点(s, l)
  //   在frenet坐标轴的投影的直角坐标(proj_x, proj_y, proj_heading, proj_kappa)'
  // 1.寻找匹配点
  int match_index = 0;
  while (sl_reference_line[match_index].s < sl_point.s) {
    match_index = match_index + 1;
  }
  Eigen::Vector2d match_point(reference_line[match_index].x,
                              reference_line[match_index].y);
  double match_point_heading = reference_line[match_index].heading;
  double match_point_kappa = reference_line[match_index].kappa;

  double ds = sl_point.s - sl_reference_line[match_index].s;
  Eigen::Vector2d match_tor(cos(match_point_heading), sin(match_point_heading));
  Eigen::Vector2d proj_point = match_point + ds * match_tor;
  double proj_heading = match_point_heading + ds * match_point_kappa;
  double proj_kappa = match_point_kappa;

  project_point.x = proj_point(1);
  project_point.y = proj_point(2);
  project_point.heading = proj_heading;
  project_point.kappa = proj_kappa;
}

const ReferenceLine PathTimeGraph::planning_path() const {

  return planning_path_;
}

const std::vector<SLPoint> PathTimeGraph::dp_path_points() const {
  return dp_path_points_;
}; //动态规划路径点

const std::vector<SLPoint> PathTimeGraph::dp_path_points_dense() const {
  return dp_path_points_dense_;

}; //动态规划加密路径点

const std::vector<SLPoint> PathTimeGraph::qp_path_points() const {
  return qp_path_points_;
}; //二次规划路径点

const std::vector<SLPoint> PathTimeGraph::qp_path_points_dense() const {
  return qp_path_points_dense_;

}; //二次规划加密路径点