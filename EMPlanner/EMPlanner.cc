#include "EMPlanner.h"
#include <math.h>

EMPlanner::EMPlanner(const EMPlannerConfig &conf) : config_(conf) {}

void EMPlanner::Plan(const u_int64_t current_time,
                     const TrajectoryPoint &planning_init_point,
                     const ReferenceLine &reference_line,
                     const LocalizationInfo &localization_info,
                     const std::vector<ObstacleInfo> &static_obstacles,
                     const std::vector<ObstacleInfo> &dynamic_obstacles,
                     Trajectory *trajectory,
                     std::vector<ObstacleInfo> xy_virtual_obstacles) {

  sl_graph_ = std::make_unique<PathTimeGraph>(reference_line, config_);

  sl_graph_->SetStartPointSl(planning_init_point);
  sl_graph_->SetStaticObstaclesSl(static_obstacles);

  sl_graph_->CreateSamplingPoint(11, 6, 10, 1); // 11行，6列
  sl_graph_->PathDynamicPlanning();
  sl_graph_->DpPathInterpolation(60, 1); // 61个点，间隔1m
  sl_graph_->GenerateConvexSpace(4, 2);  //静态障碍物的长和宽
  sl_graph_->PathQuadraticProgramming();
  // //增密至501个点 ??path不是加密到400个点了吗？
  sl_graph_->QpPathInterpolation(601, 0.1);
  sl_graph_->GeneratePlaningPath();

  ReferenceLine plannig_path = sl_graph_->planning_path();

  st_graph_ = std::make_unique<SpeedTimeGraph>(plannig_path, config_);
  st_graph_->SetStartState(sl_graph_->sl_plan_start());
  st_graph_->SetDynamicObstaclesSL(dynamic_obstacles);
  st_graph_->GenerateSTGraph();
  st_graph_->CreateSmaplePoint(40, 16);
  st_graph_->SpeedDynamicPlanning();
  st_graph_->GenerateCovexSpace();
  st_graph_->SpeedQuadraticProgramming();
  // st_graph_->SpeedQpInterpolation(601);

  // //路径和速度合并
  // // st_graph->PathAndSpeedMerge();
  // *trajectory = st_graph_->trajectory();
  // xy_virtual_obstacles = st_graph_->xy_virtual_obstacles();
}

/*
  1.  该函数将计算规划的起点以及拼接轨迹的信息
%输入 上一周期的规划结果信息pre_trajectory x
,y,heading,kappa,velocity,accel,time %     本周期的绝对时间current_time %
定位信息host x,y,heading,kappa,vx,vy,ax,ay %输出 规划起点信息plan_start
x,y,heading,kappa,vx,vy,ax,ay(vx,vy,ax,ay)是世界坐标系的

%
待拼接的轨迹信息，一般在上一周期的轨迹中的current_time+100ms，往前取20个点。??这20个点在什么位置
%     当规划完成后，本周期的规划结果和stitch_trajectory一起拼好发给控制
*/

void EMPlanner::CalPlaningStartPoint(const Trajectory &pre_traj,
                                     const LocalizationInfo &local_info,
                                     TrajectoryPoint *plan_start_point,
                                     Trajectory *stitch_traj) {
  double x_cur = local_info.x;
  double y_cur = local_info.y;
  double heading_cur = local_info.heading;
  double kappa_cur = 0;
  //车身坐标系转换为世界坐标系
  double vx_cur =
      local_info.vx * cos(heading_cur) + local_info.vy * sin(heading_cur);
  double vy_cur =
      local_info.vx * sin(heading_cur) + local_info.vy * cos(heading_cur);
  double ax_cur =
      local_info.ax * cos(heading_cur) + local_info.ay * sin(heading_cur);
  double ay_cur =
      local_info.ax * sin(heading_cur) + local_info.ay * cos(heading_cur);
  //  %由于不是首次运行了，有上一时刻的轨迹了，找到上一周期轨迹规划的本周期车辆应该在的位置
  double cur_time = local_info.t; //当前时间等于定位信息的时间

  if (pre_traj.trajectory_points().empty()) {
    //如果是首次运行
    plan_start_point->x = local_info.x;
    plan_start_point->y = local_info.y;
    plan_start_point->heading = local_info.heading;
    plan_start_point->kappa = local_info.kappa;
    plan_start_point->v = local_info.v;
    plan_start_point->vx = local_info.vx;
    plan_start_point->vy = local_info.vy;
    plan_start_point->a = local_info.a;
    plan_start_point->ax = local_info.ax;
    plan_start_point->ay = local_info.ay;
    plan_start_point->t = local_info.t;

  } else {
    auto pre_traj_points = pre_traj.trajectory_points();
    int pre_index_desire = 0;
    for (int i = 0; i < pre_traj_points.size() - 1; i++) {
      if (cur_time >= pre_traj_points[i].t &&
          cur_time < pre_traj_points[i + 1].t) {
        pre_index_desire = i;
        break;
      }
    }
    //% 上一周期规划的本周期的车应该在的位置
    double pre_x_desire = pre_traj_points[pre_index_desire].x;
    double pre_y_desire = pre_traj_points[pre_index_desire].y;
    double pre_heading_desire = pre_traj_points[pre_index_desire].heading;
    //计算横纵向误差
    // 切向量
    auto tor = std::make_pair(cos(pre_heading_desire), sin(pre_heading_desire));
    //法向量
    auto nor =
        std::make_pair(-sin(pre_heading_desire), cos(pre_heading_desire));
    //% 误差向量
    auto d_err = std::make_pair(local_info.x - pre_x_desire,
                                local_info.y - pre_y_desire);
    //%纵向误差
    double lon_err = abs(d_err.first * tor.first + d_err.second * tor.second);
    //%横向误差
    double lat_err = abs(d_err.first * nor.first + d_err.second * nor.second);
    //%纵向误差大于2.5 横向误差大于0.5 认为控制没跟上
    double dt = 0.1; //时间间隔为0.1s一个点
    if ((lon_err > 2.5) || (lat_err > 0.5)) {
      // %此分支处理控制未跟上的情况，规划起点通过运动学递推，cur_time+0.1(下一时刻的车辆位置)
      //定位模块传递的速度是以车身为坐标系的，轨迹传递的速度是以世界为坐标系
      plan_start_point->x = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
      plan_start_point->y = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
      plan_start_point->vx = vx_cur + ax_cur * dt;
      plan_start_point->vy = vy_cur + ay_cur * dt;
      plan_start_point->heading =
          atan2(plan_start_point->vy, plan_start_point->vx);
      plan_start_point->ax = ax_cur;
      plan_start_point->ay = ay_cur;
      plan_start_point->kappa = kappa_cur;
      plan_start_point->t = cur_time + dt;
    } else {
      //找到cur+dt对应的轨迹点
      for (int i = 0; i < pre_traj_points.size() - 1; i++) {
        if (cur_time + dt >= pre_traj_points[i].t &&
            cur_time + dt < pre_traj_points[i + 1].t) {
          pre_index_desire = i;
          break;
        }
      }
      plan_start_point->x = pre_traj_points[pre_index_desire].x;
      plan_start_point->y = pre_traj_points[pre_index_desire].y;
      plan_start_point->heading = pre_traj_points[pre_index_desire].heading;
      plan_start_point->kappa = pre_traj_points[pre_index_desire].kappa;

      plan_start_point->vx =
          pre_traj_points[pre_index_desire].v * cos(plan_start_point->heading);
      plan_start_point->vy =
          pre_traj_points[pre_index_desire].v * sin(plan_start_point->heading);
      plan_start_point->heading =
          atan2(plan_start_point->vy, plan_start_point->vx);

      // 切向量
      tor = std::make_pair(cos(plan_start_point->heading),
                           sin(plan_start_point->heading));
      //法向量
      nor = std::make_pair(-sin(plan_start_point->heading),
                           cos(plan_start_point->heading));
      auto a_tor =
          std::make_pair(pre_traj_points[pre_index_desire].a * tor.first,
                         pre_traj_points[pre_index_desire].a * tor.second);
      auto a_nor = std::make_pair(
          pow(pre_traj_points[pre_index_desire].v, 2) *
              pre_traj_points[pre_index_desire].kappa * nor.first,
          pow(pre_traj_points[pre_index_desire].v, 2) *
              pre_traj_points[pre_index_desire].kappa * nor.second);
      plan_start_point->ax = a_nor.first + a_nor.first;
      plan_start_point->ay = a_nor.second + a_nor.second;

      plan_start_point->t = pre_traj_points[pre_index_desire].t;
      // cur_time+dt
      /*
        %计算拼接轨
        %j 表示 current_time + 0.1 的时间点在 pre_trajectory_time的编号
       %拼接是往从j开始，往前拼接20个，也就是pre_trajectory(j-1),j-2,j-3,....j-19
       %分两种情况，如果j小于20，意味着前面的轨迹点不够20个
        %如果j >= 20,意味着前面的点多于20个
         %还有一个细节需要考虑，pre_trajectory x(j) y(j)
       ....是规划的起点，那么在轨迹拼接时
       %需不需要在stitch_trajectory中把pre_trajectory x(j) y(j) ....包含进去
       %答案是否定的，不应该包进去，因为规划的起点会在规划模块计算完成后的结果中包含，如果在拼接的时候
       %还要包含，那就等于规划起点包含了两次
       %除非本周期计算的轨迹不包含规划起点，那么stitch_trajectory可以包含规划起点。
       %如果本周期计算的轨迹包含规划起点，那么stitch_trajectory就不可以包含规划起点。
       %我们选择规划包含起点，拼接不包含起点的做法
      */
      pre_index_desire--;
      if (pre_index_desire >= 20) {
        auto it_start = pre_traj_points.begin() + pre_index_desire - 19;
        auto it_end = pre_traj_points.begin() + pre_index_desire;
        std::vector<TrajectoryPoint> stitch_traj_points(it_start, it_end);
        stitch_traj->set_trajectory_points(stitch_traj_points);
      } else {
        stitch_traj->set_trajectory_points(pre_traj_points);
      }
    }
  }
}

// 2.
void EMPlanner::StitchTrajectory(const Trajectory &cur_traj,
                                 const Trajectory &stitch_traj,
                                 Trajectory &final_traj) {
  std::vector<TrajectoryPoint> cur_traj_points = cur_traj.trajectory_points();
  std::vector<TrajectoryPoint> stitch_traj_points =
      stitch_traj.trajectory_points();
  auto final_traj_points = stitch_traj_points.insert(
      stitch_traj_points.end(), cur_traj_points.begin(), cur_traj_points.end());
  final_traj.set_trajectory_points(stitch_traj_points);
}

// const std::unique_ptr<PathTimeGraph> EMPlanner::sl_graph()  {
//   return sl_graph_;
// };
// const std::unique_ptr<SpeedTimeGraph> EMPlanner::st_graph() const {
//   return st_graph_;
// };