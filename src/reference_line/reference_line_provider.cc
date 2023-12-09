#include "reference_line_provider.h"
ReferenceLineProvider::ReferenceLineProvider() {}

void ReferenceLineProvider::Provide(
    const std::vector<MapPoint> &routing_path_points,
    const LocalizationInfo &localzation_info,
    const ReferenceLine &pre_reference_line, ReferenceLine &reference_line)

{
  // 1.将全局路径转换为自然坐标系参考路径，增加heading和kappa信息
  RoutingPathToFrenetPath(routing_path_points, &frenet_path_);
  // 2.找到host在全局路径的匹配点和投影点
  MapPoint host_xy;
  std::vector<MapPoint> vct_host_xy;
  host_xy.x = localzation_info.x;
  host_xy.x = localzation_info.y;
  vct_host_xy.push_back(host_xy);
  std::vector<ReferencePoint> host_match_points;
  std::vector<ReferencePoint> host_project_points;
  if (pre_reference_line.reference_points().empty()) //首次运行
    FindMatchAndProjectPoint(
        frenet_path_, vct_host_xy, 0, 50, host_match_points,
        host_project_points); //找到车辆在全局路径的匹配匹配点
  else                        //非首次运行
  {
    int pre_match_point_index = pre_reference_line.match_point_index();
    FindMatchAndProjectPoint(frenet_path_, vct_host_xy, pre_match_point_index,
                             5, host_match_points, host_project_points);
  }

  frenet_path_.set_host_match_point(host_match_points.front());
  frenet_path_.set_host_project_point(host_project_points.front());

  //截取初始参考线
  GetReferenceLine(frenet_path_, reference_line.host_match_point().index,
                   raw_reference_line_);
  //参考线平滑
  ReferenceLineSmootherConfig smoother_config; //平滑配置
  std::unique_ptr<ReferenceLineSmoother> smoother =
      std::make_unique<ReferenceLineSmoother>(smoother_config);
  smoother->Smooth(raw_reference_line_, reference_line);
  smoothed_reference_line_ = reference_line;
}

// 1.寻找匹配点,计算匹配点在全局路径的投影.//此处与老王有所区别，将存储上一次的匹配点的功能放在Provider函数内，另外是首次运行由provide函数做判断
//公共函数，其他地方也可以调用，因此不能在其中操作其私有变量
void ReferenceLineProvider::FindMatchAndProjectPoint(
    const ReferenceLine &frenet_path, const std::vector<MapPoint> &map_points,
    const int index_start_search, const int increase_count_limit,
    std::vector<ReferencePoint> &match_points,
    std::vector<ReferencePoint> &project_points) {
  auto frenet_path_points = frenet_path.reference_points();
  int size = frenet_path_points.size();
  int increase_count = 0;
  match_points.resize(map_points.size());
  project_points.resize(map_points.size());

  for (int i = 0; i < map_points.size(); i++) {
    double min_distance = DBL_MAX;
    for (int j = index_start_search; j < size; j++) {
      double distance = pow(map_points[i].x - frenet_path_points[j].x, 2) +
                        pow(map_points[i].y - frenet_path_points[j].y, 2);
      if (distance < min_distance) {
        //如果距离小于最小距离，说明距离在递减
        min_distance = distance;

        match_points[i].index = j;

        increase_count = 0;
      } else {
        //如果距离增加，则计数+1。增加此部分为了避免在小半径连续转弯的情况，自车从前一时刻到当前时刻行走了具有两个极小值得道路，
        //而错误地将第一个极小值作为投影点，实际上是第二个才是投影点
        increase_count = increase_count + 1;
      }
      //如果distance连续增加50次就不要再遍历了，节省时间
      if (increase_count > increase_count_limit)
        break;
    }

    match_points[i].x = frenet_path_points[match_points[i].index].x;
    match_points[i].y = frenet_path_points[match_points[i].index].y;
    match_points[i].heading = frenet_path_points[match_points[i].index].heading;
    match_points[i].kappa = frenet_path_points[match_points[i].index].kappa;

    //根据匹配点求投影点

    // 计算匹配点的方向向量与法向量,此处更换为eigen vector2d
    std::pair<double, double> vector_match_point =
        std::make_pair(match_points[i].x, match_points[i].y);
    std::pair<double, double> vector_match_point_direction = std::make_pair(
        cos(match_points[i].heading), sin(match_points[i].heading));
    //声明待投影点的位矢
    std::pair<double, double> vector_r =
        std::make_pair(map_points[i].x, map_points[i].y);

    //通过匹配点计算投影点
    std::pair<double, double> vector_d =
        std::make_pair(vector_r.first - vector_match_point.first,
                       vector_r.second - vector_match_point.second);
    double ds =
        vector_d.first * vector_match_point_direction.first +
        vector_d.second * vector_match_point_direction.second; //向量点乘

    // vector_proj_point = vector_match_point + ds *
    // vector_match_point_direction;
    project_points[i].x =
        match_points[i].x + ds * vector_match_point_direction.first;
    project_points[i].y =
        match_points[i].y + ds * vector_match_point_direction.second;
    project_points[i].heading =
        match_points[i].heading + match_points[i].kappa * ds;
    project_points[i].kappa = match_points[i].kappa;
  }
}

// 2.根据自车位置的匹配点，截取参考线 首次截取调用参考线平滑器，进行参考线平滑
void ReferenceLineProvider::GetReferenceLine(
    const ReferenceLine &frenet_path,
    const int host_match_point_index, //车辆在全局path的索引
    ReferenceLine &raw_reference_line) {
  int start_index = -1;
  int len = frenet_path.reference_points().size();
  //% 匹配点后面的点太少了，不够30个
  int host_match_index = 0;
  if (host_match_point_index - 30 < 0) {
    start_index = 0;
    host_match_index = host_match_point_index;
  }
  //% 匹配点前面的点太少了，不够150个
  else if (host_match_point_index + 150 > len) {
    start_index = len - 180;
    host_match_index = 180 - host_match_point_index;
  }

  //正常情况
  else {
    start_index = host_match_point_index - 30;
    host_match_index = 30;
  }

  std::vector<ReferencePoint>::const_iterator it_start =
      frenet_path.reference_points().begin() + start_index;
  std::vector<ReferencePoint>::const_iterator it_end =
      frenet_path.reference_points().begin() + start_index + 180;
  std::vector<ReferencePoint> reference_points(it_start, it_end);

  raw_reference_line.set_reference_points(reference_points); // 0点发生变化
  raw_reference_line.set_match_point_index(host_match_point_index);

  //匹配点
  raw_reference_line_.set_host_match_point(frenet_path.host_match_point());
  //投影点
  raw_reference_line_.set_host_project_point(frenet_path.host_project_point());
}

// 4.非首次截取进行平滑轨迹的拼接

// 1.1 由全局路径转换参考线，即(x,y)->(x,y,heading,kappa)
/*
%该函数将计算path的切线方向与x轴的夹角和曲率
%输入：path_x,y路径坐标
%输出：path heading kappa 路径的heading和曲率
%原理 heading = arctan(dy/dx);
%     kappa = dheading/ds;
%     ds = (dx^2 + dy^2)^0.5
*/
// const对象只能访问const类型的成员
void ReferenceLineProvider::RoutingPathToFrenetPath(
    const std::vector<MapPoint> &routing_path_points,
    ReferenceLine *frenet_path) {
  auto points = routing_path_points;
  int size = points.size();
  /*此部分可以通过构造2d矢量类，来统一计算，见apollo common::math::Vec2D
  此处使用的是vector编程，后面再考虑使用eigen怎么操作
  */
  //前向差分，后向差分，差分中值
  //求heading
  std::vector<MapPoint> points_diff, points_diff_pre, points_diff_after,
      points_diff_fianl;
  std::vector<double> ds_final, points_heading;
  for (int i = 0; i < size - 1; i++) {
    MapPoint mp;
    mp.x = points[i + 1].x - points[i].x;
    mp.y = points[i + 1].y - points[i].y;
    points_diff.push_back(mp);
  }

  points_diff_pre = points_diff;
  points_diff_after = points_diff;
  points_diff_pre.insert(points_diff_pre.begin(), points_diff.front());
  points_diff_after.emplace_back(points_diff.back());

  for (int i = 0; i < size; i++) {
    MapPoint mp;
    mp.x = (points_diff_pre[i].x + points_diff_after[i].x) / 2;
    mp.y = (points_diff_pre[i].y + points_diff_after[i].y) / 2;
    points_diff_fianl.push_back(mp);

    ds_final.push_back(sqrt(pow(mp.x, 2) + pow(mp.y, 2)));

    points_heading.push_back(atan2(mp.y, mp.x));
  }

  //求kappa
  std::vector<double> heading_diff, heading_diff_pre, heading_diff_after,
      heading_diff_final, points_kappa;
  for (int i = 0; i < size - 1; i++) {
    heading_diff.push_back(points_heading[i + 1] - points_heading[i]);
  }
  heading_diff_pre = heading_diff;
  heading_diff_after = heading_diff;
  heading_diff_pre.insert(heading_diff_pre.begin(), heading_diff.front());
  heading_diff_after.emplace_back(heading_diff.back());

  for (int i = 0; i < size; i++) {
    heading_diff_final.push_back((heading_diff_pre[i] + heading_diff_after[i]) /
                                 2);
    //为了防止dheading出现多一个2pi的错误，假设真实的dheading较小，用sin(dheading)
    //近似dheading
    points_kappa.push_back(sin(heading_diff_final[i]) / ds_final[i]);
  }
  std::vector<ReferencePoint> reference_points;
  for (int i = 0; i < size; i++) {
    ReferencePoint rp;
    rp.x = points[i].x;
    rp.y = points[i].y;
    rp.index = i;
    rp.heading = points_heading[i];
    rp.kappa = points_kappa[i];
    reference_points.push_back(rp);
  }
  frenet_path->set_reference_points(reference_points);
}

const ReferenceLine ReferenceLineProvider::raw_reference_line() const {
  return raw_reference_line_;
}
const ReferenceLine ReferenceLineProvider::smoothed_reference_line() const {
  return smoothed_reference_line_;
}
