#include "reference_line.h"

void ReferenceLine::set_reference_points(
    std::vector<ReferencePoint> reference_points) {
  // vector是一个构造对象，不能直接使用=符号进行复制，必须迭代每个元素来复制。或者重载=操作符。
  //   for (int i = 0; i < reference_points.size(); i++)
  //     reference_points_.push_back(reference_points[i]);
  reference_points_ = reference_points;
}

const std::vector<ReferencePoint> ReferenceLine::reference_points() const {
  return reference_points_;
}

void ReferenceLine::set_match_point_index(int index) {
  match_point_index_ = index;
}

void ReferenceLine::set_host_project_point(ReferencePoint host_project_point) {
  host_project_point_ = host_project_point;
}
void ReferenceLine::set_host_match_point(ReferencePoint host_match_point) {

  host_match_point_ = host_match_point;
}

const int ReferenceLine::match_point_index() const {
  return match_point_index_;
}

const ReferencePoint ReferenceLine::host_project_point() const {
  return host_project_point_;
}
const ReferencePoint ReferenceLine::host_match_point() const {
  return host_match_point_;
}