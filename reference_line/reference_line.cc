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

const int ReferenceLine::match_point_index() const {
  return match_point_index_;
}
void ReferenceLine::set_match_point_index(int index) {
  match_point_index_ = index;
}