#include "matplot/plot.h"

namespace plt = matplotlibcpp;

void Plot::PlotRoutingPath(std::vector<MapPoint> routing_path_points,
                           const std::string &color) {
  std::vector<double> x, y;
  for (const auto point : routing_path_points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  plt::plot(x, y, color);
}
void Plot::PlotReferenceLine(ReferenceLine reference_line,
                             const std::string &color) {
  std::vector<double> x, y;
  for (const auto point : reference_line.reference_points()) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  plt::plot(x, y, color);
}
void Plot::PlotTrajetory(Trajectory trajectory, const std::string &color) {

  std::vector<double> x, y;
  for (const auto point : trajectory.trajectory_points()) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  plt::plot(x, y, color);
}