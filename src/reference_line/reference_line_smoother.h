/*
此类使用Apollo的discrete_points_smoother的FEM_POS_DEVIATION_SMOOTHING
*/
#include "config/reference_line_smoother_config.h"
#include "reference_line/reference_line.h"

class ReferenceLineSmoother {
public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig &config);
  ~ReferenceLineSmoother() = default;

  // 1.平滑

  void Smooth(const ReferenceLine &raw_reference_line,
              ReferenceLine &smoothed_reference_line);

private:
  ReferenceLineSmootherConfig config_;
  //对2d曲线进行平滑处理
  bool DiscretePointsSmooth(
      const std::vector<std::pair<double, double>> &raw_point2d,
      std::vector<std::pair<double, double>> *ptr_smoothed_point2d);
};

// double weight_smooth; // weight_fem_pos_deviation;
// (x1+x3-2x2)^2+(y1+y3-2y2)^2;平滑代价 double weight_path_length;// double
// weight_ref_deviation;