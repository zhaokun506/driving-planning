
class ReferenceLineSmootherConfig {
public:
  double weight_smooth =
      1; // weight_fem_pos_deviation; (x1+x3-2x2)^2+(y1+y3-2y2)^2
  double weight_path_length = 2;
  double weight_ref_deviation = 3;

  double x_lower_bound = -0.2;
  double x_upper_bound = 0.2;
  double y_lower_bound = -0.2;
  double y_upper_bound = 0.2;
};

/*
//     weight_fem_pos_deviation : 1e10;
// weight_ref_deviation : 1.0;
//  weight_path_length : 1.0 ;
//  apply_curvature_constraint : false ;
//  max_iter : 500;
//   time_limit : 0.0;
//    verbose : false scaled_termination : true warm_start : true private:
// double longitudinal_boundary_bound = 5;
double longitudinal_boundary_bound = 1.0;
double max_lateral_boundary_bound = 0.5;
double min_lateral_boundary_bound = 4;
*/