#pragma once

class EMPlannerConfig {
private:
  /* data */
public:
  EMPlannerConfig(/* args */) = default;
  ~EMPlannerConfig() = default;
  double planning_cycle_time;

  double dp_cost_collision = 1.8; // default=10
  double dp_cost_dl = 120;        //
  double dp_cost_ddl = 1;
  double dp_cost_dddl = 1;
  double dp_cost_ref = 20; // default=100

  double dp_row = 1;
  double dp_col = 1;

  double qp_cost_l = 10;
  double qp_cost_dl = 1;
  double qp_cost_ddl = 1;
  double qp_cost_dddl = 1;
  double qp_cost_centre = 5;
  double qp_cost_end_l = 1;
  double qp_cost_end_dl = 1;
  double qp_cost_end_ddl = 1;

  double ref_speed = 30 / 3.6;
  double speed_dp_cost_ref_speed = 10;
  double speed_dp_cost_accel = 0;
  double speed_dp_cost_obs = 0;

  double speed_qp_cost_v_ref = 1;
  double speed_qp_cost_dds_dt = 1;
  double speed_qp_cost_jerk = 1;

  double max_lateral_accel = 1;
};
