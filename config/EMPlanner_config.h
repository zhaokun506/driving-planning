#pragma once

class EMPlannerConfig {
private:
  /* data */
public:
  EMPlannerConfig(/* args */);
  ~EMPlannerConfig();
  double planning_cycle_time;

  double dp_cost_collision = 1;
  double dp_cost_dl = 1;
  double dp_cost_ddl = 1;
  double dp_cost_dddl = 1;
  double dp_cost_ref = 1;

  double dp_row = 1;
  double dp_col = 1;

  double qp_cost_l = 1;
  double qp_cost_dl = 1;
  double qp_cost_ddl = 1;
  double qp_cost_dddl = 1;
  double qp_cost_centre = 1;
  double qp_cost_end_l = 1;
  double qp_cost_end_dl = 1;
  double qp_cost_end_ddl = 1;

  double ref_speed = 30;
  double speed_dp_cost_ref_speed = 1;
  double speed_dp_cost_accel = 1;
  double speed_dp_cost_obs = 1;

  double speed_qp_cost_v_ref = 1;
  double speed_qp_cost_dds_dt = 1;
  double speed_qp_cost_jerk = 1;

  double max_lateral_accel = 1;
};
