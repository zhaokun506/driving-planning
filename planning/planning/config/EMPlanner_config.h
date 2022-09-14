class EMPlannerConfig
{
private:
    /* data */
public:
    EMPlannerConfig(/* args */);
    ~EMPlannerConfig();
    double planning_cycle_time;

    double dp_cost_collision;
    double dp_cost_dl;
    double dp_cost_ddl;
    double dp_cost_dddl;
    double dp_cost_ref;

    double dp_row;
    double dp_col;

    double qp_cost_l;
    double qp_cost_dl;
    double qp_cost_ddl;
    double qp_cost_dddl;
    double qp_cost_centre;
    double qp_cost_end_l;
    double qp_cost_end_dl;
    double qp_cost_end_ddl;

    double ref_speed;
    double speed_dp_cost_ref_speed;
    double speed_dp_cost_accel;
    double speed_dp_cost_obs;

    double speed_qp_cost_v_ref;
    double speed_qp_cost_dds_dt;
    double speed_qp_cost_jerk;

    double max_lateral_accel;
 };
