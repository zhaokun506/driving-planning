#pragma once
#include <vector>
#include "reference_line.h"

class TrajectoryPoint : public ReferencePoint
{
    /* data */
public:
    double v;  //速度
    double vx; //车身坐标系
    double vy;
    double a; //加速度
    double ax;
    double ay;
    double t; //时间
};

class Trajectory
{

public:
    Trajectory(/* args */);
    ~Trajectory();
    const std::vector<TrajectoryPoint> trajectory_points() const;
    void set_trajectory_points(const std::vector<TrajectoryPoint> &trajectory_points);

private:
    /* data */
    std::vector<TrajectoryPoint>
        trajectory_points_;
};

Trajectory::Trajectory(/* args */)
{
}

Trajectory::~Trajectory()
{
}
