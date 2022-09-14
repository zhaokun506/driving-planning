/*
    位置轨迹的数据类型
*/
#include "trajectory.h"

class LocalizationInfo : public TrajectoryPoint
{
public:
    double
};

class LocalizationEstimate
{
private:
    /* data */
    LocalizationInfo localization_info_;

public:
    LocalizationEstimate(/* args */);
    ~LocalizationEstimate();

    const LocalizationInfo localization_info() const;
};

LocalizationEstimate::LocalizationEstimate(/* args */)
{
}

LocalizationEstimate::~LocalizationEstimate()
{
}


