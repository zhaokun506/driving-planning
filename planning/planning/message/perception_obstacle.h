#include <vector>

/*障碍物的数据类型*/
class ObstacleInfo : public TrajectoryPoint
{
public:
    int ID;
    double v;
    double heading;
};

class PerceptionObstacle
{
private:
    /* data */
    std::vector<ObstacleInfo> obstacles_;
    
public:
    PerceptionObstacle(/* args */);
    ~PerceptionObstacle();
    const std::vector<ObstacleInfo> obstacles() const;


    
};

PerceptionObstacle::PerceptionObstacle(/* args */)
{
}

PerceptionObstacle::~PerceptionObstacle()
{
}
