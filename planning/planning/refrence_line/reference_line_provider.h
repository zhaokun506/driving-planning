/*此类的功能主要为EMPlanner提供参考线*/
#include <math.h>
#include <float.h>
#include "routing_path.h"
#include "localization_estimate.h"
#include "reference_line.h"

class ReferenceLineProvider
{
public:
    ReferenceLineProvider() = default;
    ~ReferenceLineProvider();

    //类的主功能函数，由全局路径，定位信息，上一时刻的参考线信息，生成新的参考线
    void Provide(const RoutingPath &routing_path,
                 const LocalizationEstimate &localzation,
                 const ReferenceLine &pre_reference_line,
                 ReferenceLine *reference_line);

    // 1.给定一些仅包含x,y信息的点，寻找在全局路径的匹配点和投影点信息(其他地方可以用到此函数，不能改变自己的子类)☆☆☆☆☆☆☆？？有没有必要使用公共类归纳
    static void FindMatchAndProjectPoint(const ReferenceLine &frenet_path, const std::vector<MapPoint> &map_points,
                                         const int index_start_search, const int increase_count_limit,
                                         std::vector<ReferencePoint> &match_points, std::vector<ReferencePoint> &project_points);
    // 2.根据匹配点，截取参考线
private:
    // 3.首次截取调用参考线平滑器，进行参考线平滑
    void GetReferenceLine(const ReferenceLine &frenet_path, const int host_match_point_index, ReferenceLine *reference_line);

    // 4.非首次截取进行平滑轨迹的拼接

    // 1.1 由全局路径转换参考线，即(x,y)->(x,y,heading,kappa)
    void RoutingPathToFrenetPath(const RoutingPath &routing_path, ReferenceLine *frenet_path);

    std::vector<MapPoint> pre_points_; //用于存储上一时刻，传入该模块的待处理点
    bool is_first_run_ = false;
    ReferenceLine raw_reference_line_;      //传入的原始参考线
    ReferenceLine smoothed_reference_line_; //平滑后的参考线
    ReferencePoint host_project_point;
    ReferencePoint host_match_point;
};
