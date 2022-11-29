#include "EMPlanner/trajectory.h"
#include "plot/matplot/matplotlibcpp.h"
#include "reference_line/reference_line.h"
#include "routing/routing_path.h"

#include <map>
#include <vector>
namespace plt = matplotlibcpp;

class Plot {
public:
  Plot();
  ~Plot() = default;
  void PlotRoutingPath(std::vector<MapPoint> routing_path_points,
                       const std::string &color);
  void PlotReferenceLine(ReferenceLine reference_line,
                         const std::string &color);
  void PlotTrajetory(Trajectory trajectory, const std::string &color);

  void PlotSLPath(std::vector<SLPoint> sl_path_points,
                  const std::string &color);

  void PlotPlanningPath(std::vector<ReferencePoint> planning_path_points,
                        const std::string &color);

  void PlotObs(ObstacleInfo obs, const std::string &color);

  void PlotSTObs(std::vector<STLine> st_obstacles, const std::string &color);

  void PlotSTPath(std::vector<STPoint> dp_speed_points,
                  const std::string &color);
};