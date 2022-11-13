#include "EMPlanner/trajectory.h"
#include "matplot/matplotlibcpp.h"
#include "reference_line/reference_line.h"
#include "routing/routing_path.h"

#include <map>
#include <vector>
using namespace std;
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
};