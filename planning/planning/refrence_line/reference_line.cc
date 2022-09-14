#include "reference_line.h"

void ReferenceLine::set_reference_points(std::vector<ReferencePoint> reference_points)
{
    reference_points_ = reference_points;
}

const std::vector<ReferencePoint> ReferenceLine::reference_points() const
{
    return reference_points_;
}
