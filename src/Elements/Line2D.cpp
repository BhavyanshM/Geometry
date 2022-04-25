//
// Created by quantum on 4/24/22.
//

#include "Line2D.h"

Line2D::Line2D(const Eigen::Vector2f& point, float slope)
{
   float c = point.y() - slope * point.x();
   _data(0) = -slope;
   _data(1) = 1;
   _data(2) = c;
   _data /= _data(2);
}

Line2D::Line2D(float slope, float intercept)
{
   _data(0) = -slope;
   _data(1) = 1;
   _data(2) = intercept;
   _data /= _data(2);
}

Line2D::Line2D(const Eigen::Vector2f& point1, const Eigen::Vector2f& point2)
{
   float slope = (point1.y() - point2.y())/(point1.x() - point2.x());
   float c = point1.y() - slope * point1.x();
   _data(0) = -slope;
   _data(1) = 1;
   _data(2) = c;
   _data /= _data(2);
}

const Eigen::Vector2f& Line2D::IntersectWith(const Line2D& line)
{
   /* x = (b1c2 - b2c1)/(a1b2 - a2b1)
    * y = (a1c2 - a2c1)/(a2b1 - a1b2)
    * */

   Eigen::Vector2f intersection;
   intersection.x() = (_data(1) * line(2) - line(1) * _data(2)) / (_data(0) * line(1) - line(0) * _data(1));
   intersection.y() = (_data(0) * line(2) - line(0) * _data(2)) / (line(0) * _data(1) - _data(0) * line(1));
   return std::move(intersection);
}
