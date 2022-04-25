//
// Created by quantum on 4/8/21.
//

#ifndef LINESEGMENT2D_H
#define LINESEGMENT2D_H

#include "Eigen/Core"

class LineSegment2D
{
   public:
      LineSegment2D(const Eigen::Vector2f& point1, const Eigen::Vector2f& point2) : _firstPoint(point1), _secondPoint(point2) {}
      const Eigen::Vector2f& IntersectWith(const LineSegment2D& segment);

   private:
      Eigen::Vector2f _firstPoint;
      Eigen::Vector2f _secondPoint;
};

#endif //LINESEGMENT2D_H
