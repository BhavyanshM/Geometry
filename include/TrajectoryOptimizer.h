//
// Created by quantum on 4/25/22.
//

#ifndef TRAJECTORYOPTIMIZER_H
#define TRAJECTORYOPTIMIZER_H

#include "Eigen/Core"

class TrajectoryOptimizer
{
   public:
      TrajectoryOptimizer() = default;
      TrajectoryOptimizer(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void Optimize();
      void SetExtremalConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);

      double GetRate(double t) const;
      double GetValue(double t) const;


   private:
      Eigen::Matrix4d _A;
      Eigen::Vector4d _x;
      Eigen::Vector4d _b;
};

#endif //TRAJECTORYOPTIMIZER_H
