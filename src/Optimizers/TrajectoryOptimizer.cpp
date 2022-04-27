//
// Created by quantum on 4/25/22.
//

#include "TrajectoryOptimizer.h"
#include "Eigen/Dense"

TrajectoryOptimizer::TrajectoryOptimizer(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void TrajectoryOptimizer::Optimize()
{
   _x = _A.colPivHouseholderQr().solve(_b);
}

void TrajectoryOptimizer::SetExtremalConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   double t = startTime;
   double T = endTime;
   double t_sq = t*t;
   double T_sq = T*T;
   double t_cu = t*t_sq;
   double T_cu = T*T_sq;

   _A << 1,    t,    t_sq,    t_cu,
         0,    1,    2*t,     3*t_sq,
         1,    T,    T_sq,    T_cu,
         0,    1,    2*T,     3*T_sq;

   _b << start, startRate, end, endRate;
}

double TrajectoryOptimizer::GetValue(double t) const
{
   Eigen::Vector4d timeVec;
   double t_sq = t * t;
   double t_cu = t * t_sq;
   timeVec << 1, t, t_sq, t_cu;

   return _x.dot(timeVec);
}

double TrajectoryOptimizer::GetRate(double t) const
{
   Eigen::Vector4d timeVec;
   double t_sq = t * t;
   timeVec << 0, 1, 2*t, 3*t_sq;

   return _x.dot(timeVec);
}
