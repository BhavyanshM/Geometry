//
// Created by quantum on 4/26/22.
//

#ifndef POSETRAJECTORY_H
#define POSETRAJECTORY_H

#include "TrajectoryOptimizer.h"


class PoseTrajectory
{
   public:
      void SetYawConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetPitchConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetRollConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetXConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetYConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetZConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);

   private:
      TrajectoryOptimizer _yaw;
      TrajectoryOptimizer _pitch;
      TrajectoryOptimizer _roll;
      TrajectoryOptimizer _x;
      TrajectoryOptimizer _y;
      TrajectoryOptimizer _z;
};

#endif //POSETRAJECTORY_H
