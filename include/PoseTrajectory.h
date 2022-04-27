//
// Created by quantum on 4/26/22.
//

#ifndef POSETRAJECTORY_H
#define POSETRAJECTORY_H

#include "TrajectoryOptimizer.h"
#include "RigidBodyTransform.h"

class PoseTrajectory
{
   public:
      PoseTrajectory() = default;
      void Optimize();
      void SetYawConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetPitchConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetRollConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetXConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetYConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);
      void SetZConditions(double startTime, double endTime, double start, double end, double startRate, double endRate);

      RigidBodyTransform GetPose(double time);
      Eigen::Vector3d GetAngles(double time);
      Eigen::Vector3d GetPosition(double time);


   private:
      TrajectoryOptimizer _yaw;
      TrajectoryOptimizer _pitch;
      TrajectoryOptimizer _roll;
      TrajectoryOptimizer _x;
      TrajectoryOptimizer _y;
      TrajectoryOptimizer _z;
};

#endif //POSETRAJECTORY_H
