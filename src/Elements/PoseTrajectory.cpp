//
// Created by quantum on 4/26/22.
//

#include "PoseTrajectory.h"

void PoseTrajectory::SetYawConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   _yaw.SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void PoseTrajectory::SetPitchConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   _pitch.SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void PoseTrajectory::SetRollConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   _roll.SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void PoseTrajectory::SetXConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   _x.SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void PoseTrajectory::SetYConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   _y.SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void PoseTrajectory::SetZConditions(double startTime, double endTime, double start, double end, double startRate, double endRate)
{
   _z.SetExtremalConditions(startTime, endTime, start, end, startRate, endRate);
}

void PoseTrajectory::Optimize()
{
   _yaw.Optimize();
   _pitch.Optimize();
   _roll.Optimize();
   _x.Optimize();
   _y.Optimize();
   _z.Optimize();
}

RigidBodyTransform PoseTrajectory::GetPose(double time)
{
   RigidBodyTransform pose;
   pose.SetAnglesAndTranslation({_roll.GetValue(time), _pitch.GetValue(time), _yaw.GetValue(time)},{_x.GetValue(time), _y.GetValue(time), _z.GetValue(time)});
   return pose;
}

Eigen::Vector3d PoseTrajectory::GetAngles(double time)
{
   Eigen::Vector3d angles;
   angles << _roll.GetValue(time), _pitch.GetValue(time), _yaw.GetValue(time);
   return angles;
}

Eigen::Vector3d PoseTrajectory::GetPosition(double time)
{
   Eigen::Vector3d position;
   position << _x.GetValue(time), _y.GetValue(time), _z.GetValue(time);
   return position;
}