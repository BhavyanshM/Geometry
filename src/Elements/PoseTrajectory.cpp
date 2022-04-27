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
