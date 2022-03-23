//
// Created by quantum on 4/8/21.
//

#ifndef PLANE3D_H
#define PLANE3D_H

#include "Eigen/Dense"
#include "RigidBodyTransform.h"
#include "vector"

class Plane3D
{
   public:
      Plane3D(double x, double y, double z, double w);

      Plane3D(double x, double y, double z, double w, int id);

      Plane3D(double px, double py, double pz, double nx, double ny, double nz, int id);

      const Eigen::Vector4d& GetParams() const {return _params;};

      void SetData(const Eigen::Vector4d& data) { _params = data;}

      int GetID() const {return _id + 1;}

      void SetID(int id) { _id = id;}

      std::string GetString();

      Plane3D GetTransformed(RigidBodyTransform& transform);

   private:
      Eigen::Vector4d _params;
      Eigen::Vector3d _origin;
      Eigen::Vector3d _normal;

      int _id = -1;
};

struct PlaneSet3D
{
   private:
      std::vector<Plane3D> planes;
      int poseId = 0;
   public:
      int GetID() const { return poseId;}
      void SetID(int id) { poseId = id;}

      std::vector<Plane3D>& GetPlanes() { return planes;}

};

#endif //PLANE3D_H
