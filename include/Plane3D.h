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
      Plane3D() = default;

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
      std::unordered_map<int, Plane3D> planes;
      int poseId = 0;
   public:
      int GetID() const { return poseId;}
      void SetID(int id) { poseId = id;}

      void InsertPlane(const Plane3D& plane, int id) {
         if (planes.find(id) == planes.end())
         {
            planes[id] = plane;
         }
      } // Insert if not present.

      const std::unordered_map<int, Plane3D>& GetPlanes() const { return planes;}

};

#endif //PLANE3D_H
