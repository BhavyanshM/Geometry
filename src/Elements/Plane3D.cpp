//
// Created by quantum on 4/8/21.
//

#include "Plane3D.h"
#include <boost/format.hpp>

Plane3D::Plane3D(double x, double y, double z, double w)
{
   _params << x, y, z, w;
}

Plane3D::Plane3D(double x, double y, double z, double w, int id)
{
   _params << x, y, z, w;
   _id = id;
}

Plane3D::Plane3D(double px, double py, double pz, double nx, double ny, double nz, int id)
{
   _params << px, py, pz, -(px*nx + py*ny + pz*nz);
   _origin << px, py, pz;
   _normal << nx, ny, nz;
   _id = id;
}

std::string Plane3D::GetString()
{
   boost::format formatter("Plane(%.3f,%.3f,%.3f,%.3f)");
   formatter % _params.x() % _params.y() % _params.z() % _params.w();
   return formatter.str();
}

Plane3D Plane3D::GetTransformed(RigidBodyTransform& transform)
{
   Eigen::Vector3d origin = (transform.GetRotation() * _origin + transform.GetTranslation());
   Eigen::Vector3d normal = (transform.GetRotation() * _normal);
   return {origin.x(), origin.y(), origin.z(), normal.x(), normal.y(), normal.z(), _id};
}