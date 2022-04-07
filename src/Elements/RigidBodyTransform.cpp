//
// Created by quantum on 4/2/21.
//

#include "RigidBodyTransform.h"
#include <utility>
#include <iostream>

RigidBodyTransform::RigidBodyTransform()
{
   this->matrix = Eigen::Matrix4d::Identity();
}

RigidBodyTransform::RigidBodyTransform(const RigidBodyTransform& transform) : matrix(transform.matrix)
{
   _id = transform.GetID();
}

RigidBodyTransform::RigidBodyTransform(RigidBodyTransform&& transform) : matrix(std::move(transform.matrix))
{
   _id = transform.GetID();
}

RigidBodyTransform::RigidBodyTransform(Eigen::Matrix4d mat)
{
   this->matrix = std::move(mat);
}

void RigidBodyTransform::setToInverse()
{
   this->matrix.block<3, 1>(0, 3) = -this->matrix.block<3, 3>(0, 0).transpose() * this->matrix.block<3, 1>(0, 3);
   this->matrix.block<3, 3>(0, 0) = this->matrix.block<3, 3>(0, 0).transpose();
}

RigidBodyTransform RigidBodyTransform::GetInverse()
{
   RigidBodyTransform transformToPack;
   transformToPack.matrix.block<3, 3>(0, 0) = this->matrix.block<3, 3>(0, 0).transpose();
   transformToPack.matrix.block<3, 1>(0, 3) = -this->matrix.block<3, 3>(0, 0).transpose() * this->matrix.block<3, 1>(0, 3);
   return transformToPack;
}

RigidBodyTransform::RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation)
{

   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) eulerAngles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) eulerAngles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) eulerAngles.z(), Eigen::Vector3d::UnitZ());
   this->matrix.setIdentity();
   this->matrix.block<3, 3>(0, 0) = rotation;
   this->matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::SetAnglesAndTranslation(const Eigen::Vector3d& angles, const Eigen::Vector3d& translation)
{
   Eigen::Matrix3d rotation;
   rotation = Eigen::AngleAxisd((double) angles.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd((double) angles.y(), Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd((double) angles.z(), Eigen::Vector3d::UnitZ());
   this->matrix.setIdentity();
   this->matrix.block<3, 3>(0, 0) = rotation;
   this->matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::SetQuaternionAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation)
{
   this->matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
   this->matrix.block<3, 1>(0, 3) = translation;
}

void RigidBodyTransform::SetAnglesAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
   this->matrix.setIdentity();
   this->matrix.block<3, 3>(0, 0) = rotation;
   this->matrix.block<3, 1>(0, 3) = translation;
}

const Eigen::Matrix4d& RigidBodyTransform::GetMatrix() const
{
   return matrix;
}

void RigidBodyTransform::SetMatrix(const Eigen::Matrix4d& matrix)
{
   this->matrix = matrix;
}

void RigidBodyTransform::MultiplyLeft(const RigidBodyTransform& transform)
{
   this->matrix = transform.matrix * this->matrix;
}

void RigidBodyTransform::MultiplyRight(const RigidBodyTransform& transform)
{
   this->matrix = this->matrix * transform.matrix;
}

Eigen::Vector3d RigidBodyTransform::transformVector(const Eigen::Vector3d& vector)
{
   return this->matrix.block<3, 3>(0, 0) * vector + this->matrix.block<3, 1>(0, 3);
}

void RigidBodyTransform::print()
{
   std::cout << matrix << std::endl;
}

Eigen::Vector3d RigidBodyTransform::GetTranslation()
{
   return this->matrix.block<3, 1>(0, 3);
}

Eigen::Quaterniond RigidBodyTransform::GetQuaternion()
{
   return Eigen::Quaterniond(this->matrix.block<3, 3>(0, 0));
}

Eigen::Matrix3d RigidBodyTransform::GetRotation()
{
   return this->matrix.block<3, 3>(0, 0);
}

void RigidBodyTransform::rotateX(float rad)
{
   this->rotate(rad, Eigen::Vector3d::UnitX());
}

void RigidBodyTransform::rotateY(float rad)
{
   this->rotate(rad, Eigen::Vector3d::UnitY());
}

void RigidBodyTransform::rotateZ(float rad)
{
   this->rotate(rad, Eigen::Vector3d::UnitZ());
}

void RigidBodyTransform::rotate(float rad, Eigen::Vector3d axis)
{
   Eigen::Matrix4d rotation = Eigen::Matrix4d::Identity();
   rotation.block<3,3>(0,0) = Eigen::AngleAxisd(rad, axis).toRotationMatrix();
   this->matrix =  rotation * this->matrix ;
}
