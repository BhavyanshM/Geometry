//
// Created by quantum on 4/2/21.
//

#ifndef RIGIDBODYTRANSFORM_H
#define RIGIDBODYTRANSFORM_H


#include "Eigen/Dense"

class RigidBodyTransform
{
   private:
      Eigen::Matrix4d matrix;

      int _id;

   public:
      RigidBodyTransform();

      RigidBodyTransform(const RigidBodyTransform& transform);

      RigidBodyTransform(RigidBodyTransform&& transform);

      RigidBodyTransform(Eigen::Matrix4d matrix);

      RigidBodyTransform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      RigidBodyTransform(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void setToInverse();

      RigidBodyTransform getInverse();

      const Eigen::Matrix4d& GetMatrix() const;

      int GetID() const {return _id;};

      void SetID(int id) {_id = id;};

      void SetMatrix(const Eigen::Matrix4d& matrix);

      void MultiplyLeft(const RigidBodyTransform& transform);

      void MultiplyRight(const RigidBodyTransform& transform);

      Eigen::Vector3d transformVector(const Eigen::Vector3d& vector);

      void print();

      void setRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

      void setRotationAndTranslation(const Eigen::Vector3d& eulerAngles, const Eigen::Vector3d& translation);

      void SetQuaternionAndTranslation(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& translation);

      Eigen::Vector3d GetTranslation();

      Eigen::Matrix3d GetRotation();

      Eigen::Quaterniond GetQuaternion();

      void rotateX(float angleRad);

      void rotateY(float angleRad);

      void rotateZ(float angleRad);

      void rotate(float rad, Eigen::Vector3d axis);
};

#endif //RIGIDBODYTRANSFORM_H
