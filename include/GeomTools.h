//
// Created by quantum on 3/3/21.
//

#ifndef GEOMTOOLS_H
#define GEOMTOOLS_H

#include "Eigen/Dense"

#include "PlanarRegion.h"
#include <stack>
#include <iostream>
#include <string>
#include <fstream>

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BoolDynamicMatrix;

class GeomTools
{
   public:
      static void CanvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, std::vector<Eigen::Vector2f>& concaveHull,
                                    Eigen::Vector2f start, float scale);

      static std::vector<Eigen::Vector2f> CanvasApproximateConcaveHull(std::vector<Eigen::Vector2f> points, uint16_t windowHeight, uint16_t windowWidth);

      static Eigen::Matrix3f GetRotationFromAngleApproximations(Eigen::Vector3f eulerAngles);

      static Eigen::Vector3f GetProjectedPoint(Eigen::Vector4f plane, Eigen::Vector3f point);

      static std::vector<Eigen::Vector2f> GrahamScanConvexHull(std::vector<Eigen::Vector2f> points);

      static void GetParametricCurve(std::vector<Eigen::Vector2f> points, uint8_t m, Eigen::MatrixXf& params);

      static void LoadRegions(int frameId, std::vector<std::shared_ptr<PlanarRegion>>& regions, std::string directory, std::vector<std::string> files);

      static void SaveRegions(std::vector<std::shared_ptr<PlanarRegion>> regions, std::string fileName);

      static void TransformRegions(std::vector<std::shared_ptr<PlanarRegion>>& regions, RigidBodyTransform transform);

      static void TransformRegions(std::vector<std::shared_ptr<PlanarRegion>>& regions, Eigen::Vector3d translation, Eigen::Matrix3d rotation);

      static void LoadPoseStamped(std::ifstream& poseFile, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

      static Eigen::Vector3f GetLineFromTwoPoints2D(const Eigen::Vector2f& start, const Eigen::Vector2f& end);

      static float GetDistanceFromLine2D(const Eigen::Vector3f& line, const Eigen::Vector2f& point);

      static float GetCosineSimilarity2D(const Eigen::Vector2f& a, const Eigen::Vector2f& b);

      static float ComputeWindingNumber(const std::vector<Eigen::Vector2f>& concaveHull, const Eigen::Vector2f& point);

      static bool
      CheckPatchConnection(const Eigen::Vector3f& ag, const Eigen::Vector3f& an, const Eigen::Vector3f& bg, const Eigen::Vector3f& bn, float distanceThreshold,
                           float angularThreshold);

      static void
      AppendMeasurementsToFile(const Eigen::Matrix4f odometry, const std::vector<std::pair<int, int>>& matches, const std::string& filename, int prevId,
                               int curId);

      static const std::vector<Eigen::Vector2f>& CalculateIntersection(const std::vector<Eigen::Vector2f>& points1, const std::vector<Eigen::Vector2f>& points2);
};

#endif //GEOMTOOLS_H
