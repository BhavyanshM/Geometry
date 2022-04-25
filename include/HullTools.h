//
// Created by quantum on 4/24/22.
//

#ifndef MAP_SENSE_HULLTOOLS_H
#define MAP_SENSE_HULLTOOLS_H

#include "vector"
#include "stack"
#include "iostream"
#include "Eigen/Core"

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BoolDynamicMatrix;

class HullTools
{
   public:
      static std::vector<Eigen::Vector2f> CalculateIntersection(const std::vector<Eigen::Vector2f>& points1, const std::vector<Eigen::Vector2f>& points2);

      static void GetParametricCurve(std::vector<Eigen::Vector2f> points, uint8_t m, Eigen::MatrixXf& params);

      static bool CheckPointInside(const std::vector<Eigen::Vector2f>& hull, const Eigen::Vector2f& point);


      static float ComputeWindingNumber(const std::vector<Eigen::Vector2f>& concaveHull, const Eigen::Vector2f& point);

      static void CanvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, std::vector<Eigen::Vector2f>& concaveHull,
                                    Eigen::Vector2f start, float scale);

      static std::vector<Eigen::Vector2f> GrahamScanConvexHull(std::vector<Eigen::Vector2f> points);

      static std::vector<Eigen::Vector2f> CanvasApproximateConcaveHull(std::vector<Eigen::Vector2f> points, uint16_t windowHeight, uint16_t windowWidth);
};

#endif //MAP_SENSE_HULLTOOLS_H
