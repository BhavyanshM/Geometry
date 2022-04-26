//
// Created by quantum on 4/24/22.
//

#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_session.hpp"
#include "catch2/catch_approx.hpp"

#include "TrajectoryOptimizer.h"
#include "Line2D.h"
#include "HullTools.h"


#include "iostream"

using namespace Catch;

TEST_CASE( "Line2D Functions", "[Line2D]" )
{
   Line2D line({0,0}, 1);
   Line2D other({0,0}, 2);
}

TEST_CASE("HullTools Functions", "[HullTools]")
{
   REQUIRE(HullTools::ComputeBoundingBoxIoU({{0,0},{1,1}}, {{0.5,-0.5},{1.5,0.5}}) == Approx(0.25/(1.75)).epsilon(1e-5) );
   REQUIRE(HullTools::ComputeBoundingBoxIoU({{-1,-1},{1,1}}, {{0.0,0.0},{1.0,1.0}}) == Approx(0.25).epsilon(1e-5) );
   REQUIRE(HullTools::ComputeBoundingBoxIoU({{-1,-1},{1,1}}, {{-1,-1},{1,1}}) == Approx(1.0).epsilon(1e-5) );
   REQUIRE(HullTools::ComputeBoundingBoxIoU({{-1,-1},{1,1}}, {{2,2},{4,4}}) == Approx(0.0).epsilon(1e-5) );
   REQUIRE(HullTools::ComputeBoundingBoxIoU({{-1,-1},{0,0}}, {{0,0},{1,1}}) == Approx(0.0).epsilon(1e-5) );

   std::vector<Eigen::Vector2f> hull;
   hull.emplace_back(0.25f, 0.1f);
   hull.emplace_back(-0.25, 0.1);
   hull.emplace_back(0.0, 0.6);

   std::vector<Eigen::Vector2f> hull2;
   hull2.emplace_back(0.0f, 0.35f);
   hull2.emplace_back(0.0f, 0.85f);
   hull2.emplace_back(0.5f, 0.35f);

   REQUIRE(HullTools::ComputeBoundingBoxIoU({hull}, {hull2}) == Approx(1.0f/7.0f).epsilon(1e-5));


}

TEST_CASE("Trajectory Optimization", "[TrajectoryOptimizer]")
{
   TrajectoryOptimizer traj(0, 1, 0, 0, 1, -1);
   traj.Optimize();
   int totalSamples = 300;
   float timeUnit = 1.0f/(float) totalSamples;

   for(int i = 0; i<totalSamples; i++)
   {
      if(i<totalSamples/2) REQUIRE(traj.GetRate(timeUnit * (float)i) > 0.0f);
      if(i>totalSamples/2) REQUIRE(traj.GetRate(timeUnit * (float)i) < 0.0f);
   }

   REQUIRE(traj.GetRate(0.5) == Approx(0.0f).margin(1e-5));

   TrajectoryOptimizer traj1(0, 1, 0, 0, -1, 1);
   traj1.Optimize();

   for(int i = 0; i<totalSamples; i++)
   {
      if(i<totalSamples/2) REQUIRE(traj1.GetRate(timeUnit * (float)i) < 0.0f);
      if(i>totalSamples/2) REQUIRE(traj1.GetRate(timeUnit * (float)i) > 0.0f);
   }

   REQUIRE(traj1.GetRate(0.5) == Approx(0.0f).margin(1e-5));
}

int main(int argc, char** argv)
{

   /* Setup Here */

   int result = Catch::Session().run( argc, argv );

   /* Clean-Up Here*/

   return result;
}