//
// Created by quantum on 4/24/22.
//

#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_session.hpp"
#include "catch2/catch_approx.hpp"

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
   REQUIRE(HullTools::ComputeBoundingBoxIoU({{-1,-1},{0,0}}, {{0,0},{1,1}}) == Approx(0.1).epsilon(1e-5) );
}

int main(int argc, char** argv)
{

   /* Setup Here */

   int result = Catch::Session().run( argc, argv );

   /* Clean-Up Here*/

   return result;
}