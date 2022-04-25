//
// Created by quantum on 4/24/22.
//

#include <catch2/catch_test_macros.hpp>
#include "catch2/catch_session.hpp"

#include "Line2D.h"

#include "iostream"

TEST_CASE( "Line2D Functions", "[Line2D]" ) {

   Line2D line({0,0}, 1);
   Line2D other({0,0}, 2);

   auto point = line.IntersectWith(other);
   REQUIRE( (point.x() == 0 && point.y() == 0));

}

int main(int argc, char** argv)
{

   /* Setup Here */

   int result = Catch::Session().run( argc, argv );

   /* Clean-Up Here*/

   return result;
}