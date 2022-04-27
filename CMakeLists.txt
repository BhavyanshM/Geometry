cmake_minimum_required(VERSION 3.4)
project(Geometry)

set(Geometry_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)

find_package(Catch2 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)


set(GEOMTOOLS_SRC
        include/CameraParams.h
        include/NonLinearOptimizer.h
        include/GeomTools.h
        include/KDTree.h
        include/KDNode.h
        include/RigidBodyTransform.h
        include/LieAlgebra.h
        include/Rotation.h
        include/Line3D.h
        include/Plane3D.h
        include/Point3D.h
        include/PointLandmark.h
        include/RegionRing.h
        include/PlanarRegion.h
        include/LineSegment2D.h
        include/Line2D.h
        include/HullTools.h
        include/LineTools.h

        src/HullTools.cpp
        src/LineTools.cpp
        src/Elements/Line2D.cpp
        src/Elements/LineSegment2D.cpp
        src/Elements/CameraParams.cpp
        src/Optimizers/NonLinearOptimizer.cpp
        src/GeomTools.cpp
        src/Structures/KDTree.cpp
        src/Structures/KDNode.cpp
        src/Elements/RigidBodyTransform.cpp
        src/Elements/Rotation.cpp
        src/Elements/LieAlgebra.cpp
        src/Elements/Point3D.cpp
        src/Elements/Line3D.cpp
        src/Elements/Plane3D.cpp
        src/Elements/PointLandmark.cpp
        src/Elements/PlanarRegion.cpp
        src/Elements/RegionRing.cpp src/Elements/Line2D.cpp include/Line2D.h test/GeometryTests.cpp)

add_library(geom-tools STATIC ${GEOMTOOLS_SRC} src/HullTools.cpp include/HullTools.h src/LineTools.cpp include/LineTools.h src/Optimizers/TrajectoryOptimizer.cpp include/TrajectoryOptimizer.h src/Elements/PoseTrajectory.cpp include/PoseTrajectory.h)
target_link_libraries(geom-tools PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS})
target_include_directories(geom-tools PUBLIC include/)

add_executable(geom-tools-test test/GeometryTests.cpp
        ${GEOMTOOLS_SRC})
target_link_libraries(geom-tools-test PRIVATE
        Catch2::Catch2
        geom-tools)
target_include_directories(geom-tools-test PUBLIC include/)

