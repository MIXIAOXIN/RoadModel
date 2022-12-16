cmake_minimum_required(VERSION 3.24)
project(RoadModel)

set(CMAKE_CXX_STANDARD 14)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -luuid")

#add external depedent include directoies
find_package(Boost REQUIRED COMPONENTS filesystem thread)
include_directories(${BOOST_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(CGAL REQUIRED COMPONENTS Qt5)
#find_package(Qt5 COMPONENTS Xml Script OpenGL Svg)
find_package(Qt5 COMPONENTS Core Gui OpenGL Widgets)

find_package(shapelib REQUIRED)
include_directories(${shapelib_INCLUDE_DIRS})


#creat_single_source_cgal_program("main.cpp")

add_executable(RoadModel
        main.cpp
        shpline_io_shplib.cpp function.cpp)

if(CGAL_Qt5_FOUND)
#    add_definitions(-DCGAL_USE_BASIC_VIEWER)
# target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)
    target_link_libraries(RoadModel
            PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
            PUBLIC CGAL::CGAL_Basic_viewer
            CGAL::CGAL
#            CGAL::CGAL_Core
            libshp.so
            ${Boost_LIBRARY_LIB}
            #        ${Qt5Widgets_LIBRARIES}

            )
else()
    target_link_libraries(RoadModel
            libshp.so
            ${Boost_LIBRARY_LIB}
            CGAL::CGAL
            )
endif()
