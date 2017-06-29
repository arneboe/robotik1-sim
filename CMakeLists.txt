pkg_check_modules(EIGEN REQUIRED "eigen3")
include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})

find_package(OpenSceneGraph REQUIRED osgManipulator osgViewer osgDB osgGA osgText osgUtil)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
link_directories(${OPENSCENEGRAPH_LIBRARY_DIRS})

FIND_PACKAGE( Boost COMPONENTS system REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_definitions(-DSIMULATOR)

ADD_EXECUTABLE(sim
    Parcour.cpp
    Simulator.cpp
    Parser.cpp
    Robot.cpp
    Visualization.cpp
    camera_stub.cpp
    MadgwickReplace.cpp
    ../../engine.cpp
    ../../protocol.cpp
    ../../crc.c
    ../../odometry.cpp
    ../../bv_processor.cpp
    ../../drivers/time.cpp
)
target_link_libraries(sim stubs pthread rt ${OPENSCENEGRAPH_LIBRARIES} ${Boost_LIBRARIES} )
