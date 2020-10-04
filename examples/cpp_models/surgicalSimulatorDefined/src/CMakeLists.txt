# CMakeLists.txt

# Older versions of CMake are likely to work just fine but, since
# I don't know where to cut off I just use the version I'm using
cmake_minimum_required(VERSION "3.17")

# name of this example project
project(SurgicalSim)

# set OpenCV_DIR variable equal to the path to the cmake
# files within the previously installed opencv program
set(OpenCV_DIR /Users/nikhilushinde/anaconda3)

# Tell compiler to use C++ 14 features which is needed because
# Clang version is often behind in the XCode installation
set(CMAKE_CXX_STANDARD 14)

# configure the necessary common CMake environment variables
# needed to include and link the OpenCV program into this
# demo project, namely OpenCV_INCLUDE_DIRS and OpenCV_LIBS
find_package( OpenCV REQUIRED )

# tell the build to include the headers from OpenCV
include_directories( ${OpenCV_INCLUDE_DIRS} )

# set all the source files 

# to just test the robot arm independently 
# set(SOURCES test_robot_arm_simulator_renderer.cpp robot_arm.cpp surgical_utils.cpp render_sim.cpp obstacle.cpp environment.cpp)

# to test the robot 
# set(SOURCES test_robot_simulator_renderer.cpp robot_arm.cpp robot.cpp surgical_utils.cpp render_sim_robot.cpp)

# to test the entire environment
set (SOURCES test_environment_simulator_renderer.cpp robot_arm.cpp robot.cpp obstacle.cpp environment.cpp camera.cpp surgical_utils.cpp render_sim.cpp)

# file(GLOB SOURCES "./*.cpp")

# specify the executable target to be built
add_executable(surgicalSimulator ${SOURCES})

# tell it to link the executable target against OpenCV
target_link_libraries(surgicalSimulator ${OpenCV_LIBS} )