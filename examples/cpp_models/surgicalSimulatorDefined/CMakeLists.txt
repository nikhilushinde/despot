cmake_minimum_required(VERSION 2.8.3)

# to enable multi threading
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# set the OpenCV_DIR variable equal to the path of previously installed opencv
set(OpenCV_DIR /Users/nikhilushinde/anaconda3/envs/cv_env)

# configure the necessary common CMake environment variables
# needed to include and link the OpenCV program into this
# demo project, namely OpenCV_INCLUDE_DIRS and OpenCV_LIBS
find_package( OpenCV REQUIRED )

# include the following directories
include_directories(
  src
  ${OpenCV_INCLUDE_DIRS}
)

add_executable("${PROJECT_NAME}_surgicalSimulator"
  src/surgical_despot.cpp
  src/environment.cpp
  src/obstacle.cpp
  src/robot.cpp
  src/robot_arm.cpp
  src/camera.cpp
  src/render_sim.cpp
  src/surgical_utils.cpp
  src/pq_dict.cpp
  src/astar_planner.cpp
  #src/test_a_star.cpp
  #src/test.cpp
  src/main.cpp
  #src/test_environment_simulator_renderer.cpp
)

# tell it to link the executable target against OpenCV
target_link_libraries("${PROJECT_NAME}_surgicalSimulator"
  "${PROJECT_NAME}" 
  "${OpenCV_LIBS}"
)



install(TARGETS
    "${PROJECT_NAME}_surgicalSimulator"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)
