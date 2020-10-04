cmake_minimum_required(VERSION 2.8.3)

# set the OpenCV_DIR variable equal to the path of previously installed opencv
set(OpenCV_DIR /Users/nikhilushinde/anaconda3)

# configure the necessary common CMake environment variables
# needed to include and link the OpenCV program into this
# demo project, namely OpenCV_INCLUDE_DIRS and OpenCV_LIBS
find_package( OpenCV REQUIRED )

# include the following directories
include_directories(
  src
  ${OpenCV_INCLUDE_DIRS}
)

add_executable("${PROJECT_NAME}_tag_nikhil"
  src/tag_nikhil.cpp
  src/tag_state.cpp
  src/main.cpp
  #src/simulator.cpp
)

# tell it to link the executable target against OpenCV
target_link_libraries("${PROJECT_NAME}_tag_nikhil"
  "${PROJECT_NAME}" 
  "${OpenCV_LIBS}"
)



install(TARGETS
    "${PROJECT_NAME}_tag_nikhil"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)