cmake_minimum_required(VERSION 2.8.3)

include_directories(
  src
)

add_executable("${PROJECT_NAME}_tag_nikhil"
  src/tag_nikhil.cpp
  src/tag_state.cpp
  src/main.cpp
)
target_link_libraries("${PROJECT_NAME}_tag_nikhil"
  "${PROJECT_NAME}"
)


install(TARGETS
    "${PROJECT_NAME}_tag_nikhil"
  RUNTIME DESTINATION "${BINARY_INSTALL_PATH}"
)