cmake_minimum_required(VERSION 2.8.3)
project(drive)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  tf
)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
    roscpp
    )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(drive
  src/drive.cpp
  )

target_link_libraries(drive
     ${catkin_LIBRARIES}
     )

add_dependencies(drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(drive_node
  src/drive_node.cpp
)
add_dependencies(drive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive_node drive)
set_target_properties(drive_node PROPERTIES OUTPUT_NAME drive)

install(
    TARGETS
        drive
        drive_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
