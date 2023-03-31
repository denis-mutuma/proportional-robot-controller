cmake_minimum_required(VERSION 3.0.2)
project(practical)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES practical
  # CATKIN_DEPENDS roscpp std_msgs
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
)

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)

add_executable(goto_xy src/goto_xy.cpp)
target_link_libraries(goto_xy ${catkin_LIBRARIES})