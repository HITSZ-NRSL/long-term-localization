cmake_minimum_required(VERSION 3.0.2)
project(long_term_relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# ====================LIB====================
# cs_add_library( ${PROJECT_NAME} src/utils/params_loader.cc
# src/utils/pose_saver.cc src/offline_process/semantic_kitti_utils.cc)

# ====================EXE====================
cs_add_executable(
  semantic_bag_writer_main src/offline_process/semantic_bag_writer_main.cc
  src/utils/params_loader.cc src/utils/pose_saver.cc
  src/offline_process/semantic_kitti_utils.cc)
# target_link_libraries(semantic_bag_writer_main ${PROJECT_NAME})

cs_install()
cs_export()
