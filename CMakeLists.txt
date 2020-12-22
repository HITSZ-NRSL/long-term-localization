cmake_minimum_required(VERSION 3.0.2)
project(long_term_relocalization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_definitions(${PCL_DEFINITIONS})

# ====================LIB====================
cs_add_library(
  ${PROJECT_NAME}
  src/depth_clustering/depth_clustering_utils.cc
  src/offline_process/semantic_kitti_utils.cc
  src/recognizers/correspondence_recognizer_factory.cc
  src/recognizers/geometric_consistency_recognizer.cc
  src/recognizers/graph_based_geometric_consistency_recognizer.cc
  src/recognizers/incremental_geometric_consistency_recognizer.cc
  src/recognizers/partitioned_geometric_consistency_recognizer.cc
  src/relocalization/cluster.cc
  src/relocalization/cluster_manager.cc
  src/relocalization/cluster_matcher.cc
  src/relocalization/relocalization.cc
  src/relocalization/semantic_cluster_map.cc
  src/utils/utils.cc
  src/utils/params_loader.cc
  src/utils/pose_saver.cc)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# ====================EXE====================
cs_add_executable(semantic_bag_writer_main
                  src/offline_process/semantic_bag_writer_main.cc)
target_link_libraries(semantic_bag_writer_main ${PROJECT_NAME})

cs_add_executable(depth_clustering_main
                  src/depth_clustering/depth_clustering_main.cc)
target_link_libraries(depth_clustering_main ${PROJECT_NAME})

cs_add_executable(relocalization_main src/relocalization/relocalization_main.cc)
target_link_libraries(relocalization_main ${PROJECT_NAME})

cs_install()
cs_export()
