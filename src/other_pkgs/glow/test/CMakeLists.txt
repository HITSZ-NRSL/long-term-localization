# build the test cases
include_directories(${GTEST_INCLUDE_DIRS})

set(TEST_BINARY ${PROJECT_NAME}_test)
add_executable(${TEST_BINARY}
  main.cpp
  texture-test.cpp
  buffer-test.cpp
  framebuffer-test.cpp
  color-test.cpp
  #camera-test.cpp
)


target_link_libraries(${TEST_BINARY} ${GTEST_MAIN_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glow glow_util)

add_test(${TEST_BINARY} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TEST_BINARY})
