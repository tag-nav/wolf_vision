# Retrieve googletest from github & compile
add_subdirectory(${PROJECT_SOURCE_DIR}/test/gtest)

include_directories(${GTEST_INCLUDE_DIRS})

# Create a specific test executable for gtest_example
wolf_add_gtest(gtest_example gtest_example.cpp)
target_link_libraries(gtest_example ${PROJECT_NAME})
