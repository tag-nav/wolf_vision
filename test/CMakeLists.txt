# Retrieve googletest from github & compile
add_subdirectory(${PROJECT_SOURCE_DIR}/test/gtest)

include_directories(${GTEST_INCLUDE_DIRS})

