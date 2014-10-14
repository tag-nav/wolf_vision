# create an example application & link necessary libraries
ADD_EXECUTABLE(test_ceres_wrapper_numeric test_ceres_wrapper_numeric.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_numeric ${CERES_LIBRARIES} ${PROJECT_NAME})
