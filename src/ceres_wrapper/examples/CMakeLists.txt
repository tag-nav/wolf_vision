# numeric test
ADD_EXECUTABLE(test_ceres_wrapper_numeric test_ceres_wrapper_numeric.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_numeric ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test
ADD_EXECUTABLE(test_ceres_wrapper_jet test_ceres_wrapper_jet.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test with 2 sensors
# ADD_EXECUTABLE(test_ceres_wrapper_jet test_ceres_wrapper_jet_2sensors.cpp)
# TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet ${CERES_LIBRARIES} ${PROJECT_NAME})
