# numeric test
ADD_EXECUTABLE(test_ceres_wrapper_numeric test_ceres_wrapper_numeric.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_numeric ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test
ADD_EXECUTABLE(test_ceres_wrapper_jet test_ceres_wrapper_jet.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet individual blocks test
ADD_EXECUTABLE(test_ceres_wrapper_jet_ind_block test_ceres_wrapper_jet_ind_block.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet_ind_block ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test with 2 sensors
ADD_EXECUTABLE(test_ceres_wrapper_jet_2_sensors test_ceres_wrapper_jet_2_sensors.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper_jet_2_sensors ${CERES_LIBRARIES} ${PROJECT_NAME})

# jet test with 2 sensors and wrapper
ADD_EXECUTABLE(test_ceres_wrapper test_ceres_wrapper.cpp)
TARGET_LINK_LIBRARIES(test_ceres_wrapper ${CERES_LIBRARIES} ${PROJECT_NAME})
