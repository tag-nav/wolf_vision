# Retrieve googletest from github & compile
add_subdirectory(gtest)


## Added these two include_dirs: ######################
#
#CMAKE modules
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
#MESSAGE(STATUS ${CMAKE_MODULE_PATH})
#
# Include Eigen
#FIND_PACKAGE(Eigen 3 REQUIRED)
#INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
#
# Include Ceres
#FIND_PACKAGE(Ceres QUIET) #Ceres is not required
#IF(Ceres_FOUND)
#    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
#ENDIF(Ceres_FOUND)
#
## and now gtest_motion_2d works ######################


# Include gtest directory.
include_directories(${GTEST_INCLUDE_DIRS})

############# USE THIS TEST AS AN EXAMPLE #################
#                                                         #
# Create a specific test executable for gtest_example     #
wolf_add_gtest(gtest_example gtest_example.cpp)           #
target_link_libraries(gtest_example ${PROJECT_NAME})      #
#                                                         #
###########################################################


################# ADD YOUR TESTS BELOW ####################
#                                                         #
#           ==== IN ALPHABETICAL ORDER! ====              #
#                                                         #

# ------- First Core classes ----------

# CaptureBase class test
wolf_add_gtest(gtest_capture_base gtest_capture_base.cpp)
target_link_libraries(gtest_capture_base ${PROJECT_NAME})

# FeatureBase classes test
wolf_add_gtest(gtest_feature_base gtest_feature_base.cpp)
target_link_libraries(gtest_feature_base ${PROJECT_NAME})

# FrameBase classes test
wolf_add_gtest(gtest_frame_base gtest_frame_base.cpp)
target_link_libraries(gtest_frame_base ${PROJECT_NAME})

# FrameIMU class test
wolf_add_gtest(gtest_frame_imu gtest_frame_imu.cpp)
target_link_libraries(gtest_frame_imu ${PROJECT_NAME})

# LocalParametrizationXxx classes test
wolf_add_gtest(gtest_local_param gtest_local_param.cpp)
target_link_libraries(gtest_local_param ${PROJECT_NAME})

# MotionBuffer class test
wolf_add_gtest(gtest_motion_buffer gtest_motion_buffer.cpp)
target_link_libraries(gtest_motion_buffer ${PROJECT_NAME})

# Problem class test
wolf_add_gtest(gtest_problem gtest_problem.cpp)
target_link_libraries(gtest_problem ${PROJECT_NAME})

# Rotation test
wolf_add_gtest(gtest_rotation gtest_rotation.cpp)
target_link_libraries(gtest_rotation ${PROJECT_NAME})

# TimeStamp class test
wolf_add_gtest(gtest_time_stamp gtest_time_stamp.cpp)
target_link_libraries(gtest_time_stamp ${PROJECT_NAME})

# TrajectoryBase class test
wolf_add_gtest(gtest_trajectory gtest_trajectory.cpp)
target_link_libraries(gtest_trajectory ${PROJECT_NAME})

# ------- Now Derived classes ----------

# ConstraintFix3D class test
wolf_add_gtest(gtest_constraint_fix_3D gtest_constraint_fix_3D.cpp)
target_link_libraries(gtest_constraint_fix_3D ${PROJECT_NAME})

# ConstraintFix class test
wolf_add_gtest(gtest_constraint_fix gtest_constraint_fix.cpp)
target_link_libraries(gtest_constraint_fix ${PROJECT_NAME})

# ConstraintIMU test
wolf_add_gtest(gtest_constraint_imu gtest_constraint_imu.cpp)
target_link_libraries(gtest_constraint_imu ${PROJECT_NAME})

# ConstraintOdom3D class test
wolf_add_gtest(gtest_constraint_odom_3D gtest_constraint_odom_3D.cpp)
target_link_libraries(gtest_constraint_odom_3D ${PROJECT_NAME})

# FeatureIMU test
wolf_add_gtest(gtest_feature_imu gtest_feature_imu.cpp)
target_link_libraries(gtest_feature_imu ${PROJECT_NAME})

# IMU Bias + estimation tests
wolf_add_gtest(gtest_imuBias gtest_imuBias.cpp)
target_link_libraries(gtest_imuBias ${PROJECT_NAME})

# Pinhole test
wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)
target_link_libraries(gtest_pinhole ${PROJECT_NAME})

# ProcessorIMU class test
wolf_add_gtest(gtest_processor_imu gtest_processor_imu.cpp)
target_link_libraries(gtest_processor_imu ${PROJECT_NAME})

# ProcessorIMU_jacobians test
wolf_add_gtest(gtest_processorIMU_jacobians gtest_processorIMU_jacobians.cpp)
target_link_libraries(gtest_processorIMU_jacobians ${PROJECT_NAME})

# Ceres ProcessorMotion solver tests
wolf_add_gtest(gtest_processorMotion_optimization_testCase gtest_processorMotion_optimization_testCase.cpp)
target_link_libraries(gtest_processorMotion_optimization_testCase ${PROJECT_NAME})

# ProcessorMotion in 2D
wolf_add_gtest(gtest_odom_2D gtest_odom_2D.cpp)
target_link_libraries(gtest_odom_2D ${PROJECT_NAME})

# ProcessorOdom3D class test
wolf_add_gtest(gtest_odom_3D gtest_odom_3D.cpp)
target_link_libraries(gtest_odom_3D ${PROJECT_NAME})

# ROI test
wolf_add_gtest(gtest_roi_ORB gtest_roi_ORB.cpp)
target_link_libraries(gtest_roi_ORB ${PROJECT_NAME})

# SensorIMU test
wolf_add_gtest(gtest_sensor_imu gtest_sensor_imu.cpp)
target_link_libraries(gtest_sensor_imu ${PROJECT_NAME})
