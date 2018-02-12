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

# CaptureBase class test
#wolf_add_gtest(gtest_constraint_sparse gtest_constraint_sparse.cpp)
#target_link_libraries(gtest_constraint_sparse ${PROJECT_NAME})

# FeatureBase classes test
wolf_add_gtest(gtest_eigen_predicates gtest_eigen_predicates.cpp)
target_link_libraries(gtest_eigen_predicates ${PROJECT_NAME})

# FeatureBase classes test
wolf_add_gtest(gtest_feature_base gtest_feature_base.cpp)
target_link_libraries(gtest_feature_base ${PROJECT_NAME})

# FrameBase classes test
wolf_add_gtest(gtest_frame_base gtest_frame_base.cpp)
target_link_libraries(gtest_frame_base ${PROJECT_NAME})

# IMU tools test
wolf_add_gtest(gtest_imu_tools gtest_imu_tools.cpp)
# target_link_libraries(gtest_imu_tools ${PROJECT_NAME}) // WOLF library not needed

# LocalParametrizationXxx classes test
wolf_add_gtest(gtest_local_param gtest_local_param.cpp)
target_link_libraries(gtest_local_param ${PROJECT_NAME})

# MotionBuffer class test
wolf_add_gtest(gtest_motion_buffer gtest_motion_buffer.cpp)
target_link_libraries(gtest_motion_buffer ${PROJECT_NAME})

# Problem class test
wolf_add_gtest(gtest_problem gtest_problem.cpp)
target_link_libraries(gtest_problem ${PROJECT_NAME})

# ProcessorMotion class test
wolf_add_gtest(gtest_processor_motion gtest_processor_motion.cpp)
target_link_libraries(gtest_processor_motion ${PROJECT_NAME})
  
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

# ConstraintAbs(P/O/V) classes test
wolf_add_gtest(gtest_constraint_absolute gtest_constraint_absolute.cpp)
target_link_libraries(gtest_constraint_absolute ${PROJECT_NAME})

# ConstraintOdom3D class test
wolf_add_gtest(gtest_constraint_odom_3D gtest_constraint_odom_3D.cpp)
target_link_libraries(gtest_constraint_odom_3D ${PROJECT_NAME})

# ConstraintPose2D class test
wolf_add_gtest(gtest_constraint_pose_2D gtest_constraint_pose_2D.cpp)
target_link_libraries(gtest_constraint_pose_2D ${PROJECT_NAME})

# ConstraintPose3D class test
wolf_add_gtest(gtest_constraint_pose_3D gtest_constraint_pose_3D.cpp)
target_link_libraries(gtest_constraint_pose_3D ${PROJECT_NAME})

# FeatureIMU test
wolf_add_gtest(gtest_feature_imu gtest_feature_imu.cpp)
target_link_libraries(gtest_feature_imu ${PROJECT_NAME})

# IMU Bias + estimation tests
wolf_add_gtest(gtest_IMU gtest_IMU.cpp)
target_link_libraries(gtest_IMU ${PROJECT_NAME})

# Pinhole test
wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)
target_link_libraries(gtest_pinhole ${PROJECT_NAME})

# ProcessorFrameNearestNeighborFilter class test
wolf_add_gtest(gtest_processor_frame_nearest_neighbor_filter_2D gtest_processor_frame_nearest_neighbor_filter_2D.cpp)
target_link_libraries(gtest_processor_frame_nearest_neighbor_filter_2D ${PROJECT_NAME})

# ProcessorIMU class test
wolf_add_gtest(gtest_processor_imu gtest_processor_imu.cpp)
target_link_libraries(gtest_processor_imu ${PROJECT_NAME})

# ProcessorIMU_jacobians test
wolf_add_gtest(gtest_processorIMU_jacobians gtest_processorIMU_jacobians.cpp)
target_link_libraries(gtest_processorIMU_jacobians ${PROJECT_NAME})

# ProcessorMotion in 2D
wolf_add_gtest(gtest_odom_2D gtest_odom_2D.cpp)
target_link_libraries(gtest_odom_2D ${PROJECT_NAME})

# ProcessorOdom3D class test
wolf_add_gtest(gtest_odom_3D gtest_odom_3D.cpp)
target_link_libraries(gtest_odom_3D ${PROJECT_NAME})

# ConstraintAutodiff class test
wolf_add_gtest(gtest_constraint_autodiff gtest_constraint_autodiff.cpp)
target_link_libraries(gtest_constraint_autodiff ${PROJECT_NAME})

<<<<<<< HEAD
=======
# ROI test
IF(OpenCV_FOUND)
  wolf_add_gtest(gtest_roi_ORB gtest_roi_ORB.cpp)
  target_link_libraries(gtest_roi_ORB ${PROJECT_NAME})
ENDIF(OpenCV_FOUND)

>>>>>>> master
# ------- Now Core classes Serialization ----------

add_subdirectory(serialization)

