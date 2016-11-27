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

# FrameBase classes test
wolf_add_gtest(gtest_frame_base gtest_frame_base.cpp)
target_link_libraries(gtest_frame_base ${PROJECT_NAME})

# LocalParametrizationXxx classes test
wolf_add_gtest(gtest_local_param gtest_local_param.cpp)
target_link_libraries(gtest_local_param ${PROJECT_NAME})

# MotionBuffer class test
wolf_add_gtest(gtest_motion_buffer gtest_motion_buffer.cpp)
target_link_libraries(gtest_motion_buffer ${PROJECT_NAME})

# Pinhole test
wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)
target_link_libraries(gtest_pinhole ${PROJECT_NAME})

# Problem class test
wolf_add_gtest(gtest_problem gtest_problem.cpp)
target_link_libraries(gtest_problem ${PROJECT_NAME})

# ProcessorIMU class test
wolf_add_gtest(gtest_processor_imu gtest_processor_imu.cpp)
target_link_libraries(gtest_processor_imu ${PROJECT_NAME})

# ProcessorMotion in 2D
wolf_add_gtest(gtest_motion_2d gtest_motion_2d.cpp)
target_link_libraries(gtest_motion_2d ${PROJECT_NAME})

# ProcessorOdom3D class test
wolf_add_gtest(gtest_odom_3D gtest_odom_3D.cpp)
target_link_libraries(gtest_odom_3D ${PROJECT_NAME})

# Rotation test
wolf_add_gtest(gtest_rotation gtest_rotation.cpp)
target_link_libraries(gtest_rotation ${PROJECT_NAME})

# TimeStamp class test
wolf_add_gtest(gtest_time_stamp gtest_time_stamp.cpp)
target_link_libraries(gtest_time_stamp ${PROJECT_NAME})

# TrajectoryBase class test
wolf_add_gtest(gtest_trajectory gtest_trajectory.cpp)
target_link_libraries(gtest_trajectory ${PROJECT_NAME})

