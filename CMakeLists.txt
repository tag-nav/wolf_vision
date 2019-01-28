# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
# MAC OSX RPATH
SET(CMAKE_MACOSX_RPATH 1)


# The project name
PROJECT(wolf)


#string(COMPARE EQUAL "${CMAKE_BINARY_DIR}" "" result)
#IF(result)
#  SET(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR})
#ENDIF()
#message(STATUS "Binary path : " ${CMAKE_BINARY_DIR})
#
#SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
#
# We'll set the install prefix only is it's empty
# which shouldn't be the case ...
#string(COMPARE EQUAL "${CMAKE_INSTALL_PREFIX}" "" result)
#IF(result)
  # This path is actually default on linux
#  SET(CMAKE_INSTALL_PREFIX /usr/local)
#ENDIF()
#message(STATUS "Installation path : " ${CMAKE_INSTALL_PREFIX})

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX /usr/local)

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "DEBUG")
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

if(UNIX)
  # GCC is not strict enough by default, so enable most of the warnings.
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Werror=all -Werror=extra -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-parameter -Wno-missing-field-initializers")
endif(UNIX)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
		message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
		message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


#OPTION(BUILD_DOC "Build Documentation" OFF)
OPTION(BUILD_TESTS "Build Unit tests" ON)
#############
## Testing ##
#############
#
if(BUILD_TESTS)
    # Enables testing for this directory and below.
    # Note that ctest expects to find a test file in the build directory root.
    # Therefore, this command should be in the source directory root.
    #include(CTest) # according to http://public.kitware.com/pipermail/cmake/2012-June/050853.html
    enable_testing()
endif()

#+START_SRC --------------------------------------------------------------------------------------------------------------------------------
#Start WOLF build
MESSAGE("Starting WOLF CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#CMAKE modules

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
MESSAGE(STATUS ${CMAKE_MODULE_PATH})

# Some wolf compilation options

IF((CMAKE_BUILD_TYPE MATCHES DEBUG) OR (CMAKE_BUILD_TYPE MATCHES debug) OR (CMAKE_BUILD_TYPE MATCHES Debug))
  set(_WOLF_DEBUG true)
ENDIF()

option(_WOLF_TRACE "Enable wolf tracing macro" ON)

option(BUILD_EXAMPLES "Build examples" OFF)
set(BUILD_TESTS true)

# Does this has any other interest
# but for the examples ?
# yes, for the tests !
IF(BUILD_EXAMPLES OR BUILD_TESTS)
  set(_WOLF_ROOT_DIR ${CMAKE_SOURCE_DIR})
ENDIF(BUILD_EXAMPLES OR BUILD_TESTS)


#find dependencies.

FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)

FIND_PACKAGE(Threads REQUIRED)

FIND_PACKAGE(Ceres QUIET) #Ceres is not required
IF(Ceres_FOUND)
    MESSAGE("Ceres Library FOUND: Ceres related sources will be built.")
ENDIF(Ceres_FOUND)

FIND_PACKAGE(faramotics QUIET) #faramotics is not required
IF(faramotics_FOUND)
	FIND_PACKAGE(GLUT REQUIRED)
	FIND_PACKAGE(pose_state_time REQUIRED)
    MESSAGE("Faramotics Library FOUND: Faramotics related sources will be built.")
ENDIF(faramotics_FOUND)

FIND_PACKAGE(laser_scan_utils QUIET) #laser_scan_utils is not required
IF(laser_scan_utils_FOUND)
    MESSAGE("laser_scan_utils Library FOUND: laser_scan_utils related sources will be built.")
ENDIF(laser_scan_utils_FOUND)

FIND_PACKAGE(raw_gps_utils QUIET) #raw_gps_utils is not required
IF(raw_gps_utils_FOUND)
    MESSAGE("raw_gps_utils Library FOUND: raw_gps_utils related sources will be built.")
ENDIF(raw_gps_utils_FOUND)

# Vision Utils
FIND_PACKAGE(vision_utils QUIET)
IF (vision_utils_FOUND)
	MESSAGE("vision_utils Library FOUND: vision related sources will be built.")
	SET(PRINT_INFO_VU false)
	FIND_PACKAGE(OpenCV QUIET)
ENDIF(vision_utils_FOUND)

# Cereal
FIND_PACKAGE(cereal QUIET)
IF(cereal_FOUND)
    MESSAGE("cereal Library FOUND: cereal related sources will be built.")
ENDIF(cereal_FOUND)

# YAML with yaml-cpp
INCLUDE (${PROJECT_SOURCE_DIR}/cmake_modules/FindYamlCpp.cmake)
IF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library FOUND: yaml-cpp related sources will be built.")
ELSEIF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library NOT FOUND!")
ENDIF(YAMLCPP_FOUND)

#GLOG
INCLUDE (${PROJECT_SOURCE_DIR}/cmake_modules/FindGlog.cmake)
IF(GLOG_FOUND)
    MESSAGE("glog Library FOUND: glog related sources will be built.")
    MESSAGE(STATUS ${GLOG_INCLUDE_DIR})
    MESSAGE(STATUS ${GLOG_LIBRARY})
ELSEIF(GLOG_FOUND)
    MESSAGE("glog Library NOT FOUND!")
ENDIF(GLOG_FOUND)

# SuiteSparse doesn't have find*.cmake:
FIND_PATH(
    Suitesparse_INCLUDE_DIRS
    NAMES SuiteSparse_config.h
    PATHS /usr/include/suitesparse /usr/local/include/suitesparse)
MESSAGE("Found suitesparse_INCLUDE_DIRS:" ${Suitesparse_INCLUDE_DIRS})

IF(Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND TRUE)
   MESSAGE("Suitesparse FOUND: wolf_solver will be built.")
ELSE (Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND FALSE)
   MESSAGE(FATAL_ERROR "Suitesparse NOT FOUND")
ENDIF (Suitesparse_INCLUDE_DIRS)

# Define the directory where will be the configured config.h
SET(WOLF_CONFIG_DIR ${PROJECT_BINARY_DIR}/conf/internal)

# Create the specified output directory if it does not exist.
IF(NOT EXISTS "${WOLF_CONFIG_DIR}")
  message(STATUS "Creating config output directory: ${WOLF_CONFIG_DIR}")
  file(MAKE_DIRECTORY "${WOLF_CONFIG_DIR}")
ENDIF()
IF(EXISTS "${WOLF_CONFIG_DIR}" AND NOT IS_DIRECTORY "${WOLF_CONFIG_DIR}")
  message(FATAL_ERROR "Bug: Specified CONFIG_DIR: "
    "${WOLF_CONFIG_DIR} exists, but is not a directory.")
ENDIF()
# Configure config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/internal/config.h.in "${WOLF_CONFIG_DIR}/config.h")
message("CONFIG DIRECTORY ${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_BINARY_DIR}/conf")
# include spdlog (logging library)
FIND_PATH(SPDLOG_INCLUDE_DIR spdlog.h /usr/local/include/spdlog /usr/include/spdlog)
IF (SPDLOG_INCLUDE_DIR)
  INCLUDE_DIRECTORIES(${SPDLOG_INCLUDE_DIR})
  MESSAGE(STATUS "Found spdlog: ${SPDLOG_INCLUDE_DIR}")
ELSE (SPDLOG_INCLUDE_DIR)
 MESSAGE(FATAL_ERROR "Could not find spdlog")
ENDIF (SPDLOG_INCLUDE_DIR)  
  
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
include_directories("include")
include_directories("templinks")
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

IF(faramotics_FOUND)
    INCLUDE_DIRECTORIES(${faramotics_INCLUDE_DIRS})
ENDIF(faramotics_FOUND)

IF(laser_scan_utils_FOUND)
    INCLUDE_DIRECTORIES(${laser_scan_utils_INCLUDE_DIRS})
ENDIF(laser_scan_utils_FOUND)

IF(raw_gps_utils_FOUND)
    INCLUDE_DIRECTORIES(${raw_gps_utils_INCLUDE_DIRS})
ENDIF(raw_gps_utils_FOUND)

IF(vision_utils_FOUND)
	INCLUDE_DIRECTORIES(${vision_utils_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
ENDIF(vision_utils_FOUND)

# cereal
IF(cereal_FOUND)
    INCLUDE_DIRECTORIES(${cereal_INCLUDE_DIRS})
ENDIF(cereal_FOUND)

IF(Suitesparse_FOUND)
    INCLUDE_DIRECTORIES(${Suitesparse_INCLUDE_DIRS})
ENDIF(Suitesparse_FOUND)

IF(YAMLCPP_FOUND)
    INCLUDE_DIRECTORIES(${YAMLCPP_INCLUDE_DIR})
ENDIF(YAMLCPP_FOUND)

IF(GLOG_FOUND)
INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})
ENDIF(GLOG_FOUND)

#HEADERS

SET(HDRS_BASE
include/base/diff_drive_tools.h
include/base/diff_drive_tools.hpp
include/base/eigen_assert.h
include/base/eigen_predicates.h
include/base/factory.h
include/base/frame_base.h
include/base/hardware_base.h
include/base/IMU_tools.h
include/base/local_parametrization_angle.h
include/base/local_parametrization_base.h
include/base/local_parametrization_homogeneous.h
include/base/local_parametrization_polyline_extreme.h
include/base/local_parametrization_quaternion.h
include/base/logging.h
include/base/make_unique.h
include/base/map_base.h
include/base/motion_buffer.h
include/base/node_base.h
include/base/pinhole_tools.h
include/base/problem.h
include/base/rotations.h
include/base/singleton.h
include/base/state_angle.h
include/base/state_block.h
include/base/state_homogeneous_3D.h
include/base/state_quaternion.h
include/base/three_D_tools.h
include/base/time_stamp.h
include/base/track_matrix.h
include/base/trajectory_base.h
include/base/wolf.h
    )
SET(HDRS
include/base/capture/capture_motion.h
include/base/capture/capture_GPS_fix.h
include/base/capture/capture_IMU.h
include/base/capture/capture_odom_2D.h
include/base/capture/capture_odom_3D.h
include/base/constraint/constraint_block_absolute.h
include/base/constraint/constraint_container.h
include/base/constraint/constraint_corner_2D.h
include/base/constraint/constraint_AHP.h
include/base/constraint/constraint_epipolar.h
include/base/constraint/constraint_IMU.h
include/base/constraint/constraint_fix_bias.h
include/base/constraint/constraint_GPS_2D.h
include/base/constraint/constraint_GPS_pseudorange_3D.h
include/base/constraint/constraint_GPS_pseudorange_2D.h
include/base/constraint/constraint_odom_2D.h
include/base/constraint/constraint_odom_2D_analytic.h
include/base/constraint/constraint_odom_3D.h
include/base/constraint/constraint_point_2D.h
include/base/constraint/constraint_point_to_line_2D.h
include/base/constraint/constraint_pose_2D.h
include/base/constraint/constraint_pose_3D.h
include/base/constraint/constraint_quaternion_absolute.h
include/base/constraint/constraint_relative_2D_analytic.h
include/base/feature/feature_corner_2D.h
include/base/feature/feature_GPS_fix.h
include/base/feature/feature_GPS_pseudorange.h
include/base/feature/feature_IMU.h
include/base/feature/feature_odom_2D.h
include/base/feature/feature_polyline_2D.h
include/base/IMU_tools.h
include/base/landmark/landmark_corner_2D.h
include/base/landmark/landmark_container.h
include/base/landmark/landmark_line_2D.h
include/base/landmark/landmark_polyline_2D.h
include/base/local_parametrization_polyline_extreme.h
include/base/processor/processor_frame_nearest_neighbor_filter.h
include/base/processor/processor_IMU.h
test/processor_IMU_UnitTester.h
include/base/processor/processor_odom_2D.h
include/base/processor/processor_odom_3D.h
include/base/processor/processor_tracker_feature_dummy.h
include/base/processor/processor_tracker_landmark_dummy.h
include/base/sensor/sensor_camera.h
include/base/sensor/sensor_GPS.h
include/base/sensor/sensor_GPS_fix.h
include/base/sensor/sensor_IMU.h
include/base/sensor/sensor_odom_2D.h
include/base/sensor/sensor_odom_3D.h
    )
  SET(HDRS_CAPTURE
include/base/capture/capture_GPS_fix.h
include/base/capture/capture_IMU.h
include/base/capture/capture_odom_2D.h
include/base/capture/capture_odom_3D.h
include/base/capture/capture_velocity.h
include/base/capture/capture_wheel_joint_position.h
    )
  SET(HDRS_CONSTRAINT
include/base/constraint/constraint_autodiff_trifocal.h
include/base/constraint/constraint_autodiff_distance_3D.h
include/base/constraint/constraint_AHP.h
include/base/constraint/constraint_block_absolute.h
include/base/constraint/constraint_container.h
include/base/constraint/constraint_corner_2D.h
include/base/constraint/constraint_diff_drive.h
include/base/constraint/constraint_epipolar.h
include/base/constraint/constraint_IMU.h
include/base/constraint/constraint_fix_bias.h
include/base/constraint/constraint_GPS_2D.h
include/base/constraint/constraint_GPS_pseudorange_3D.h
include/base/constraint/constraint_GPS_pseudorange_2D.h
include/base/constraint/constraint_odom_2D.h
include/base/constraint/constraint_odom_2D_analytic.h
include/base/constraint/constraint_odom_3D.h
include/base/constraint/constraint_point_2D.h
include/base/constraint/constraint_point_to_line_2D.h
include/base/constraint/constraint_pose_2D.h
include/base/constraint/constraint_pose_3D.h
include/base/constraint/constraint_quaternion_absolute.h
include/base/constraint/constraint_relative_2D_analytic.h
    )
  SET(HDRS_FEATURE
include/base/feature/feature_corner_2D.h
include/base/feature/feature_diff_drive.h
include/base/feature/feature_GPS_fix.h
include/base/feature/feature_GPS_pseudorange.h
include/base/feature/feature_IMU.h
include/base/feature/feature_odom_2D.h
include/base/feature/feature_polyline_2D.h
    )
  SET(HDRS_LANDMARK
include/base/landmark/landmark_match.h
include/base/landmark/landmark_corner_2D.h
include/base/landmark/landmark_container.h
include/base/landmark/landmark_line_2D.h
include/base/landmark/landmark_polyline_2D.h
    )
  SET(HDRS_PROCESSOR
include/base/processor/processor_capture_holder.h
include/base/processor/processor_diff_drive.h
include/base/processor/processor_frame_nearest_neighbor_filter.h
include/base/processor/processor_IMU.h
include/base/processor/processor_odom_2D.h
include/base/processor/processor_odom_3D.h
include/base/processor/processor_tracker_feature_dummy.h
include/base/processor/processor_tracker_landmark.h
include/base/processor/processor_tracker_landmark_dummy.h
    )
  SET(HDRS_SENSOR
include/base/sensor/sensor_camera.h
include/base/sensor/sensor_diff_drive.h
include/base/sensor/sensor_GPS.h
include/base/sensor/sensor_GPS_fix.h
include/base/sensor/sensor_IMU.h
include/base/sensor/sensor_odom_2D.h
include/base/sensor/sensor_odom_3D.h
    )
SET(HDRS_SOLVER
include/base/solver/solver_manager.h
)
# [Add generic derived header before this line]

SET(HDRS_DTASSC
include/base/track_matrix.h
include/base/association/association_solver.h
include/base/association/association_node.h
include/base/association/association_tree.h
include/base/association/association_nnls.h
    )

SET(HDRS_CORE 
include/base/capture/capture_base.h
include/base/capture/capture_buffer.h
include/base/capture/capture_pose.h
include/base/capture/capture_void.h
include/base/constraint/constraint_analytic.h
include/base/constraint/constraint_autodiff.h
include/base/constraint/constraint_base.h
include/base/processor/processor_factory.h
include/base/feature/feature_base.h
include/base/feature/feature_match.h
include/base/feature/feature_pose.h
include/base/frame_base.h
include/base/hardware_base.h
include/base/landmark/landmark_base.h
include/base/local_parametrization_angle.h
include/base/local_parametrization_base.h
include/base/local_parametrization_homogeneous.h
include/base/local_parametrization_quaternion.h
include/base/processor/processor_logging.h
include/base/map_base.h
include/base/motion_buffer.h
include/base/node_base.h
include/base/problem.h
include/base/processor/processor_base.h
include/base/processor/processor_factory.h
include/base/processor/processor_loopclosure_base.h
include/base/processor/processor_motion.h
include/base/processor/processor_tracker_feature.h
include/base/processor/processor_tracker.h
include/base/rotations.h
include/base/sensor/sensor_base.h
include/base/processor/processor_factory.h
include/base/singleton.h
include/base/state_angle.h
include/base/state_block.h
include/base/state_homogeneous_3D.h
include/base/state_quaternion.h
include/base/three_D_tools.h
include/base/time_stamp.h
include/base/track_matrix.h
include/base/trajectory_base.h
include/base/wolf.h
  )

#SOURCES
SET(SRCS_CORE 
src/capture/capture_base.cpp
src/capture/capture_pose.cpp
src/capture/capture_void.cpp
src/constraint/constraint_analytic.cpp
src/constraint/constraint_base.cpp
src/feature/feature_base.cpp
src/feature/feature_pose.cpp
src/frame_base.cpp
src/hardware_base.cpp
src/landmark/landmark_base.cpp
src/local_parametrization_base.cpp
src/local_parametrization_homogeneous.cpp
src/local_parametrization_quaternion.cpp
src/map_base.cpp
src/motion_buffer.cpp
src/node_base.cpp
src/problem.cpp
src/processor/processor_base.cpp
src/processor/processor_loopclosure_base.cpp
src/processor/processor_motion.cpp
src/processor/processor_tracker.cpp
src/sensor/sensor_base.cpp
src/state_block.cpp
src/time_stamp.cpp
src/track_matrix.cpp
src/trajectory_base.cpp
  )

SET(SRCS_BASE
src/capture/capture_motion.cpp
src/processor/processor_capture_holder.cpp
# examples/test_processor_tracker_landmark.cpp
    )

SET(SRCS
src/local_parametrization_polyline_extreme.cpp
test/processor_IMU_UnitTester.cpp
  )
  SET(SRCS_CAPTURE
src/capture/capture_GPS_fix.cpp
src/capture/capture_IMU.cpp
src/capture/capture_odom_2D.cpp
src/capture/capture_odom_3D.cpp
src/capture/capture_velocity.cpp
src/capture/capture_wheel_joint_position.cpp
    )
  SET(SRCS_FEATURE
src/feature/feature_corner_2D.cpp
src/feature/feature_diff_drive.cpp
src/feature/feature_GPS_fix.cpp
src/feature/feature_GPS_pseudorange.cpp
src/feature/feature_IMU.cpp
src/feature/feature_odom_2D.cpp
src/feature/feature_polyline_2D.cpp
    )
  SET(SRCS_LANDMARK
src/landmark/landmark_corner_2D.cpp
src/landmark/landmark_container.cpp
src/landmark/landmark_line_2D.cpp
src/landmark/landmark_polyline_2D.cpp
    )
  SET(SRCS_PROCESSOR
src/processor/processor_frame_nearest_neighbor_filter.cpp
src/processor/processor_diff_drive.cpp
src/processor/processor_IMU.cpp
src/processor/processor_odom_2D.cpp
src/processor/processor_odom_3D.cpp
src/processor/processor_tracker_feature_dummy.cpp
src/processor/processor_tracker_landmark_dummy.cpp
    )
  SET(SRCS_SENSOR
src/sensor/sensor_camera.cpp
src/sensor/sensor_diff_drive.cpp
src/sensor/sensor_GPS.cpp
src/sensor/sensor_GPS_fix.cpp
src/sensor/sensor_IMU.cpp
src/sensor/sensor_odom_2D.cpp
src/sensor/sensor_odom_3D.cpp
    )
SET(SRCS_DTASSC
src/association/association_solver.cpp
src/association/association_node.cpp
src/association/association_tree.cpp
src/association/association_nnls.cpp
    )
SET(SRCS_SOLVER
src/solver/solver_manager.cpp
)
#OPTIONALS
#optional HDRS and SRCS
IF (Ceres_FOUND)
    SET(HDRS_WRAPPER
include/base/solver_suitesparse/sparse_utils.h
include/base/solver/solver_manager.h
include/base/ceres_wrapper/ceres_manager.h
        #ceres_wrapper/qr_manager.h
include/base/ceres_wrapper/cost_function_wrapper.h
include/base/ceres_wrapper/create_numeric_diff_cost_function.h
include/base/ceres_wrapper/local_parametrization_wrapper.h 
        )
    SET(SRCS_WRAPPER
src/solver/solver_manager.cpp
src/ceres_wrapper/ceres_manager.cpp
        #ceres_wrapper/qr_manager.cpp
src/ceres_wrapper/local_parametrization_wrapper.cpp 
        )
ELSE(Ceres_FOUND)
    SET(HDRS_WRAPPER)
    SET(SRCS_WRAPPER)
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    SET(HDRS ${HDRS}
include/base/capture/capture_laser_2D.h
include/base/sensor/sensor_laser_2D.h
include/base/processor/processor_tracker_feature_corner.h
include/base/processor/processor_tracker_landmark_corner.h
include/base/processor/processor_tracker_landmark_polyline.h
        )
    SET(SRCS ${SRCS}
# examples/test_capture_laser_2D.cpp
src/sensor/sensor_laser_2D.cpp
src/processor/processor_tracker_feature.cpp
src/processor/processor_tracker_feature_corner.cpp
src/processor/processor_tracker_landmark_corner.cpp
src/processor/processor_tracker_landmark_polyline.cpp
src/processor/processor_tracker_landmark.cpp
src/processor/processor_tracker_landmark_dummy.cpp
        )
ENDIF(laser_scan_utils_FOUND)

IF (raw_gps_utils_FOUND)
    SET(HDRS ${HDRS}
include/base/capture/capture_GPS.h
include/base/processor/processor_GPS.h
        )
    SET(SRCS ${SRCS}
src/capture/capture_GPS.cpp
src/processor/processor_GPS.cpp
        )
ENDIF(raw_gps_utils_FOUND)

# Vision
IF (vision_utils_FOUND)
    SET(HDRS ${HDRS}
include/base/capture/capture_image.h
include/base/feature/feature_point_image.h
include/base/landmark/landmark_AHP.h
include/base/processor/processor_params_image.h
include/base/processor/processor_tracker_feature_image.h
include/base/processor/processor_tracker_landmark_image.h
        )
    SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
src/processor/processor_tracker_feature_trifocal.h
    )
  SET(HDRS_LANDMARK ${HDRS_LANDMARK}
src/landmark/landmark_point_3D.h
    )
    SET(SRCS ${SRCS}
src/capture/capture_image.cpp
src/feature/feature_point_image.cpp
src/landmark/landmark_AHP.cpp
src/processor/processor_tracker_feature_image.cpp
# examples/test_processor_tracker_landmark_image.cpp
        )
    SET(SRCS_LANDMARK ${SRCS_LANDMARK}
src/landmark/landmark_point_3D.cpp
    )
    SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
src/processor/processor_tracker_feature_trifocal.cpp
    )
ENDIF(vision_utils_FOUND)

#SUBDIRECTORIES
add_subdirectory(hello_wolf)
IF (cereal_FOUND)
  ADD_SUBDIRECTORY(serialization/cereal)
ENDIF(cereal_FOUND)

IF (Suitesparse_FOUND)
    #DOES NOTHING?!    
    #ADD_SUBDIRECTORY(solver_suitesparse)
ENDIF(Suitesparse_FOUND)
# LEAVE YAML FILES ALWAYS IN THE LAST POSITION !!
IF(YAMLCPP_FOUND)
    # headers
    SET(HDRS ${HDRS}
include/base/yaml/yaml_conversion.h
        )
    SET(HDRS_YAML ${HDRS_YAML}
include/base/yaml/yaml_conversion.h
        )

    # sources
    SET(SRCS ${SRCS}
src/yaml/processor_odom_3D_yaml.cpp
src/yaml/processor_IMU_yaml.cpp
src/yaml/sensor_camera_yaml.cpp
src/yaml/sensor_odom_3D_yaml.cpp
src/yaml/sensor_IMU_yaml.cpp
        )
    IF(laser_scan_utils_FOUND)
        SET(SRCS ${SRCS}
src/yaml/sensor_laser_2D_yaml.cpp
            )
    ENDIF(laser_scan_utils_FOUND)
    IF(vision_utils_FOUND)
        SET(SRCS ${SRCS}
src/yaml/processor_image_yaml.cpp
src/yaml/processor_tracker_feature_trifocal_yaml.cpp
            )
    ENDIF(vision_utils_FOUND)
ENDIF(YAMLCPP_FOUND)

# create the shared library
add_library(${PROJECT_NAME}_core SHARED
                 ${SRCS_CORE}
                 ${SRCS_BASE} 
                )
ADD_LIBRARY(${PROJECT_NAME} 
            SHARED 
            ${SRCS_BASE} 
            ${SRCS}
            ${SRCS_CAPTURE}
            ${SRCS_CONSTRAINT}
            ${SRCS_FEATURE}
            ${SRCS_LANDMARK}
            ${SRCS_PROCESSOR}
            ${SRCS_SENSOR}
            #${SRCS_DTASSC} 
            ${SRCS_SOLVER}
            ${SRCS_WRAPPER}
            )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PROJECT_NAME}_core ${CMAKE_THREAD_LIBS_INIT})

#Link the created libraries
#=============================================================
IF (Ceres_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES})
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${laser_scan_utils_LIBRARY})
ENDIF (laser_scan_utils_FOUND)

IF (raw_gps_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${raw_gps_utils_LIBRARY})
ENDIF (raw_gps_utils_FOUND)

IF (OPENCV_FOUND)
   	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
	IF (vision_utils_FOUND)
    	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${vision_utils_LIBRARY})
	ENDIF (vision_utils_FOUND)
ENDIF (OPENCV_FOUND)

IF (YAMLCPP_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YAMLCPP_LIBRARY})
ENDIF (YAMLCPP_FOUND)

IF (GLOG_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GLOG_LIBRARY})
ENDIF (GLOG_FOUND)
IF (GLOG_FOUND)
    IF(BUILD_TESTS)
        MESSAGE("Building tests.")
        add_subdirectory(test)
    ENDIF(BUILD_TESTS)
ENDIF (GLOG_FOUND)

install(TARGETS ${PROJECT_NAME}_core DESTINATION lib/iri-algorithms EXPORT ${PROJECT_NAME}_core-targets)
install(EXPORT ${PROJECT_NAME}_core-targets DESTINATION lib/iri-algorithms)
#install library

#=============================================================
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)

install(EXPORT ${PROJECT_NAME}Targets DESTINATION lib/cmake/${PROJECT_NAME})
#install headers
INSTALL(FILES ${HDRS_BASE}
    DESTINATION include/iri-algorithms/wolf/base)
INSTALL(FILES ${HDRS_DTASSC}
   DESTINATION include/iri-algorithms/wolf/base/association)
INSTALL(FILES ${HDRS_CAPTURE}
    DESTINATION include/iri-algorithms/wolf/base/capture)
INSTALL(FILES ${HDRS_CONSTRAINT}
    DESTINATION include/iri-algorithms/wolf/base/constraint)
INSTALL(FILES ${HDRS_FEATURE}
    DESTINATION include/iri-algorithms/wolf/base/feature)
INSTALL(FILES ${HDRS_SENSOR}
    DESTINATION include/iri-algorithms/wolf/base/sensor)
INSTALL(FILES ${HDRS_PROCESSOR}
    DESTINATION include/iri-algorithms/wolf/base/processor)
INSTALL(FILES ${HDRS_WRAPPER}
    DESTINATION include/iri-algorithms/wolf/base/ceres_wrapper)
INSTALL(FILES ${HDRS_SOLVER_SUITESPARSE}
   DESTINATION include/iri-algorithms/wolf/base/solver_suitesparse)
INSTALL(FILES ${HDRS_SOLVER}
    DESTINATION include/iri-algorithms/wolf/base/solver)
INSTALL(FILES ${HDRS_SERIALIZATION}
    DESTINATION include/iri-algorithms/wolf/base/serialization)
INSTALL(FILES ${HDRS_YAML}
    DESTINATION include/iri-algorithms/wolf/base/yaml)
INSTALL(FILES "${CMAKE_SOURCE_DIR}/cmake_modules/Findwolf.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}")

#install Find*.cmake
configure_file("${CMAKE_SOURCE_DIR}/cmake_modules/wolfConfig.cmake"
               "${CMAKE_BINARY_DIR}/wolfConfig.cmake" @ONLY)

INSTALL(FILES "${WOLF_CONFIG_DIR}/config.h"
DESTINATION include/iri-algorithms/wolf/internal)

INSTALL(FILES "${CMAKE_BINARY_DIR}/wolfConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")

INSTALL(DIRECTORY ${SPDLOG_INCLUDE_DIRS} DESTINATION "include/iri-algorithms/")

export(PACKAGE ${PROJECT_NAME})

#-END_SRC --------------------------------------------------------------------------------------------------------------------------------
FIND_PACKAGE(Doxygen)

FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
IF (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
ELSE (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)

IF (UNIX)
  SET(CPACK_PACKAGE_FILE_NAME "iri-${PROJECT_NAME}-dev-${CPACK_PACKAGE_VERSION}${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  SET(CPACK_PACKAGE_NAME "iri-${PROJECT_NAME}-dev")
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "...Enter something here...")
  SET(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
  SET(CPACK_GENERATOR "DEB")
  SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "galenya - labrobotica@iri.upc.edu")
  SET(CPACK_SET_DESTDIR "ON")  # Necessary because of the absolute install paths
  INCLUDE(CPack)
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "packaging only implemented in unix"
    TARGET  uninstall)
ENDIF(UNIX)

