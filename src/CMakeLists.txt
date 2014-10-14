
#Start WOLF build
MESSAGE("Starting WOLF CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#find dependencies. 
FIND_PACKAGE(Ceres QUIET) #Ceres is not required
IF(Ceres_FOUND)
    MESSAGE("Ceres Library FOUND: ceres_wrapper will be built.")
ENDIF(Ceres_FOUND)

#include directories
INCLUDE_DIRECTORIES(.)
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

#headers
SET(HDRS 
    capture_gps.h
    capture.h
    capture_image.h
    correspondence_base.h
    correspondence_global.h
    correspondence_gps_doppler.h
    correspondence_gps_pseudo_range.h
    correspondence_relative.h
    correspondence_slam_landmark.h
    epipolar_constraint.h
    feature_base.h
    frame.h
    gps_doppler.h
    gps_pseudo_range.h
    image_point.h
    node_constrainer.h
    node.h
    node_linked.h
    node_terminus.h
    pin_hole.h
    quaternion_tools.h
    raw_base.h
    sensor_base.h
    state_base.h
    state_error.h
    state_error_imu.h
    state_error_pose.h
    state_error_pqv.h
    state_imu.h
    state_pose.h
    state_pq.h
    state_pqv.h
    state_root_base.h
    time_stamp.h
    trans_pin_hole.h
    trans_sensor.h
    vehicle_base.h
    wolf.h
    wolf_tools.h)
    
#sources
SET(SRCS
    state_base.cpp
#     state_error.cpp
#     state_error_imu.cpp
#     state_error_pose.cpp
#     state_error_pqv.cpp
    state_imu.cpp
    state_pose.cpp
    state_pq.cpp
    state_pqv.cpp
    state_root_base.cpp
    time_stamp.cpp)

#optional HDRS and SRCS
IF (Ceres_FOUND)
    SET(SRCS ${SRCS} ceres_wrapper/ceres_wrapper_numeric.cpp)
    SET(HDRS ${HDRS} ceres_wrapper/ceres_wrapper_numeric.h)
ENDIF(Ceres_FOUND)

# create the shared library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRCS})

#install library
INSTALL(TARGETS ${PROJECT_NAME}
       RUNTIME DESTINATION bin
       LIBRARY DESTINATION lib/iri-algorithms
       ARCHIVE DESTINATION lib/iri-algorithms)
        
#install headers         
INSTALL(FILES ${HDRS} 
       DESTINATION include/iri-algorithms)

INSTALL(FILES ../Findwolf.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

#Build examples & tests
MESSAGE("Building examples and tests.")
ADD_SUBDIRECTORY(examples)
IF (Ceres_FOUND)
    ADD_SUBDIRECTORY(ceres_wrapper/examples)
ENDIF(Ceres_FOUND)


