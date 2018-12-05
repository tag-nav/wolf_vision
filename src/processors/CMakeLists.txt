INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.h 
  )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.cpp 
  )

IF (vision_utils_FOUND)
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_landmark_apriltag.h
  )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_landmark_apriltag.cpp
  )
ENDIF (vision_utils_FOUND)

IF (OPENCV_FOUND AND APRILTAG_LIBRARY)
  SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_landmark_apriltag.h
      )
  SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_landmark_apriltag.cpp
  )
ENDIF(OPENCV_FOUND AND APRILTAG_LIBRARY)

# These lines always at the end
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR} PARENT_SCOPE)
SET(SRCS_PROCESSOR ${SRCS_PROCESSOR} PARENT_SCOPE)
