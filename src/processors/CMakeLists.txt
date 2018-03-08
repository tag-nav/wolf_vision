INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.h
  PARENT_SCOPE)

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.cpp
  PARENT_SCOPE)

