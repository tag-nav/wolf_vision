INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# Forward var to parent scope

SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.h 
  )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.cpp 
  )

#=========================================
# Add after these lines the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  

IF (vision_utils_FOUND)
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.h
  )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.cpp
  )
ENDIF (vision_utils_FOUND)

#=========================================
#=========================================
  

# These lines always at the end
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR} PARENT_SCOPE)
SET(SRCS_PROCESSOR ${SRCS_PROCESSOR} PARENT_SCOPE)
