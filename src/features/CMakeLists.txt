INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#=========================================
#=========================================
# Add in this section the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  



#=========================================
#=========================================
  
SET(HDRS_FEATURE ${HDRS_FEATURE}
  ${CMAKE_CURRENT_SOURCE_DIR}/feature_diff_drive.h 
  )

SET(SRCS_FEATURE ${SRCS_FEATURE}
  ${CMAKE_CURRENT_SOURCE_DIR}/feature_diff_drive.cpp 
  )
  
# Forward var to parent scope
# These lines always at the end
SET(HDRS_FEATURE ${HDRS_FEATURE}  PARENT_SCOPE)
SET(SRCS_FEATURE ${SRCS_FEATURE}  PARENT_SCOPE)
  