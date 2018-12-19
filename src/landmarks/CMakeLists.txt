INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#=========================================
#=========================================
# Add in this section the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  



#=========================================
#=========================================
  
SET(HDRS_LANDMARK ${HDRS_LANDMARK}
  ${CMAKE_CURRENT_SOURCE_DIR}/landmark_point_3D.h
  )

SET(SRCS_LANDMARK ${SRCS_LANDMARK}
  ${CMAKE_CURRENT_SOURCE_DIR}/landmark_point_3D.cpp
  )

# Forward var to parent scope
# These lines always at the end
SET(HDRS_LANDMARK ${HDRS_LANDMARK}  PARENT_SCOPE)
SET(SRCS_LANDMARK ${SRCS_LANDMARK}  PARENT_SCOPE)