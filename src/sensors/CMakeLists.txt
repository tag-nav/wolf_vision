INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#=========================================
#=========================================
# Add in this section the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  



#=========================================
#=========================================
  
SET(HDRS_SENSOR ${HDRS_SENSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/diff_drive_tools.h
  ${CMAKE_CURRENT_SOURCE_DIR}/diff_drive_tools.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sensor_diff_drive.h 
  )

SET(SRCS_SENSOR ${SRCS_SENSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/sensor_diff_drive.cpp 
  )

# Forward var to parent scope
# These lines always at the end
SET(HDRS_SENSOR ${HDRS_SENSOR}  PARENT_SCOPE)
SET(SRCS_SENSOR ${SRCS_SENSOR}  PARENT_SCOPE)
  