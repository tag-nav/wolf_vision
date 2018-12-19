INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#=========================================
#=========================================
# Add in this section the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  



#=========================================
#=========================================

SET(HDRS_CAPTURE ${HDRS_CAPTURE}
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_velocity.h
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_wheel_joint_position.h 
  )

SET(SRCS_CAPTURE ${SRCS_CAPTURE}
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_velocity.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/capture_wheel_joint_position.cpp 
  )

# Forward var to parent scope
# These lines always at the end
SET(HDRS_CAPTURE ${HDRS_CAPTURE}  PARENT_SCOPE)
SET(SRCS_CAPTURE ${SRCS_CAPTURE}  PARENT_SCOPE)
