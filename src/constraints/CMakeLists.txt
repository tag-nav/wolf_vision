INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#=========================================
#=========================================
# Add in this section the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  



#=========================================
#=========================================

SET(HDRS_CONSTRAINT ${HDRS_CONSTRAINT}
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_autodiff_distance_3D.h
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_autodiff_trifocal.h
  ${CMAKE_CURRENT_SOURCE_DIR}/constraint_diff_drive.h 
  )

SET(SRCS_CONSTRAINT ${SRCS_CONSTRAINT} 
  )

# Forward var to parent scope
# These lines always at the end
SET(HDRS_CONSTRAINT ${HDRS_CONSTRAINT}  PARENT_SCOPE)
SET(SRCS_CONSTRAINT ${SRCS_CONSTRAINT}   PARENT_SCOPE)

  