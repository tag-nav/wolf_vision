#edit the following line to add the librarie's header files
FIND_PATH(
    wolfvision_INCLUDE_DIR
    NAMES vision.found
    PATHS /usr/local/include/iri-algorithms/wolf/plugin_vision)
IF(wolfvision_INCLUDE_DIR)
  MESSAGE("Found vision include dirs: ${wolfvision_INCLUDE_DIR}")
ELSE(wolfvision_INCLUDE_DIR)
  MESSAGE("Couldn't find vision include dirs")
ENDIF(wolfvision_INCLUDE_DIR)

FIND_LIBRARY(
    wolfvision_LIBRARY
    NAMES libwolfvision.so
    PATHS /usr/local/lib/iri-algorithms)
IF(wolfvision_LIBRARY)
  MESSAGE("Found vision lib: ${wolfvision_LIBRARY}")
ELSE(wolfvision_LIBRARY)
  MESSAGE("Couldn't find wolf vision lib")
ENDIF(wolfvision_LIBRARY)

IF (wolfvision_INCLUDE_DIR AND wolfvision_LIBRARY)
   SET(wolfvision_FOUND TRUE)
 ELSE(wolfvision_INCLUDE_DIR AND wolfvision_LIBRARY)
   set(wolfvision_FOUND FALSE)
ENDIF (wolfvision_INCLUDE_DIR AND wolfvision_LIBRARY)

IF (wolfvision_FOUND)
   IF (NOT wolfvision_FIND_QUIETLY)
      MESSAGE(STATUS "Found vision: ${wolfvision_LIBRARY}")
   ENDIF (NOT wolfvision_FIND_QUIETLY)
ELSE (wolfvision_FOUND)
   IF (wolfvision_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find wolf vision")
   ENDIF (wolfvision_FIND_REQUIRED)
ENDIF (wolfvision_FOUND)


macro(wolf_report_not_found REASON_MSG)
  set(wolfvision_FOUND FALSE)
  unset(wolfvision_INCLUDE_DIR)
  unset(wolfvision_LIBRARIES)

  # Reset the CMake module path to its state when this script was called.
  set(CMAKE_MODULE_PATH ${CALLERS_CMAKE_MODULE_PATH})

  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by
  # FindPackage() use the camelcase library name, not uppercase.
  if (wolfvision_FIND_QUIETLY)
    message(STATUS "Failed to find wolfvision- " ${REASON_MSG} ${ARGN})
  else (wolfvision_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find wolfvision - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use SEND_ERROR which emits an error
    # that prevents generation, but continues configuration.
    message(SEND_ERROR "Failed to find wolfvision - " ${REASON_MSG} ${ARGN})
  endif ()
  return()
endmacro(wolf_report_not_found)

if(NOT wolfvision_FOUND)
  wolf_report_not_found("Something went wrong while setting up wolf vision.")
endif(NOT wolfvision_FOUND)
# Set the include directories for wolf (itself).
set(wolfvision_FOUND TRUE)