#edit the following line to add the librarie's header files
FIND_PATH(
    wolfvision_INCLUDE_DIRS
    NAMES vision.found
    PATHS /usr/local/include/iri-algorithms/wolf/plugin_vision)
IF(wolfvision_INCLUDE_DIRS)
  MESSAGE("Found vision include dirs: ${wolfvision_INCLUDE_DIRS}")
ELSE(wolfvision_INCLUDE_DIRS)
  MESSAGE("Couldn't find vision include dirs")
ENDIF(wolfvision_INCLUDE_DIRS)

FIND_LIBRARY(
    wolfvision_LIBRARIES
    NAMES libwolfvision.so libwolfvision.dylib
    PATHS /usr/local/lib)
IF(wolfvision_LIBRARIES)
  MESSAGE("Found vision lib: ${wolfvision_LIBRARIES}")
ELSE(wolfvision_LIBRARIES)
  MESSAGE("Couldn't find wolf vision lib")
ENDIF(wolfvision_LIBRARIES)

IF (wolfvision_INCLUDE_DIRS AND wolfvision_LIBRARIES)
   SET(wolfvision_FOUND TRUE)
 ELSE(wolfvision_INCLUDE_DIRS AND wolfvision_LIBRARIES)
   set(wolfvision_FOUND FALSE)
ENDIF (wolfvision_INCLUDE_DIRS AND wolfvision_LIBRARIES)

IF (wolfvision_FOUND)
   IF (NOT wolfvision_FIND_QUIETLY)
      MESSAGE(STATUS "Found vision: ${wolfvision_LIBRARIES}")
   ENDIF (NOT wolfvision_FIND_QUIETLY)
ELSE (wolfvision_FOUND)
   IF (wolfvision_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find wolf vision")
   ENDIF (wolfvision_FIND_REQUIRED)
ENDIF (wolfvision_FOUND)


macro(wolf_report_not_found REASON_MSG)
  set(wolfvision_FOUND FALSE)
  unset(wolfvision_INCLUDE_DIRS)
  unset(wolfvision_LIBRARIES)

  # Reset the CMake module path to its state when this script was called.
  set(CMAKE_MODULE_PATH ${CALLERS_CMAKE_MODULE_PATH})

  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by
  # FindPackage() use the camelcase library name, not uppercase.
  if (wolfvision_FIND_QUIETLY)
    message(STATUS "Failed to find wolfvision- " ${REASON_MSG} ${ARGN})
<<<<<<< HEAD
  elseif(wolfvision_FIND_REQUIRED)
=======
  elseif (wolfvision_FIND_REQUIRED)
>>>>>>> devel
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

# Now we gather all the required dependencies for Wolf Laser

FIND_PACKAGE(OpenCV REQUIRED)
list(APPEND wolfvision_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND wolfvision_LIBRARIES ${OpenCV_LIBS})

#Making sure wolf is looked for
if(NOT wolf_FOUND)
  FIND_PACKAGE(wolfcore REQUIRED)

  #We reverse in order to insert at the start
  list(REVERSE wolfvision_INCLUDE_DIRS)
  list(APPEND wolfvision_INCLUDE_DIRS ${wolfcore_INCLUDE_DIRS})
  list(REVERSE wolfvision_INCLUDE_DIRS)

  list(REVERSE wolfvision_LIBRARIES)
  list(APPEND wolfvision_LIBRARIES ${wolfcore_LIBRARIES})
  list(REVERSE wolfvision_LIBRARIES)
endif()

# provide both INCLUDE_DIR and INCLUDE_DIRS
SET(wolfvision_INCLUDE_DIR ${wolfvision_INCLUDE_DIRS})
# provide both LIBRARY and LIBRARIES 
SET(wolfvision_LIBRARY ${wolfvision_LIBRARIES})