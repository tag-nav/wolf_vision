INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
# INCLUDE_DIRECTORIES(/home/jcasals/workspace/wolf/src)
# link_directories(/home/jcasals/workspace/wolf/lib)

# ADD_EXECUTABLE(hello_plugin hello_plugin.cpp sensor_odom_2D.cpp processor_odom_2D.cpp)
ADD_EXECUTABLE(hello_plugin hello_plugin.cpp)
ADD_EXECUTABLE(params_autoconf params_autoconf.cpp)
target_link_libraries(hello_plugin class_loader boost_system console_bridge wolf yaml-cpp ${CERES_LIBRARIES})
target_link_libraries(params_autoconf class_loader boost_system console_bridge wolf yaml-cpp )
# These lines always at the end
SET(HDRS_PLUGIN ${HDRS_PLUGIN}   PARENT_SCOPE    )
SET(SRCS_PLUGIN ${SRCS_PLUGIN}    PARENT_SCOPE    )


