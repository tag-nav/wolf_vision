INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(HDRS_SERIALIZATION ${HDRS_SERIALIZATION}
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_eigen_core.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_eigen_geometry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_eigen_sparse.h

    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_local_parametrization_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_local_parametrization_homogeneous.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_local_parametrization_quaternion.h

    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_node_base.h

    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_sensor_intrinsic_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_sensor_odom2d_intrinsic.h

    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_processor_params_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_processor_odom2d_params.h
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_processor_odom3d_params.h

    ${CMAKE_CURRENT_SOURCE_DIR}/serialization_time_stamp.h

    ${CMAKE_CURRENT_SOURCE_DIR}/archives.h
    ${CMAKE_CURRENT_SOURCE_DIR}/io.h

    PARENT_SCOPE)
