cmake_minimum_required(VERSION 3.10)
project(robust_weight_transfer)

add_subdirectory(cmake)

add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DIGL_PARALLEL_FOR_FORCE_SERIAL)

option(USE_DOUBLE_PRECISION "Use double precision for real numbers" OFF)
if(USE_DOUBLE_PRECISION)
  add_definitions(-DDOUBLE_PRECISION_REAL_T)
endif()

set(PROOT ${CMAKE_SOURCE_DIR}/..)

add_sandbox_program_at(robust_weight_transfer ${PROOT}
  ${CMAKE_SOURCE_DIR}/test_robust_weight_transfer.cpp
  ${CMAKE_SOURCE_DIR}/thirdparty/nonmanifold-laplacian/src/bubble_offset.cpp
  ${CMAKE_SOURCE_DIR}/thirdparty/nonmanifold-laplacian/src/point_cloud_utilities.cpp
  ${CMAKE_SOURCE_DIR}/thirdparty/nonmanifold-laplacian/src/utils.hpp
  ${CMAKE_SOURCE_DIR}/thirdparty/nonmanifold-laplacian/src/process_mesh.cpp
)

target_include_directories(robust_weight_transfer SYSTEM PRIVATE 
${CMAKE_SOURCE_DIR}/thirdparty/eigen 
${CMAKE_SOURCE_DIR}/thirdparty/libigl/include 
${CMAKE_SOURCE_DIR}/thirdparty/nonmanifold-laplacian/include 
${CMAKE_SOURCE_DIR}/thirdparty/nonmanifold-laplacian/include 
${CMAKE_SOURCE_DIR}/thirdparty/geometry_central/include)

add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/geometry_central)
add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/libigl)

target_link_libraries(robust_weight_transfer PRIVATE geometry-central igl::core)
