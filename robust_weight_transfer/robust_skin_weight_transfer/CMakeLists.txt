cmake_minimum_required(VERSION 3.10)
project(robust_weight_transfer)

add_subdirectory(cmake)

add_definitions(-D_USE_MATH_DEFINES)
add_definitions(-DIGL_PARALLEL_FOR_FORCE_SERIAL)

set(PROOT ${CMAKE_SOURCE_DIR})

add_sandbox_program_at(robust_weight_transfer ${PROOT}
  robust_weight_transfer.cpp
  test_robust_weight_transfer.cpp
  thirdparty/nonmanifold-laplacian/src/bubble_offset.cpp
  thirdparty/nonmanifold-laplacian/src/point_cloud_utilities.cpp
  thirdparty/nonmanifold-laplacian/src/utils.hpp
  thirdparty/nonmanifold-laplacian/src/process_mesh.cpp
)

target_include_directories(robust_weight_transfer PRIVATE thirdparty/eigen thirdparty/libigl/include thirdparty/nonmanifold-laplacian/include thirdparty/nonmanifold-laplacian/include thirdparty/geometry_central/include)

add_subdirectory(${PROOT}/thirdparty/geometry_central)
add_subdirectory(${PROOT}/thirdparty/libigl)

target_link_libraries(robust_weight_transfer PRIVATE geometry-central igl::core)
