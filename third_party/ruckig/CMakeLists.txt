cmake_minimum_required(VERSION 3.10)

project(ruckig_ros VERSION 0.9.2 LANGUAGES CXX)

include(GNUInstallDirs)

option(BUILD_ONLINE_CLIENT "Build online client to calculate Ruckig Pro trajectories" ON)
option(BUILD_SHARED_LIBS "Build as shared library" ON)
option(EXPOSE_INTERNAL "Expose some internal classes and methods" OFF)

add_library(ruckig_ros
  src/ruckig/brake.cpp
  src/ruckig/position-step1.cpp
  src/ruckig/position-step2.cpp
  src/ruckig/velocity-step1.cpp
  src/ruckig/velocity-step2.cpp
)

target_compile_features(ruckig_ros PUBLIC cxx_std_17)
target_include_directories(ruckig_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(ruckig_ros PUBLIC)


if(EXPOSE_INTERNAL)
  target_compile_definitions(ruckig_ros PUBLIC WITH_EXPOSE_INTERNAL)
endif()


if(BUILD_ONLINE_CLIENT)
  target_include_directories(ruckig_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_compile_definitions(ruckig_ros PUBLIC WITH_ONLINE_CLIENT WITH_SERIALIZATION)
endif()


# Install headers
install(DIRECTORY include/ruckig DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install library
install(TARGETS ruckig_ros
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)