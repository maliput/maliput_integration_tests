##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput_integration_tests LANGUAGES C CXX VERSION 3.0.0)

include(${PROJECT_SOURCE_DIR}/cmake/DefaultCFlags.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SanitizersConfig.cmake)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(fmt 4.0.0 EXACT QUIET)
if(NOT ${fmt_FOUND})
  find_package(fmt 6.1.2 REQUIRED)
endif()
find_package(maliput REQUIRED)
find_package(maliput_dragway REQUIRED)
find_package(maliput_multilane REQUIRED)

##############################################################################
# Tests
##############################################################################

if (BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  enable_testing()
  add_subdirectory(test)
  ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)
endif()

##############################################################################
# Docs
##############################################################################

if(BUILD_DOCS)
  message(STATUS "Doxygen generation - Enabled")
  find_package(ament_cmake_doxygen REQUIRED)
  ament_doxygen_generate(doxygen_maliput_integration_tests
    DEPENDENCIES maliput maliput_dragway maliput_multilane
    TEST_ON_WARNS
  )
  add_definitions(-DBUILD_DOCS)
else()
  message(STATUS "Doxygen generation - Disabled")
endif()

ament_package()
