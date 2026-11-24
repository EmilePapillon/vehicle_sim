cmake_minimum_required(VERSION 3.20)
project(VehicleCore VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(VD_BUILD_SHARED "Also build libvehiclecore as a shared library (Python ctypes, plugins)" ON)
option(VD_BUILD_TOOLS "Build the vdsim command-line tool" ON)
option(VD_BUILD_TESTS "Build the test suite" ON)

set(VD_CORE_SOURCES
  core/src/json.cpp
  core/src/tire.cpp
  core/src/suspension.cpp
  core/src/drivetrain.cpp
  core/src/params.cpp
  core/src/vehicle.cpp
  core/src/driver.cpp
  core/src/scenario.cpp
  capi/vd_capi.cpp)

if(MSVC)
  set(VD_WARNINGS /W4)
else()
  set(VD_WARNINGS -Wall -Wextra -Wpedantic -Wshadow)
endif()

# Static library: what game engines (e.g. an Unreal ThirdParty module) link against.
add_library(vehiclecore STATIC ${VD_CORE_SOURCES})
target_include_directories(vehiclecore PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/capi/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(vehiclecore PRIVATE ${VD_WARNINGS})
find_package(Threads REQUIRED)
target_link_libraries(vehiclecore PUBLIC Threads::Threads)

if(VD_BUILD_SHARED)
  add_library(vehiclecore_shared SHARED ${VD_CORE_SOURCES})
  set_target_properties(vehiclecore_shared PROPERTIES OUTPUT_NAME vehiclecore)
  target_include_directories(vehiclecore_shared PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/capi/include>)
  target_compile_options(vehiclecore_shared PRIVATE ${VD_WARNINGS})
  target_link_libraries(vehiclecore_shared PUBLIC Threads::Threads)
endif()

if(VD_BUILD_TOOLS)
  add_executable(vdsim cli/main.cpp)
  target_link_libraries(vdsim PRIVATE vehiclecore)
  target_compile_options(vdsim PRIVATE ${VD_WARNINGS})
endif()

if(VD_BUILD_TESTS)
  enable_testing()
  add_executable(vdtests tests/test_main.cpp tests/test_physics.cpp tests/test_components.cpp)
  target_link_libraries(vdtests PRIVATE vehiclecore)
  target_compile_options(vdtests PRIVATE ${VD_WARNINGS})
  add_test(NAME vdtests COMMAND vdtests)
  if(VD_BUILD_TOOLS)
    add_test(NAME vdsim_scenarios
      COMMAND vdsim run ${CMAKE_CURRENT_SOURCE_DIR}/scenarios/step_steer.json
                        ${CMAKE_CURRENT_SOURCE_DIR}/scenarios/arb_sweep.json -q -o ${CMAKE_CURRENT_BINARY_DIR}/scenario_out)
  endif()
endif()

include(GNUInstallDirs)
install(TARGETS vehiclecore EXPORT VehicleCoreTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY core/include/ capi/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(VD_BUILD_SHARED)
  install(TARGETS vehiclecore_shared LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if(VD_BUILD_TOOLS)
  install(TARGETS vdsim RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(EXPORT VehicleCoreTargets NAMESPACE VehicleCore:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/VehicleCore)
