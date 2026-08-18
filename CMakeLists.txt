cmake_minimum_required(VERSION 3.20)
project(waypoint LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_library(waypoint STATIC
  src/packet.cpp
  src/lsdb.cpp
  src/spf.cpp
  src/neighbor.cpp
  src/router.cpp
  src/topology.cpp
  src/sim.cpp
  src/analysis.cpp
  src/udp.cpp
)
target_include_directories(waypoint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MSVC)
  target_compile_options(waypoint PRIVATE /W4)
else()
  target_compile_options(waypoint PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  target_link_libraries(waypoint PUBLIC ws2_32)
endif()

add_executable(waypoint-demo apps/demo.cpp)
target_link_libraries(waypoint-demo PRIVATE waypoint)

add_executable(waypoint-bench apps/bench.cpp)
target_link_libraries(waypoint-bench PRIVATE waypoint)

add_executable(waypoint-live apps/live.cpp)
target_link_libraries(waypoint-live PRIVATE waypoint)

enable_testing()

add_executable(waypoint-tests
  tests/check.cpp
  tests/test_packet.cpp
  tests/test_lsdb.cpp
  tests/test_spf.cpp
  tests/test_neighbor.cpp
  tests/test_topology.cpp
  tests/test_sim.cpp
  tests/test_analysis.cpp
  tests/test_udp.cpp
)
target_link_libraries(waypoint-tests PRIVATE waypoint)
if(MSVC)
  target_compile_options(waypoint-tests PRIVATE /W4)
else()
  target_compile_options(waypoint-tests PRIVATE -Wall -Wextra)
endif()

foreach(suite packet lsdb spf neighbor topology sim analysis udp)
  add_test(NAME ${suite} COMMAND waypoint-tests ${suite})
endforeach()

# `cmake --build build --target demo` runs the demonstration end to end.
add_custom_target(demo
  COMMAND waypoint-demo --out ${CMAKE_CURRENT_BINARY_DIR}/dot
  DEPENDS waypoint-demo
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  USES_TERMINAL
  COMMENT "Running the Waypoint convergence demonstration")
