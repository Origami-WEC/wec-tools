cmake_minimum_required(VERSION 3.20)
project(wec_tools VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_AUTOMOC ON)

find_package(gz-cmake REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz-common REQUIRED COMPONENTS profiler)
find_package(gz-msgs REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-sim REQUIRED COMPONENTS gui)
find_package(gz-gui REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Qml Quick)

message(STATUS "wec_tools: gz-sim ${gz-sim_VERSION}, Qt6 ${Qt6_VERSION}")

# ---------------------------------------------------------------------------
# System plugin: accelerazioni + stato PTO
# ---------------------------------------------------------------------------
add_library(WecDynamics SHARED system/WecDynamics.cc)
target_include_directories(WecDynamics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/system
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(WecDynamics PRIVATE
  gz-sim::gz-sim
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  gz-math::gz-math
  gz-plugin::gz-plugin-register
  gz-common::gz-common
)

# ---------------------------------------------------------------------------
# GUI plugin: WecDashboard (QML)
# ---------------------------------------------------------------------------
qt_add_resources(wec_dashboard_RCC gui/WecDashboard.qrc)

add_library(WecDashboard SHARED gui/WecDashboard.cc ${wec_dashboard_RCC})
target_include_directories(WecDashboard PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/gui
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(WecDashboard PRIVATE
  gz-sim::gui
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  gz-math::gz-math
  gz-plugin::gz-plugin-register
  Qt6::Core
  Qt6::Qml
  Qt6::Quick
)

install(TARGETS WecDynamics WecDashboard LIBRARY DESTINATION lib)
