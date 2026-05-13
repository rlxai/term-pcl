cmake_minimum_required(VERSION 3.16)
project(term_pcl VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(MPI COMPONENTS C QUIET)
find_package(PCL REQUIRED COMPONENTS common io filters)

option(TERM_PCL_FETCH_FTXUI "Download FTXUI with FetchContent when system package is unavailable" ON)

find_package(ftxui 5 QUIET)
if(NOT ftxui_FOUND)
  if(NOT TERM_PCL_FETCH_FTXUI)
    message(FATAL_ERROR "ftxui >= 5 not found. Install the system package or configure with -DTERM_PCL_FETCH_FTXUI=ON.")
  endif()
  include(FetchContent)
  set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
    GIT_TAG v5.0.0
  )
  FetchContent_MakeAvailable(ftxui)
endif()

add_library(term_pcl_core
  src/cloud_provider.cpp
  src/color_map.cpp
  src/term_pcl/chunk_cache.cpp
  src/term_pcl/chunk_loader.cpp
  src/term_pcl/cloud_optimizer.cpp
  src/term_pcl/hierarchical_lod.cpp
  src/term_pcl/lod_cloud_provider.cpp
  src/term_pcl/point_cloud_loader.cpp
  src/term_pcl/spatial_lod.cpp
  src/term_pcl/termcloud_index.cpp
  src/terminal_ui.cpp
  src/visualizer.cpp
)

target_include_directories(term_pcl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(term_pcl_core PUBLIC cxx_std_17)
target_link_libraries(term_pcl_core PUBLIC
  ${PCL_LIBRARIES}
  ftxui::screen
  ftxui::dom
  ftxui::component
)
target_compile_definitions(term_pcl_core PUBLIC ${PCL_DEFINITIONS})

add_executable(term-pcl src/term_pcl/main.cpp)
target_link_libraries(term-pcl PRIVATE term_pcl_core)

install(TARGETS term-pcl
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  add_executable(test_color_map tests/test_color_map.cpp)
  target_link_libraries(test_color_map PRIVATE term_pcl_core)
  add_test(NAME color_map COMMAND test_color_map)

  add_executable(test_cloud_optimizer tests/test_cloud_optimizer.cpp)
  target_link_libraries(test_cloud_optimizer PRIVATE term_pcl_core)
  add_test(NAME cloud_optimizer COMMAND test_cloud_optimizer)

  add_executable(test_lod_cloud_provider tests/test_lod_cloud_provider.cpp)
  target_link_libraries(test_lod_cloud_provider PRIVATE term_pcl_core)
  add_test(NAME lod_cloud_provider COMMAND test_lod_cloud_provider)

  add_executable(test_spatial_lod tests/test_spatial_lod.cpp)
  target_link_libraries(test_spatial_lod PRIVATE term_pcl_core)
  add_test(NAME spatial_lod COMMAND test_spatial_lod)

  add_executable(test_hierarchical_lod tests/test_hierarchical_lod.cpp)
  target_link_libraries(test_hierarchical_lod PRIVATE term_pcl_core)
  add_test(NAME hierarchical_lod COMMAND test_hierarchical_lod)

  add_executable(test_chunk_cache tests/test_chunk_cache.cpp)
  target_link_libraries(test_chunk_cache PRIVATE term_pcl_core)
  add_test(NAME chunk_cache COMMAND test_chunk_cache)

  add_executable(test_chunk_loader tests/test_chunk_loader.cpp)
  target_link_libraries(test_chunk_loader PRIVATE term_pcl_core)
  add_test(NAME chunk_loader COMMAND test_chunk_loader)

  add_executable(test_terminal_ui tests/test_terminal_ui.cpp)
  target_link_libraries(test_terminal_ui PRIVATE term_pcl_core)
  add_test(NAME terminal_ui COMMAND test_terminal_ui)

  add_executable(test_termcloud_index tests/test_termcloud_index.cpp)
  target_link_libraries(test_termcloud_index PRIVATE term_pcl_core)
  add_test(NAME termcloud_index COMMAND test_termcloud_index)
  set_tests_properties(termcloud_index PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  add_executable(test_point_cloud_loader tests/test_point_cloud_loader.cpp)
  target_link_libraries(test_point_cloud_loader PRIVATE term_pcl_core)
  add_test(NAME point_cloud_loader COMMAND test_point_cloud_loader)
  set_tests_properties(point_cloud_loader PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  add_test(NAME cli_help COMMAND term-pcl --help)
  add_test(NAME cli_behavior COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli.sh $<TARGET_FILE:term-pcl>)
  set_tests_properties(cli_behavior PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
