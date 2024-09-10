cmake_minimum_required(VERSION 3.1)

project(mc_fetch)

set(CMAKE_CXX_STANDARD 11)

find_package(mc_rtc REQUIRED)
find_description_package(fetch_description)
find_package(fetch_description REQUIRED)
if(${fetch_description_FOUND})
  # FIXME: overwrite FETCH_DESCRIPTION_PATH
  if(EXISTS ${fetch_description_INSTALL_PREFIX}/share/fetch_description/robots/fetch.urdf)
    set(FETCH_DESCRIPTION_PATH ${fetch_description_INSTALL_PREFIX}/share/fetch_description)
  elseif(EXISTS ${fetch_description_SOURCE_PREFIX}/robots/fetch.urdf)
    set(FETCH_DESCRIPTION_PATH ${fetch_description_SOURCE_PREFIX})
  else()
    message(WARNING "Could not found fetch.urdf in ${fetch_description_INSTALL_PREFIX}/share/fetch_description/robots/fetch.urdf and ${fetch_description_SOURCE_PREFIX}/robots/fetch.urdf")
  endif()
  set(FETCH_NAME "fetch")
endif()
message("-- FETCH_DESCRIPTION_PATH: ${FETCH_DESCRIPTION_PATH}")

find_description_package(mc_fetch_description)
if(${mc_fetch_description_FOUND})
  set(MC_FETCH_DESCRIPTION_PATH ${mc_fetch_description_SOURCE_PREFIX})
endif()
message("-- MC_FETCH_DESCRIPTION_PATH: ${MC_FETCH_DESCRIPTION_PATH}")

configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(fetch src/fetch.cpp src/fetch.h)
target_include_directories(fetch PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
