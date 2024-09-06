cmake_minimum_required(VERSION 3.15.3)
project(gst_min)

set(CMAKE_PREFIX_PATH "/opt/gstreamer/")
set(ENV{PKG_CONFIG_PATH} "/opt/gstreamer/lib/x86_64-linux-gnu/pkgconfig")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

find_package(PkgConfig REQUIRED)
pkg_search_module(gtk3 REQUIRED IMPORTED_TARGET gtk+-3.0>=1.4)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0>=1.4)
pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0>=1.4)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0>=1.4)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0>=1.4)
pkg_search_module(gstreamer-base REQUIRED IMPORTED_TARGET gstreamer-base-1.0>=0.0)
pkg_search_module(glib REQUIRED IMPORTED_TARGET glib-2.0>=0.0)

find_package(Boost COMPONENTS system REQUIRED)

set(SOURCES main.cpp
            utils.cpp)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME} Boost::system)

target_link_libraries(${CMAKE_PROJECT_NAME}
    PkgConfig::gtk3
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
    PkgConfig::gstreamer-base
    PkgConfig::glib
)