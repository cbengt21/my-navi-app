# Minimum CMake version required
cmake_minimum_required(VERSION 3.5.1)

# Project name
project(MyNaviApp LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(/usr/local/include /usr/local/include/osmium /usr/local/include/protozero /usr/include/boost /usr/include/asio /usr/local/include/asio)
#include_directories(/usr/local/include /usr/local/include/osmium /usr/local/include/protozero /usr/include/asio /usr/local/include/asio)


# Set the path to the Osmium package configuration file
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/osmium")

#find_package(Osmium REQUIRED)
#find_package(Protozero REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
# Find and link zlib, needed for reading and writing OSM PBF files
find_package(ZLIB REQUIRED)
find_package(nlohmann_json REQUIRED)
include_directories(${ZLIB_INCLUDE_DIRS})
include_directories(${nlohmann_json_INCLUDE_DIRS})

# Find and include Asio
#find_package(Asio REQUIRED)
#include_directories(${ASIO_INCLUDE_DIR})

# Add source files
set(SOURCE_FILES backend.cpp)

# Link libraries
link_directories(/usr/local/lib /usr/lib/x86_64-linux-gnu)

# Link libraries (Crow, Osmium, and other dependencies)
#find_library(CROW_LIB NAMES crow)
#find_library(OSMIUM_LIB NAMES osmium)

#add_definitions(-DCROW_USE_BOOST=0)

# Create the executable
add_executable(backend ${SOURCE_FILES})

# Define ASIO_STANDALONE
#target_compile_definitions(backend PRIVATE CROW_USE_BOOST=0 ASIO_STANDALONE)
#target_compile_definitions(backend PRIVATE ASIO_STANDALONE)
#target_compile_definitions(backend PRIVATE CROW_USE_BOOST=0)
target_compile_definitions(backend PRIVATE CROW_USE_BOOST)

# Link Crow, Osmium, and other required libraries
#target_link_libraries(backend pthread)
#find_package(Boost REQUIRED COMPONENTS system)
#target_link_libraries(backend Boost::system)
#target_link_libraries(backend osmpbf protozero pthread)
#target_link_libraries(backend osmpbf protozero boost_system boost_filesystem pthread)
#target_link_libraries(backend osmpbf protozero Boost::system boost_system boost_filesystem pthread)
target_link_libraries(backend osmpbf Boost::system boost_system boost_filesystem pthread ${ZLIB_LIBRARIES} nlohmann_json::nlohmann_json)