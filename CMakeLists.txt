# Minimum CMake version required
cmake_minimum_required(VERSION 3.5.1)

# Project name
project(MyNaviApp LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable -O3 optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Include directories
include_directories(
    /usr/local/include
    /usr/local/include/osmium
    /usr/local/include/protozero
    /usr/include/boost
    /usr/include/asio
    /usr/local/include/asio
)

# Set the path to the Osmium package configuration file
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/osmium")

# Find required packages
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(ZLIB REQUIRED)
find_package(nlohmann_json REQUIRED)

# Add source files
set(SOURCE_FILES backend.cpp)

# Link directories
link_directories(/usr/local/lib /usr/lib/x86_64-linux-gnu)

# Create the executable
add_executable(backend ${SOURCE_FILES})

# Define macros
target_compile_definitions(backend PRIVATE CROW_USE_BOOST)

# Include directories from found packages
include_directories(${ZLIB_INCLUDE_DIRS})
include_directories(${nlohmann_json_INCLUDE_DIRS})

# Link necessary libraries
target_link_libraries(backend
    osmpbf
    Boost::system
    boost_system
    boost_filesystem
    pthread
    ${ZLIB_LIBRARIES}
    nlohmann_json::nlohmann_json
)
