# project language setup
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # generate compile_commands.json

# ---- Project ----
project(
  FastSLAM
  LANGUAGES CXX
)

# configure output files in build dir
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# library configs
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
# source configuration
add_subdirectory(src)
target_include_directories(FastSLAM PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/include"
                         )
target_link_libraries(FastSLAM Eigen3::Eigen)
