add_library(FastSLAM
   probability.cpp
)

target_include_directories(FastSLAM PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(FastSLAM Eigen3::Eigen)