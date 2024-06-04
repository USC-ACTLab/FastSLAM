add_library(FastSLAMLib
   math-util.cpp
   EKF.cpp
   robot-manager.cpp
)

target_include_directories(FastSLAMLib PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(FastSLAMLib Eigen3::Eigen)

if(BUILD_TESTS)
  add_executable(test_MathUtil math-util_test.cpp)
  target_link_libraries(test_MathUtil
                        PRIVATE Catch2::Catch2WithMain
                        FastSLAMLib)
  catch_discover_tests(test_MathUtil)
  target_include_directories(test_MathUtil PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
  )

  add_executable(test_EKF EKF_test.cpp)
  target_link_libraries(test_EKF
                        PRIVATE Catch2::Catch2WithMain
                        FastSLAMLib)
  catch_discover_tests(test_EKF)
  target_include_directories(test_EKF PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
  )
endif()
