add_executable(FastSLAM test.cpp)

target_include_directories(FastSLAM PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/include"
                         )
target_link_libraries(FastSLAM Eigen3::Eigen)

if(BUILD_TESTS)
  add_executable(test_TestClass TestClass_test.cpp)
  target_link_libraries(test_TestClass
                        PRIVATE Catch2::Catch2WithMain
                        Eigen3::Eigen)
  catch_discover_tests(test_TestClass)
  target_include_directories(test_TestClass PUBLIC
    "${PROJECT_BINARY_DIR}"
    "${PROJECT_SOURCE_DIR}/include"
  )
endif()
