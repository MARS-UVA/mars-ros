include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${GMOCK_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/mutex)

set(CTEST_CUSTOM_COVERAGE_EXCLUDE *)

add_executable(mutex_tests mutex/mutex_test.cpp)
add_dependencies(mutex_tests googletest)

target_link_libraries(mutex_tests ${PROJECT_SOURCE_DIR}/lib/libiriutils.so)
target_link_libraries(mutex_tests ${GTEST_LIBS_DIR}/libgtest.a ${GTEST_LIBS_DIR}/libgtest_main.a)
target_link_libraries(mutex_tests ${GMOCK_LIBS_DIR}/libgmock.a ${GMOCK_LIBS_DIR}/libgmock_main.a)
target_link_libraries(mutex_tests pthread)

add_test(NAME Mutex_Test COMMAND mutex_tests)
