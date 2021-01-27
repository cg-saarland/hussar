file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES true *.h *.cpp)
file(GLOB_RECURSE TEST_DATA LIST_DIRECTORIES true *.exr)

set(TEST_BINARY libhussar_test)

file(COPY ${TEST_DATA} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(${TEST_BINARY} ${TEST_SOURCES})

add_subdirectory(../../3rdparty/googletest/ gtest)

target_include_directories(${TEST_BINARY}
  PUBLIC include
  PRIVATE include/hussar
  PRIVATE include/hussar/core
  PUBLIC ../../3rdparty/tinyexr
)

target_link_libraries(${TEST_BINARY}
  PUBLIC libhussar
  PRIVATE libradar
  PRIVATE gtest_main
)
