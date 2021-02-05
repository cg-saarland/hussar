cmake_minimum_required (VERSION 3.0)
project (examples)

include (CheckCXXSourceCompiles)
include (CheckCXXSourceRuns)

add_subdirectory (../libradar ext/radar)
add_subdirectory (../libhussar ext/hussar)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

set (CMAKE_CXX_FLAGS "-Wall -Wextra")
set (CMAKE_CXX_FLAGS_DEBUG "-g")
set (CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Source folders
source_group (src REGULAR_EXPRESSION src/.*)
include_directories (src)

set (EXAMPLES_CXX11_FEATURES
  cxx_auto_type
  cxx_explicit_conversions
  cxx_lambdas
  cxx_nullptr
  cxx_range_for
  cxx_static_assert
)

# Executables
set (EXAMPLES
    0_simplebox
    1_dihedral
    2_large_scene
)

foreach (example ${EXAMPLES})
    add_executable (${example} "src/${example}.cpp")
    set_target_properties (${example} PROPERTIES CXX_STANDARD 20 OUTPUT_NAME ${example})
    target_compile_features (${example} PRIVATE ${EXAMPLES_CXX11_FEATURES})
    target_link_libraries (${example} libhussar libradar ${IMGUI_LIBRARIES})

    install (TARGETS ${example} DESTINATION bin)
endforeach ()
