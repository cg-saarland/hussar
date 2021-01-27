cmake_minimum_required (VERSION 3.0)
project (libradar)

include (CheckCXXSourceCompiles)
include (CheckCXXSourceRuns)

#
# external dependencies
#

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_library (FFTW_LIBRARY NAMES fftw3f)
find_path (FFTW_INCLUDES NAMES "fftw3.h")

if (FFTW_INCLUDES AND FFTW_LIBRARY)
    list (APPEND RADAR_DEFINITIONS RADAR_HAS_FFTW3)
    message (STATUS "Found fftw3.h")
else ()
    message (STATUS "fftw3.h not found. Disabling FFT support.")
endif ()

#
# Main executables
#

file (GLOB LIBRADAR_SOURCE
  include/radar/*
  src/*
)

add_library (
  libradar
  ${LIBRADAR_SOURCE}
)

target_include_directories (libradar
  PUBLIC include
  PRIVATE include/radar
  PRIVATE ${FFTW_INCLUDES}
)

set_target_properties (libradar PROPERTIES CXX_STANDARD 17 OUTPUT_NAME radar)
target_compile_definitions (libradar PRIVATE ${RADAR_DEFINITIONS})
target_link_libraries (libradar PRIVATE ${FFTW_LIBRARY} Eigen3::Eigen)

install (TARGETS libradar
  DESTINATION lib
)
