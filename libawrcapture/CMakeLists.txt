CMAKE_MINIMUM_REQUIRED ( VERSION 3.0 )
PROJECT ( libawrcapture )

INCLUDE ( CheckCXXSourceCompiles )
INCLUDE ( CheckCXXSourceRuns )

if(NOT TARGET libradar)
  ADD_SUBDIRECTORY ( ../libradar ext )
endif()

#
# Source folders
#

SOURCE_GROUP ( include REGULAR_EXPRESSION include/awrcapture/.* )

SET ( LIBAWRCAPTURE_CXX11_FEATURES
  cxx_auto_type
  cxx_explicit_conversions
  cxx_lambdas
  cxx_nullptr
  cxx_range_for
  cxx_static_assert
)

#
# Dependencies
#

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#
# Main executables
#

FILE ( GLOB LIBAWRCAPTURE_SOURCE
  include/awrcapture/*
  src/*
)

ADD_LIBRARY (
  libawrcapture
  ${LIBAWRCAPTURE_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES ( libawrcapture
  PUBLIC include
  PRIVATE include/libawrcapture
)

SET_TARGET_PROPERTIES ( libawrcapture PROPERTIES OUTPUT_NAME awrcapture )
TARGET_COMPILE_FEATURES ( libawrcapture PRIVATE ${LIBAWRCAPTURE_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( libawrcapture PRIVATE Eigen3::Eigen )

INSTALL ( TARGETS libawrcapture
  DESTINATION lib
)
