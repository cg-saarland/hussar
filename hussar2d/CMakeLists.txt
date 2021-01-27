CMAKE_MINIMUM_REQUIRED ( VERSION 3.0 )
PROJECT ( hussar2d )

INCLUDE ( CheckCXXSourceCompiles )
INCLUDE ( CheckCXXSourceRuns )

if(NOT TARGET libradar)
  ADD_SUBDIRECTORY ( ../libradar ext/libradar )
endif()

if(NOT TARGET libguiding)
  ADD_SUBDIRECTORY ( ../3rdparty/libguiding ext/libguiding )
endif()

#
# Source folders
#

SOURCE_GROUP ( include REGULAR_EXPRESSION include/hussar2d/.* )

SOURCE_GROUP ( core REGULAR_EXPRESSION src/core/.* )

SET ( HUSSAR2D_CXX11_FEATURES
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

FIND_PACKAGE ( Eigen3 3.3 REQUIRED NO_MODULE )
FIND_PACKAGE ( SDL2 REQUIRED )

#
# Main executables
#

IF (NOT CMAKE_BUILD_TYPE)
  SET ( CMAKE_BUILD_TYPE Release )
ENDIF()

SET ( CMAKE_CXX_FLAGS_RELEASE "-O3" )

FILE ( GLOB HUSSAR2D_SOURCE
  include/hussar2d/*

  src/*
  src/core/*
)

ADD_EXECUTABLE (
  hussar2d
  ${HUSSAR2D_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES ( hussar2d
  PUBLIC include
  PRIVATE include/hussar2d
  PRIVATE include/hussar2d/core
  PRIVATE ${SDL2_INCLUDE_DIRS}
)

SET_TARGET_PROPERTIES ( hussar2d PROPERTIES CXX_STANDARD 20 OUTPUT_NAME hussar2d )
TARGET_COMPILE_FEATURES ( hussar2d PRIVATE ${HUSSAR2D_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( hussar2d PRIVATE libradar libguiding )
TARGET_LINK_LIBRARIES ( hussar2d PRIVATE ${SDL2_LIBRARIES} )
TARGET_LINK_LIBRARIES ( hussar2d PUBLIC Eigen3::Eigen -lpthread )

INSTALL ( TARGETS hussar2d
  DESTINATION lib
)
