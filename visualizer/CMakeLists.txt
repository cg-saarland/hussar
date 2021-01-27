CMAKE_MINIMUM_REQUIRED ( VERSION 3.0 )
PROJECT ( visualizer )

INCLUDE ( CheckCXXSourceCompiles )
INCLUDE ( CheckCXXSourceRuns )

ADD_SUBDIRECTORY ( ../libradar ext/radar )
ADD_SUBDIRECTORY ( ../libhussar ext/hussar )
ADD_SUBDIRECTORY ( ../libawrcapture ext/awrcapture )

INCLUDE ( ../3rdparty/imgui/CMakeLists.txt )

IF (WIN32)
	INCLUDE ( ../3rdparty/termiWin/CMakeLists.txt )
ENDIF()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#
# Source folders
#

SOURCE_GROUP ( src REGULAR_EXPRESSION src/.* )

INCLUDE_DIRECTORIES ( src )

SET ( VISUALIZER_CXX11_FEATURES
  cxx_auto_type
  cxx_explicit_conversions
  cxx_lambdas
  cxx_nullptr
  cxx_range_for
  cxx_static_assert
)

#
# Main executables
#

find_package(OpenGL REQUIRED)

FILE ( GLOB VISUALIZER_SOURCE
  src/*
)

ADD_EXECUTABLE (
  visualizer
  ${VISUALIZER_SOURCE}
)

TARGET_INCLUDE_DIRECTORIES ( visualizer
  PRIVATE ${OPENGL_INCLUDE_DIR}
  PRIVATE ../3rdparty/tinyexr
)
SET_TARGET_PROPERTIES ( visualizer PROPERTIES CXX_STANDARD 20 OUTPUT_NAME visualizer )
TARGET_COMPILE_FEATURES ( visualizer PRIVATE ${VISUALIZER_CXX11_FEATURES} )
TARGET_LINK_LIBRARIES ( visualizer libawrcapture libhussar libradar ${IMGUI_LIBRARIES} )

IF (WIN32)
	TARGET_LINK_LIBRARIES ( visualizer ws2_32 opengl32 termiWin )
ENDIF()

INSTALL ( TARGETS
  visualizer
  DESTINATION
  bin
)
