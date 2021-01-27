cmake_minimum_required (VERSION 3.12)
project (libhussar LANGUAGES CXX C)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

include (CheckCXXSourceCompiles)
include (CheckCXXSourceRuns)

#
# Configuration options
#

#option (HUSSAR_NVTX "Insert NVTX annotations for NVIDIA Profiling and Debugging Tools" OFF)
set (HUSSAR_OPTIX7_PATH "" CACHE STRING "Path to OptiX 7 SDK")

option (HUSSAR_BUILD_NATIVE_EXECUTABLE "Build executable optimized for CPU architecture of system libhussar was built on" ON)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message (STATUS "Setting build type to 'Release' as none was specified.")
  set (CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property (CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif ()

if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
  # Build tests by default when building only libhussar
  option (HUSSAR_BUILD_TESTS "Build libhussar tests" ON)
else ()
  option (HUSSAR_BUILD_TESTS "Build libhussar tests" OFF)
endif ()

#
# Dependencies
#

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (embree 3.0)

if (embree_FOUND)
  set (HUSSAR_CPU_SUPPORT ON)
  set (HUSSAR_DEFINITIONS ${HUSSAR_DEFINITIONS} HUSSAR_BUILD_CPU_RENDERER)

  message (STATUS "Found embree: ${embree_VERSION}")
else ()
  message (WARNING "embree not found. Disabling CPU compilation.")
endif ()

set (RADAR_UNIFIED_MEMORY ON)

if (NOT TARGET libradar)
  add_subdirectory (../libradar ext/radar)
endif ()

if (NOT TARGET libguiding)
  add_subdirectory (../3rdparty/libguiding ext/guiding)
endif ()

if (HUSSAR_BUILD_TESTS)
  add_subdirectory (tst)
endif ()

#
# CUDA / OptiX
# Taken from pbrt-v4
#

include (CheckLanguage)
check_language (CUDA)

if (CMAKE_CUDA_COMPILER)
  find_package (CUDA REQUIRED)

  # This seems to be necessary starting with 3.17.1, but gives an error
  # about 17 being an unsupported version earlier...
  if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.17.0")
    set (CMAKE_CUDA_STANDARD 17)
  endif ()

  if (NOT CMAKE_CUDA_COMPILER_VERSION)
    set(CMAKE_CUDA_COMPILER_VERSION ${CUDA_VERSION})
  endif ()

  message (STATUS "Found CUDA: ${CMAKE_CUDA_COMPILER_VERSION}")
  if ("${HUSSAR_OPTIX7_PATH}" STREQUAL "")
    message (WARNING "Found CUDA but HUSSAR_OPTIX7_PATH is not set. Disabling GPU compilation.")
  else ()
    enable_language (CUDA)
    set (HUSSAR_DEFINITIONS ${HUSSAR_DEFINITIONS} HUSSAR_BUILD_GPU_RENDERER)
    set (HUSSAR_DEFINITIONS ${HUSSAR_DEFINITIONS} NVTX)
    set (HUSSAR_GPU_SUPPORT ON)

    # FIXME
    include_directories (${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})  # for regular c++ compiles

    # http://www.ssl.berkeley.edu/~jimm/grizzly_docs/SSL/opt/intel/cc/9.0/lib/locale/en_US/mcpcom.msg
    set (HUSSAR_CUDA_DIAG_FLAGS "")
    #set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} -Xptxas --warn-on-double-precision-use")
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} -Xcudafe --diag_suppress=partial_override")
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} -Xcudafe --diag_suppress=virtual_function_decl_hidden")
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} -Xcudafe --diag_suppress=integer_sign_change")
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} -Xcudafe --diag_suppress=declared_but_not_referenced")
    # WAR invalid warnings about this with "if constexpr"
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} -Xcudafe --diag_suppress=implicit_return_from_non_void_function")
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} --expt-relaxed-constexpr")
    set (HUSSAR_CUDA_DIAG_FLAGS "${HUSSAR_CUDA_DIAG_FLAGS} --extended-lambda")
    set (CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${HUSSAR_CUDA_DIAG_FLAGS}")

    # Willie hears yeh..
    set (CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xnvlink -suppress-stack-size-warning")

    # https://wagonhelm.github.io/articles/2018-03/detecting-cuda-capability-with-cmake
    # Get CUDA compute capability
    set (OUTPUTFILE ${CMAKE_CURRENT_BINARY_DIR}/checkcuda)
    execute_process (COMMAND nvcc -lcuda ${CMAKE_CURRENT_SOURCE_DIR}/cmake/checkcuda.cu -o ${OUTPUTFILE})
    execute_process (COMMAND ${OUTPUTFILE}
                      RESULT_VARIABLE CUDA_RETURN_CODE
                      OUTPUT_VARIABLE ARCH)

    set (CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --std=c++17")
    if (CMAKE_BUILD_TYPE MATCHES Release)
      set (CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --use_fast_math -lineinfo --maxrregcount 128")
    else()
      set (CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --use_fast_math -G -g")
    endif ()

    if (NOT ${CUDA_RETURN_CODE} EQUAL 0)
      message (SEND_ERROR "Unable to determine GPU's compute capability")
    else ()
      message (STATUS "CUDA Architecture: ${ARCH}")
      set (CMAKE_CUDA_ARCHITECTURES ${ARCH})
      set (CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --gpu-architecture=${ARCH}")
    endif ()

    # optix
    # FIXME
    include_directories (${HUSSAR_OPTIX7_PATH}/include)
    # FIXME. Sigh. I'm not sure how else to pass this through to cuda_compile_ptx...
    include_directories (${CUDA_INCLUDE_DIRS})
    include_directories (include)
    include_directories (../libradar/include)
    include_directories (../3rdparty/libguiding/include)
    include_directories (${EIGEN3_INCLUDE_DIRS})
    #include_directories (${CMAKE_BINARY_DIR})

    # from Ingo's configure_optix.cmake (Apache licensed)
    find_program (BIN2C bin2c DOC "Path to the CUDA SDK bin2c executable.")

    # this macro defines cmake rules that execute the following four steps:
    # 1) compile the given cuda file ${cuda_file} to an intermediary PTX file
    # 2) use the 'bin2c' tool (that comes with CUDA) to
    #    create a second intermediary (.c-)file which defines a const string variable
    #    (named '${c_var_name}') whose (constant) value is the PTX output
    #    from the previous step.
    # 3) compile the given .c file to an intermediary object file (why thus has
    #    that PTX string 'embedded' as a global constant.
    # 4) assign the name of the intermediary .o file to the cmake variable
    #    'output_var', which can then be added to cmake targets.
    macro (cuda_compile_and_embed output_var cuda_file)
      set (c_var_name ${output_var})
      cuda_compile_ptx (ptx_files ${cuda_file}
                        OPTIONS --std=c++17 -O3 ${HUSSAR_CUDA_DIAG_FLAGS} -DNDEBUG --use_fast_math
                                  # disable "extern declaration... is treated as a static definition" warning
                                -Xcudafe=--display_error_number -Xcudafe=--diag_suppress=3089
                                --gpu-architecture=${ARCH} -D HUSSAR_BUILD_GPU_RENDERER)
      list (GET ptx_files 0 ptx_file)
      set (embedded_file ${ptx_file}_embedded.c)
      add_custom_command (
        OUTPUT ${embedded_file}
        COMMAND ${BIN2C} -c --padd 0 --type char --name ${c_var_name} ${ptx_file} > ${embedded_file}
        DEPENDS ${ptx_file}
        COMMENT "compiling (and embedding ptx from) ${cuda_file}"
        )
      set (${output_var} ${embedded_file})
    endmacro ()
  endif ()
else ()
  message (STATUS "CUDA not found")
endif ()

#
# Annoying compiler-specific details
# Taken from pbrt-v4
#

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag ("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if (COMPILER_SUPPORTS_MARCH_NATIVE AND HUSSAR_BUILD_NATIVE_EXECUTABLE AND NOT HUSSAR_GPU_SUPPORT)
  list (APPEND HUSSAR_CXX_FLAGS -march=native)
endif ()

#
# Main executables
#

file (GLOB LIBHUSSAR_SOURCE
  include/hussar/*

  src/core/*
  src/io/*
  src/antennas/*
  src/integrators/*
  src/guiding/*
  src/samplers/*
  src/shapes/*
)

if (HUSSAR_CPU_SUPPORT)
  file (GLOB LIBHUSSAR_CPU_SOURCE
    src/arch/cpu.cpp
  )
endif ()

if (HUSSAR_GPU_SUPPORT)
  file (GLOB LIBHUSSAR_GPU_SOURCE
    src/arch/gpu.cpp
  )
endif ()

# source groups

source_group (include REGULAR_EXPRESSION include/hussar/.*)

source_group (arch REGULAR_EXPRESSION src/arch/.*)
source_group (core REGULAR_EXPRESSION src/core/.*)
source_group (antennas REGULAR_EXPRESSION src/antennas/.*)
source_group (integrators REGULAR_EXPRESSION src/integrators/.*)
source_group (samplers REGULAR_EXPRESSION src/samplers/.*)
source_group (shapes REGULAR_EXPRESSION src/shapes/.*)
source_group (guiding REGULAR_EXPRESSION src/guiding/.*)
source_group (tests REGULAR_EXPRESSION src/tests/.*)

# some compilation flags

if (HUSSAR_NVTX)
  list(APPEND HUSSAR_DEFINITIONS NVTX)
endif ()

# actual library

add_library (libhussar
  ${LIBHUSSAR_SOURCE}
  ${LIBHUSSAR_CPU_SOURCE}
  ${LIBHUSSAR_GPU_SOURCE}
)

target_include_directories (libhussar
  PUBLIC include
  PRIVATE include/hussar
  PRIVATE include/hussar/core
  PUBLIC ../3rdparty/tinyexr
)

if (HUSSAR_GPU_SUPPORT AND HUSSAR_OPTIX7_PATH)
  target_include_directories (libhussar SYSTEM PUBLIC ${CUDA_INCLUDE_DIRS})
  target_include_directories (libhussar SYSTEM PUBLIC ${HUSSAR_OPTIX7_PATH}/include)
endif ()

set_target_properties (libhussar PROPERTIES CXX_STANDARD 17 OUTPUT_NAME hussar)
target_compile_definitions (libhussar PUBLIC ${HUSSAR_DEFINITIONS})
target_compile_options (libhussar PUBLIC ${HUSSAR_CXX_FLAGS})
target_link_libraries (libhussar PUBLIC libradar libguiding Eigen3::Eigen -lpthread)

if (HUSSAR_CPU_SUPPORT)
  target_link_libraries (libhussar PRIVATE embree)
endif ()

if (HUSSAR_GPU_SUPPORT)
  # inspired by PBRT v4
  cuda_compile_and_embed (HUSSAR_EMBEDDED_PTX src/arch/gpu/device/kernel.cu)
  add_library (hussar_embedded_ptx_lib STATIC ${HUSSAR_EMBEDDED_PTX})
  target_link_libraries (libhussar PRIVATE hussar_embedded_ptx_lib ${CUDA_LIBRARIES})
endif ()

install (TARGETS libhussar DESTINATION lib)
