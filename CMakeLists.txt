PROJECT(ttl)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


# Find modules

# First extend CMake module path
OPTION(EXTEND_CMAKE_MODULE_PATH 
  "Extend the CMAKE_MODULE_PATH variable with user directories?" ON)
IF(EXTEND_CMAKE_MODULE_PATH)
  SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
    ${CMAKE_HOME_DIRECTORY}/cmake/Modules
    ${CMAKE_INSTALL_PREFIX}/share/cmake/Modules
    "./cmake/Modules"
    "~/cmake/Modules"
    "C:/cmake/Modules")
ENDIF(EXTEND_CMAKE_MODULE_PATH)

FIND_PACKAGE(Boost REQUIRED)

# Include directories

INCLUDE_DIRECTORIES(
  ${ttl_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
  )


# Linked in libraries

SET(DEPLIBS
  )


# Make the ttl library

FILE(GLOB_RECURSE ttl_SRCS src/*.C src/*.cpp include/*.h)
ADD_LIBRARY(ttl ${ttl_SRCS})
TARGET_LINK_LIBRARIES(ttl ${DEPLIBS})


# Apps, examples, tests, ...?

FILE(GLOB_RECURSE ttl_EXAMPLES examples/*.C examples/*.cpp)
FOREACH(example ${ttl_EXAMPLES})
  GET_FILENAME_COMPONENT(examplename ${example} NAME_WE)
  ADD_EXECUTABLE(${examplename} ${example})
  TARGET_LINK_LIBRARIES(${examplename} ttl ${DEPLIBS})
ENDFOREACH(example)

# Copy data
ADD_CUSTOM_COMMAND(
  TARGET ttl
  POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy_directory ${ttl_SOURCE_DIR}/data 
  ${ttl_BINARY_DIR}/data
  )


# 'install' target

IF(WIN32)
  # Windows
  SET(CMAKE_INSTALL_PREFIX CACHE INTERNAL "")
  SET(ttl_INSTALL_PREFIX "C:/Program Files (x86)/TTL"
    CACHE PATH "Path to install TTL")
  # lib
  INSTALL(TARGETS ttl DESTINATION ${ttl_INSTALL_PREFIX}/lib)
  # include
  INSTALL(DIRECTORY include/ttl
    DESTINATION ${ttl_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )
ELSE(WIN32)
  # Linux
  # lib
  INSTALL(TARGETS ttl DESTINATION lib)
  # include
  INSTALL(DIRECTORY include/ttl
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )
ENDIF(WIN32)
