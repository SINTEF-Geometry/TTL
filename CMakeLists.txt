PROJECT(ttl)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)


# Find modules

# First extend CMake module path
OPTION(EXTEND_CMAKE_MODULE_PATH 
  "Extend the CMAKE_MODULE_PATH variable with user directories?" ON)
IF(EXTEND_CMAKE_MODULE_PATH)
  SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
    "./cmake/Modules"
    "~/cmake/Modules")
ENDIF(EXTEND_CMAKE_MODULE_PATH)

FIND_PACKAGE(GoTools)


# Include directories

INCLUDE_DIRECTORIES(
  ${ttl_SOURCE_DIR}/include
  ${GoTools_INCLUDE_DIRS}
  )

IF(WIN32)
  FIND_PATH(
    BOOST_DIR boost/shared_ptr.hpp
    PATHS "C:/Program Files/boost/boost_1_34_1"
    DOC "Boost must be installed on the system"
    )
  INCLUDE_DIRECTORIES(
    ${BOOST_DIR}
    )
ENDIF(WIN32)


# Linked in libraries

SET(DEPLIBS
  ${GoTools_LIBRARIES}
  )


# Make the ttl library

FILE(GLOB ttl_SRCS src/*.C src/*.cpp)
ADD_LIBRARY(ttl ${ttl_SRCS})
TARGET_LINK_LIBRARIES(ttl ${DEPLIBS})


# Apps, examples, tests, ...?

FILE(GLOB ttl_APPS app/*.C app/*.cpp)
FOREACH(app ${ttl_APPS})
  GET_FILENAME_COMPONENT(appname ${app} NAME_WE)
  ADD_EXECUTABLE(${appname} ${app})
  TARGET_LINK_LIBRARIES(${appname} ttl ${DEPLIBS})
ENDFOREACH(app)

# Copy data
ADD_CUSTOM_COMMAND(
  TARGET ttl
  POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy_directory ${ttl_SOURCE_DIR}/data 
  ${ttl_BINARY_DIR}/data
  )


# 'install' target

INSTALL(TARGETS ttl DESTINATION lib)
INSTALL(DIRECTORY include/GoTools/ttl
  DESTINATION include/GoTools
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".hpp"
  PATTERN ".svn" EXCLUDE
)
