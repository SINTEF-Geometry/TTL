PROJECT(ttl)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


# Include directories

INCLUDE_DIRECTORIES(
  ${ttl_SOURCE_DIR}/include
  )


# Linked in libraries

#SET(DEPLIBS
#  )


# Make the ttl library

FILE(GLOB_RECURSE ttl_SRCS src/*.C src/*.cpp include/*.h)
ADD_LIBRARY(ttl ${ttl_SRCS})
SET_PROPERTY(TARGET ttl
  PROPERTY FOLDER "ttl/Libs")


# Set various compiler flags
IF(CMAKE_COMPILER_IS_GNUXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUXX)
IF(WIN32)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP8 -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS")
ENDIF(WIN32)


# Apps, examples, tests, ...?

FILE(GLOB_RECURSE ttl_EXAMPLES examples/*.C examples/*.cpp)
FOREACH(example ${ttl_EXAMPLES})
  GET_FILENAME_COMPONENT(examplename ${example} NAME_WE)
  ADD_EXECUTABLE(${examplename} ${example})
  TARGET_LINK_LIBRARIES(${examplename} ttl ${DEPLIBS})
    SET_PROPERTY(TARGET ${examplename}
      PROPERTY FOLDER "ttl/Examples")
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
  SET(ttl_INSTALL_PREFIX "$ENV{PROGRAMFILES}/SINTEF/TTL"
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
  INSTALL(TARGETS ttl DESTINATION lib COMPONENT ttl)
  # include
  INSTALL(DIRECTORY include/ttl
    COMPONENT ttl-dev
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )
ENDIF(WIN32)


# CPack stuff
IF(NOT GoTools_ALL_COMPILE)
  SET(CPACK_SOURCE_PACKAGE_FILE_NAME "TTL-1.1.0")
  SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\.svn/;\\.swp$;\\.#;/#;.*~")
  INCLUDE(CPack)
ENDIF(NOT GoTools_ALL_COMPILE)

SET(CPACK_STRIP_FILES ${CPACK_STRIP_FILES} libttl.so)
