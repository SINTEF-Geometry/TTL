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


# 'install' target for TTL

# Set the installation prefix based on platform
IF(WIN32)
  SET(ttl_INSTALL_PREFIX "$ENV{PROGRAMFILES}/SINTEF/ttl" CACHE PATH "Path to install ttl")
ELSE()
  SET(ttl_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}" CACHE PATH "Path to install ttl")
ENDIF()

# === Install Targets and Export CMake Package ===
include(CMakePackageConfigHelpers)

# Install the ttl target with export
install(TARGETS ttl
    EXPORT ttlTargets
    ARCHIVE DESTINATION ${ttl_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${ttl_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${ttl_INSTALL_PREFIX}/bin
    INCLUDES DESTINATION ${ttl_INSTALL_PREFIX}/include
)

# Install header files
install(DIRECTORY include/
    DESTINATION ${ttl_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

# Export target info to a file
install(EXPORT ttlTargets
    FILE ttlTargets.cmake
    #NAMESPACE TTL::              # Uncomment if using target namespaces
    DESTINATION ${ttl_INSTALL_PREFIX}/lib/cmake/ttl
)

# Generate and install version/config files
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ttlConfigVersion.cmake"
    VERSION 1.2.0
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ttlConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ttlConfig.cmake"
    INSTALL_DESTINATION ${ttl_INSTALL_PREFIX}/lib/cmake/ttl
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ttlConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ttlConfigVersion.cmake"
    DESTINATION ${ttl_INSTALL_PREFIX}/lib/cmake/ttl
)

# CPack setup for packaging if needed
IF(NOT GoTools_ALL_COMPILE)
  SET(CPACK_SOURCE_PACKAGE_FILE_NAME "ttl-1.2.0")
  SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\.svn/;\\.swp$;\\.#;/#;.*~")
  INCLUDE(CPack)
ENDIF()

SET(CPACK_STRIP_FILES ${CPACK_STRIP_FILES} libttl.so)
