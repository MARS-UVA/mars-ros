# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW) 
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(iriutils)
SET(PACKAGE_NAME iriutils)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

IF ("${INSTALL_DIR}" STREQUAL "")
  MESSAGE ("[INSTALL] set install_dir to /usr/local")
  SET(CMAKE_INSTALL_PREFIX /usr/local)
ELSE ()
  MESSAGE ("[INSTALL] set install_dir to input ${INSTALL_DIR}")
  SET(CMAKE_INSTALL_PREFIX ${INSTALL_DIR})
ENDIF ()

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "DEBUG") 
ENDIF (NOT CMAKE_BUILD_TYPE)

SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT -O0 --coverage -fprofile-arcs")
SET(CMAKE_LINK_FLAGS_DEBUG "--coverage -fprofile-arcs")


#
# Add ctest
#
# INCLUDE(CTest)

ADD_SUBDIRECTORY(src)

# ADD_SUBDIRECTORY(test/gtest)

# ADD_SUBDIRECTORY(test)

FIND_PACKAGE(Doxygen)

FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
IF (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
ELSE (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt
    COMMAND rm -rd ${CMAKE_INSTALL_PREFIX}/lib/iri/${PROJECT_NAME}
    COMMAND rm -rd ${CMAKE_INSTALL_PREFIX}/include/iri/${PROJECT_NAME}
    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)


IF (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  SET (X86 TRUE)
  SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
ELSE ()
  SET (X86 FALSE)
  SET(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
ENDIF()

IF (UNIX)
  SET(CPACK_PACKAGE_FILE_NAME "iri-${PACKAGE_NAME}-dev-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  SET(CPACK_PACKAGE_NAME "iri-${PACKAGE_NAME}-dev")
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Part of IRI-laboratory libraries. More information at https://gitlab.iri.upc.edu/labrobotica/labrobotica_how_to")
  SET(CPACK_PACKAGING_INSTALL_PREFIX /usr)
  SET(CPACK_GENERATOR "DEB")
  SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "labrobotica - labrobotica@iri.upc.edu")
  SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://gitlab.iri.upc.edu/labrobotica/algorithms/iriutils")

# Uncomment to add the necessary mantainer scripts
  SET(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/scripts/debian/postinst;${CMAKE_SOURCE_DIR}/scripts/debian/prerm")

# Uncomment to add dependencies comma separated
# SET(CPACK_DEBIAN_PACKAGE_DEPENDS "iri-<package_name>-dev (>= 1.0~${DISTRIB})")

  INCLUDE(CPack)
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "packaging only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)

