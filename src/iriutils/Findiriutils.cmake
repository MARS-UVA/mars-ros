FIND_PATH(iriutils_INCLUDE_DIRS exceptions.h mutex.h mutexexceptions.h event.h eventserver.h eventexceptions.h thread.h threadserver.h threadexceptions.h ctime.h ctimexceptions.h log.h logexceptions.h /usr/local/include/iri/iriutils /usr/include/iri/iriutils)


FIND_LIBRARY(iriutils_LIBRARIES
    NAMES iriutils
    PATHS /usr/local/lib/iri/iriutils /usr/lib/iri/iriutils) 
    
SET(iriutils_INCLUDE_DIR ${iriutils_INCLUDE_DIRS})
SET(iriutils_LIBRARY    ${iriutils_LIBRARIES})

IF (iriutils_INCLUDE_DIRS AND iriutils_LIBRARIES)
   SET(iriutils_FOUND TRUE)
ENDIF (iriutils_INCLUDE_DIRS AND iriutils_LIBRARIES)

IF (iriutils_FOUND)
   IF (NOT iriutils_FIND_QUIETLY)
      MESSAGE(STATUS "Found iriutils: ${iriutils_LIBRARIES}")
   ENDIF (NOT iriutils_FIND_QUIETLY)
ELSE (iriutils_FOUND)
   IF (iriutils_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find iriutils")
   ENDIF (iriutils_FIND_REQUIRED)
ENDIF (iriutils_FOUND)
