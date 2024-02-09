# edit the following line to add all the source code files of the library
SET(sources ./exceptions/exceptions.cpp ./mutex/mutex.cpp ./mutex/mutexexceptions.cpp ./events/event.cpp ./events/eventserver.cpp ./events/eventexceptions.cpp ./threads/thread.cpp ./threads/threadserver.cpp ./threads/threadexceptions.cpp ./logs/log.cpp ./logs/logexceptions.cpp ./time/ctime.cpp ./time/ctimeexceptions.cpp)
# edit the following line to add all the header files of the library
SET(headers ../include/exceptions/exceptions.h ../include/mutex/mutex.h ../include/mutex/mutexexceptions.h ../include/events/event.h ../include/events/eventserver.h ../include/events/eventexceptions.h ../include/threads/thread.h ../include/threads/threadserver.h ../include/threads/threadexceptions.h ../include/logs/log.h ../include/logs/logexceptions.h ../include/time/ctime.h ../include/time/ctimeexceptions.h)

# edit the following line to add the necessary include directories
INCLUDE_DIRECTORIES(../include/exceptions ../include/mutex ../include/events ../include/threads ../include/logs ../include/time)

# edit the following line to find the necessary packages

ADD_LIBRARY(iriutils SHARED ${sources})

#edit the following line to add the necessary system libraries (if any)
IF(APPLE)
  MESSAGE("APPLE SYSTEM FOUND")
  TARGET_LINK_LIBRARIES(iriutils pthread)
ELSE(APPLE)
  TARGET_LINK_LIBRARIES(iriutils pthread rt)
ENDIF(APPLE)

INSTALL(TARGETS iriutils
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/iri/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/iri/${PROJECT_NAME}
)

INSTALL(FILES ${headers} DESTINATION include/iri/${PROJECT_NAME})

INSTALL(FILES ../Findiriutils.cmake DESTINATION ${CMAKE_ROOT}/Modules/)

INSTALL(FILES  ../scripts/add_lib_to_ld_config.sh 
               ../scripts/remove_lib_from_ld_config.sh 
               DESTINATION share/iri
               PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_READ WORLD_EXECUTE)


ADD_SUBDIRECTORY(examples)
