# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_events test_events.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_events iriutils pthread)

# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_eventserver test_eventserver.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_eventserver iriutils pthread)

# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_threads test_threads.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_threads iriutils pthread)

# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_threadserver test_threadserver.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_threadserver iriutils pthread)

# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_both test_bothservers.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_both iriutils pthread)

# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_time test_time.cpp)

# edit the following line to add the necessary libraries
IF(APPLE)
  TARGET_LINK_LIBRARIES(test_time iriutils pthread)
ELSE(APPLE)
  TARGET_LINK_LIBRARIES(test_time iriutils pthread rt)
ENDIF(APPLE)

# edit the following line to add the source code for the example and the name of the executable
ADD_EXECUTABLE(test_logs test_logs.cpp)

# edit the following line to add the necessary libraries
TARGET_LINK_LIBRARIES(test_logs iriutils pthread)



