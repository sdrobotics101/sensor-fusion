cmake_minimum_required(VERSION 2.8.9)
set(PROJECT_NAME_STR SensorFusion)
project(${PROJECT_NAME_STR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")

find_package(Boost REQUIRED COMPONENTS system thread program_options OPTIONAL_COMPONENTS log date_time)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

if(UNIX AND NOT APPLE)
    set(LINUX_LIBRARIES rt pthread)
else()
     set(LINUX_LIBRARIES )
endif()

set(DSM_LIBRARIES ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${LINUX_LIBRARIES})
set(DSM_SOURCES "${PROJECT_SOURCE_DIR}/src/Dependencies/DistributedSharedMemory/src/Client/DSMClient.cpp" "${PROJECT_SOURCE_DIR}/src/Dependencies/DistributedSharedMemory/src/Shared/DSMBase.cpp")

if(Boost_LOG_FOUND AND Boost_DATE_TIME_FOUND)
    set(LOG_LIBRARIES ${Boost_LOG_LIBRARY} ${Boost_DATE_TIME_LIBRARY})
    set(LOG_DEPENDENCIES true)
else()
    set(LOG_DEPENDENCIES false)
endif()

file(GLOB DATACORE_SOURCES "${PROJECT_SOURCE_DIR}/src/DataCore/*.cpp")
set(DATACORE_TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/DataCore_test.cpp")

file(GLOB FUSIONCORE_SOURCES "${PROJECT_SOURCE_DIR}/src/FusionCore/*.cpp")

# DataCore target
set(PROJECT_DATACORE_NAME DataCore)
add_executable(${PROJECT_DATACORE_NAME} ${DATACORE_SOURCES} ${DSM_SOURCES})
option(DISABLE_LOGGING OFF)
if(LOG_DEPENDENCIES AND NOT DISABLE_LOGGING)
    target_compile_definitions(${PROJECT_DATACORE_NAME} PUBLIC BOOST_LOG_DYN_LINK LOGGING_ENABLED)
    target_link_libraries(${PROJECT_DATACORE_NAME} ${DSM_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${LOG_LIBRARIES})
else()
    target_link_libraries(${PROJECT_DATACORE_NAME} ${DSM_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

# DataCore test target
set(PROJECT_DATACORE_TEST_NAME DataCore_test)
add_executable(${PROJECT_DATACORE_TEST_NAME} ${DATACORE_TEST_SOURCES} ${DSM_SOURCES})
target_link_libraries(${PROJECT_DATACORE_TEST_NAME} ${DSM_LIBRARIES})

set(PROJECT_FUSIONCORE_NAME FusionCore)
add_executable(${PROJECT_FUSIONCORE_NAME} ${FUSIONCORE_SOURCES} ${DSM_SOURCES})
target_link_libraries(${PROJECT_FUSIONCORE_NAME} ${DSM_LIBRARIES})
