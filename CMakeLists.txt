cmake_minimum_required(VERSION 2.8.9)
set(PROJECT_NAME_STR SensorFusion)
project(${PROJECT_NAME_STR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED COMPONENTS system thread)
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})

set(DSM_LIBRARIES ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${LINUX_LIBRARIES})
set(DSM_SOURCES "${PROJECT_SOURCE_DIR}/src/Dependencies/DistributedSharedMemory/src/Client/DSMClient.cpp" "${PROJECT_SOURCE_DIR}/src/Dependencies/DistributedSharedMemory/src/Shared/DSMBase.cpp")

file(GLOB DATACORE_SOURCES "${PROJECT_SOURCE_DIR}/src/DataCore/*.cpp")
set(DATACORE_TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/DataCore_test.cpp")

# DataCore target
set(PROJECT_DATACORE_NAME DataCore)
add_executable(${PROJECT_DATACORE_NAME} ${DATACORE_SOURCES} ${DSM_SOURCES})
target_link_libraries(${PROJECT_DATACORE_NAME} ${DSM_LIBRARIES})

# DataCore test target
set(PROJECT_DATACORE_TEST_NAME DataCore_test)
add_executable(${PROJECT_DATACORE_TEST_NAME} ${DATACORE_TEST_SOURCES} ${DSM_SOURCES})
target_link_libraries(${PROJECT_DATACORE_TEST_NAME} ${DSM_LIBRARIES})
