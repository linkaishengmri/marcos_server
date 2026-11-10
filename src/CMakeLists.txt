cmake_minimum_required(VERSION 3.5.1)
project(marcos_server)

find_package(Threads REQUIRED)
set(MARCOS_THREAD_LIBRARIES Threads::Threads)

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL armv7l)
   # compile in release mode on the ARM
   set (CMAKE_C_FLAGS "-static -O3 -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-static -O3 -mcpu=cortex-a9 -mtune=cortex-a9 -mfpu=neon -mfloat-abi=hard -Wall -std=c++17")
   # The target libstdc++ uses weak pthread references. A plain -lpthread
   # leaves join/detach out of the static executable even though create
   # works. Pull in the full archive so the connection reaper can join.
   list(APPEND MARCOS_THREAD_LIBRARIES
        "-Wl,--whole-archive" pthread "-Wl,--no-whole-archive")
else()
   # compile in debug mode on the host PC for testing
   set (CMAKE_C_FLAGS "-O0 -g -Wall -std=c11")
   set (CMAKE_CXX_FLAGS "-O0 -g -Wall -std=c++17")
   # set (CMAKE_C_FLAGS "-static -O3 -Wall -std=c11")
   # set (CMAKE_CXX_FLAGS "-static -O3 -std=c++14")   
endif()

set(SERVER_SRC marcos_server.cpp hardware.cpp iface.cpp mpack/mpack.c)
add_executable(marcos_server ${SERVER_SRC})
target_link_libraries(marcos_server PRIVATE ${MARCOS_THREAD_LIBRARIES})
