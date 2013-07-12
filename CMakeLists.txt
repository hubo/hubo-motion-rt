cmake_minimum_required(VERSION 2.6)
project(hubo-motion-rt)

include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

set(CMAKE_C_FLAGS "-std=gnu99")

#find_package(ach)
link_libraries(-lm -lrt -lach -lhuboparams -lRobotKin)

file(GLOB daemonizer_source "src/daemonizer.c" "include/daemonizer.h")

file(GLOB drc_hubo_kin_source "src/DrcHuboKin.cpp" "include/DrcHuboKin.cpp")

file(GLOB control_daemon_source "src/control-daemon.c" "include/control-daemon.h" "tables/control.table")

file(GLOB hubo_control_source "src/Hubo_Control.cpp" "include/Hubo_Control.h")


file(GLOB balance_daemon_source "src/balance-daemon.cpp"
                                "src/Walker.cpp"
                                "include/balance-daemon.h"
                                "include/hubo-zmp.h" )

file(GLOB manip_daemon_source "src/manip-daemon.cpp"
                              "include/manip.h" )

add_library(libdaemonizer SHARED ${daemonizer_source})
link_libraries(libdaemonizer)

add_executable(control-daemon ${control_daemon_source})

add_library(libhuboctrl SHARED ${hubo_control_source})
link_libraries(libhuboctrl)

add_library(libdrchubokin SHARED ${drc_hubo_kin_source})
link_libraries(libdrchubokin)

add_executable(balance-daemon ${balance_daemon_source})

add_executable(manip-daemon ${manip_daemon_source})


