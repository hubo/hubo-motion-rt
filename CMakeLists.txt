cmake_minimum_required(VERSION 2.6)
project(hubo-motion-rt)

include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

set(CMAKE_C_FLAGS "-std=gnu99")

#find_package(ach)
link_libraries(m rt ach huboparams RobotKin)

file(GLOB daemonizer_source "src/daemonizer.c" "include/daemonizer.h")

file(GLOB drc_hubo_kin_source "src/DrcHuboKin.cpp" "include/DrcHuboKin.h")

file(GLOB control_daemon_source "src/control-daemon.c" "include/control-daemon.h" "tables/control.table")

file(GLOB hubo_control_source "src/Hubo_Control.cpp" "include/Hubo_Control.h")


file(GLOB balance_daemon_source "src/balance-daemon.cpp"
                                "src/Walker.cpp"
                                "include/balance-daemon.h"
                                "include/Walker.h"
                                "include/hubo-zmp.h" )

file(GLOB manip_daemon_source "src/manip-daemon.cpp"
                              "include/manip.h" )

add_library(daemonizer SHARED ${daemonizer_source})
link_libraries(daemonizer)

add_executable(control-daemon ${control_daemon_source})

add_library(huboctrl SHARED ${hubo_control_source})
link_libraries(huboctrl)

add_library(drchubokin SHARED ${drc_hubo_kin_source})
link_libraries(drchubokin)

add_executable(balance-daemon ${balance_daemon_source})

add_executable(manip-daemon ${manip_daemon_source})


