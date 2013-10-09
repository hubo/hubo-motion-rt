cmake_minimum_required(VERSION 2.6)
project(hubo-motion-rt)

set(CMAKE_BUILD_TYPE "Release")

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

include_directories(include)

# install path variables
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_INSTALL_PATH /usr/lib/)
set(BINARY_INSTALL_PATH /usr/local/bin/)

set(CMAKE_C_FLAGS "-std=gnu99")

#find_package(ach)
#NOTE: This dummy business should be addressed
find_path(REFLEX_INCLUDE "reflex.h" PATHS ${CMAKE_INCLUDE_PATH})

message(${REFLEX_INCLUDE})

if(${REFLEX_INCLUDE} STREQUAL REFLEX_INCLUDE-NOTFOUND)

  message("\n -- INFO: Could not find reflex includes. We will NOT slerp with reflex/amino\n")
  set(HAVE_REFLEX 0)

else() # Reflex not found

  message("\n -- Found reflex includes. We will slerp with reflex/amino\n")
  add_definitions(-DHAVE_REFLEX)
  set(HAVE_REFLEX 1)

endif() # Reflex not found


set( CMAKE_VERBOSE_MAKEFILE true )

#link_libraries(m rt ach huboparams)
set(urdfdom_DIR "/opt/ros/groovy/share/urdfdom/cmake/")
find_package( urdfdom REQUIRED )

file(GLOB daemonizer_source "src/daemonizer.c" "include/daemonizer.h")

file(GLOB control_daemon_source "src/control-daemon.c"
                                "include/control-daemon.h"
                                "tables/control.table"
                                "tables/duty.table"
                                "tables/torque.table")

#link_libraries(RobotKin)

file(GLOB drc_hubo_kin_source "src/DrcHuboKin.cpp" "include/DrcHuboKin.h")

file(GLOB hubo_control_source "src/Hubo_Control.cpp" "include/Hubo_Control.h")


file(GLOB balance_daemon_source "src/balance-daemon.cpp"
                                "include/balance-daemon.h"
                                "src/Walker.cpp"
                                "include/Walker.h"
                                "src/ImpedanceController.cpp"
                                "include/ImpedanceController.h"
                                "include/hubo-zmp.h" )

file(GLOB manip_daemon_source "src/manip-daemon.cpp"
                              "include/manip.h"
                              "src/Slerper.cpp"
                              "include/Slerper.h" )

file(GLOB gravityTest_source "src/gravityTest.cpp")

file(GLOB trajTest_source "src/trajTest.cpp")

file(GLOB calibrate_source "src/calibrate.cpp"
                           "src/calibrationAPI.cpp"
                           "include/calibration.h")

file(GLOB manipTest_source "src/manipTest.cpp")

add_library(daemonizer SHARED ${daemonizer_source})
#link_libraries(daemonizer)

add_executable(control-daemon ${control_daemon_source})
target_link_libraries(control-daemon m rt ach huboparams daemonizer)

add_library(huboctrl SHARED ${hubo_control_source})
target_link_libraries(huboctrl ach daemonizer)
#link_libraries(huboctrl)

add_library(drchubokin SHARED ${drc_hubo_kin_source})
#link_libraries(drchubokin)
target_link_libraries(drchubokin RobotKin)

add_executable(balance-daemon ${balance_daemon_source})
target_link_libraries(balance-daemon m rt ach huboparams RobotKin ${urdfdom_LIBRARIES} daemonizer drchubokin huboctrl)

add_executable(manip-daemon ${manip_daemon_source})
if(${REFLEX_INCLUDE} STREQUAL REFLEX_INCLUDE-NOTFOUND)
  target_link_libraries(manip-daemon m rt ach huboparams RobotKin daemonizer drchubokin huboctrl)
else() # Reflex not found
  target_link_libraries(manip-daemon m rt ach huboparams RobotKin daemonizer drchubokin huboctrl reflex amino)
endif()  # Reflex not found

add_executable(gravityTest ${gravityTest_source})
target_link_libraries(gravityTest drchubokin huboctrl RobotKin m rt ach huboparams daemonizer)

add_executable(trajTest ${trajTest_source})
target_link_libraries(trajTest drchubokin huboctrl RobotKin m rt ach huboparams daemonizer)

add_executable(calibrate ${calibrate_source})
target_link_libraries(calibrate drchubokin huboctrl RobotKin m rt ach huboparams daemonizer)

add_executable(manipTest ${manipTest_source})
target_link_libraries(manipTest drchubokin huboctrl RobotKin m rt ach huboparams daemonizer)


add_executable(imp-test "src/imp-test.cpp" "src/ImpedanceController.cpp" "include/ImpedanceController.h" "include/KeyboardInput.h")

# Install Libraries
INSTALL(TARGETS daemonizer
                huboctrl
        DESTINATION ${LIBRARY_INSTALL_PATH})

# Install Headers
INSTALL(FILES include/Hubo_Control.h
              include/daemonizer.h
              include/control-daemon.h 
              include/balance-daemon.h
              include/manip.h
              include/hubo-zmp.h
              include/DrcHuboKin.h
              include/motion-trajectory.h
        DESTINATION /usr/local/include/)

# Install Tables
INSTALL(FILES tables/control.table  DESTINATION /etc/hubo-ach/)
#INSTALL(FILES tables/duty.table DESTINATION /etc/hubo-ach/)
#INSTALL(FILES tables/torque.table   DESTINATION /etc/hubo-ach/)

# Install Service
INSTALL(PROGRAMS scripts/hubo-motion DESTINATION /etc/init.d/)

# Install Daemons
INSTALL(PROGRAMS control-daemon
                 balance-daemon
                 manip-daemon
        DESTINATION /usr/local/bin/)
