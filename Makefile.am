ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include
AM_CFLAGS = -std=gnu99

init_ddir = /etc/init.d
init_d_SCRIPTS = scripts/hubo

# installed under $prefix/include/
include_HEADERS = include/Hubo_Tech.h include/control-daemon.h include/daemonizer.h


bin_PROGRAMS = control-daemon balance-test home-test ik-test test-plus


hubo_techdir = $(prefix)/lib
hubo_tech_LTLIBRARIES = libhubotech.la libdaemonizer.la
libdaemonizer_la_SOURCES = src/daemonizer.c
libhubotech_la_SOURCES = src/Hubo_Tech.cpp
libhubotech_la_LIBADD = -lm -lach -lhuboparams

control_daemon_SOURCES = src/control-daemon.c
control_daemon_LDADD = -lm -lach -lc libdaemonizer.la -lhuboparams

balance_test_SOURCES = src/balance-test.cpp
balance_test_LDADD = -lm -lach libhubotech.la -lhuboparams libdaemonizer.la

home_test_SOURCES = src/home-test.cpp
home_test_LDADD = -lm -lach libhubotech.la -lhuboparams libdaemonizer.la

ik_test_SOURCES = src/ik-test.cpp
ik_test_LDADD = -lm -lach libhubotech.la -lhuboparams libdaemonizer.la

test_plus_SOURCES = src/test-plus.cpp
test_plus_LDADD = -lm -lach libhubotech.la -lhuboparams libdaemonizer.la

