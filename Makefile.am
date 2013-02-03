ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib
AM_CFLAGS = -std=gnu99 -L/usr/local/lib

EXTRA_DIST = tables/control.table

init_ddir = /etc/init.d
init_d_SCRIPTS = scripts/tech

# installed under $prefix/include/
include_HEADERS = include/Hubo_Tech.h include/control-daemon.h include/daemonizer.h include/manip.h

bin_PROGRAMS = control-daemon tech-console proto-manip-daemon

hubotechconfdir=/etc/hubo-ach
hubotechconf_DATA = tables/control.table

control_daemon_SOURCES = src/control-daemon.c
control_daemon_LDADD = -lm -lach -lc libdaemonizer.la -lhuboparams

tech_console_SOURCES = src/tech-console.cpp
tech_console_LDADD = -lm -lach -lreadline libhubotech.la libdaemonizer.la -lhuboparams

proto_manip_daemon_SOURCES = src/proto-manip-daemon.cpp
proto_manip_daemon_LDADD = -lm -lach libhubotech.la libdaemonizer.la -lhuboparams


hubo_techdir = $(prefix)/lib
hubo_tech_LTLIBRARIES = libhubotech.la libdaemonizer.la
libdaemonizer_la_SOURCES = src/daemonizer.c
libdaemonizer_la_LDFLAGS = -module -avoid-version -shared
libhubotech_la_SOURCES = src/Hubo_Tech.cpp
libhubotech_la_LDFLAGS = -module -avoid-version -shared
#libhubotech_la_LIBADD = -lm -lach -lhuboparams
