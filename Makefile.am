ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -L/usr/local/lib
AM_CFLAGS = -std=gnu99 -L/usr/local/lib

EXTRA_DIST = tables/control.table

init_ddir = /etc/init.d
init_d_SCRIPTS = scripts/hubo-motion

# installed under $prefix/include/
include_HEADERS = include/Hubo_Control.h include/control-daemon.h include/daemonizer.h include/manip.h

bin_PROGRAMS = control-daemon tech-console proto-manip-daemon

huboctrlconfdir=/etc/hubo-ach
huboctrlconf_DATA = tables/control.table

control_daemon_SOURCES = src/control-daemon.c
control_daemon_LDADD = -lm -lach -lc libdaemonizer.la -lhuboparams

tech_console_SOURCES = src/tech-console.cpp
tech_console_LDADD = -lm -lach -lreadline libhuboctrl.la libdaemonizer.la -lhuboparams

proto_manip_daemon_SOURCES = src/proto-manip-daemon.cpp
proto_manip_daemon_LDADD = -lm -lach libhuboctrl.la libdaemonizer.la -lhuboparams


hubo_ctrldir = /usr/lib
hubo_ctrl_LTLIBRARIES = libhuboctrl.la libdaemonizer.la
libdaemonizer_la_SOURCES = src/daemonizer.c
libdaemonizer_la_LDFLAGS = -module -avoid-version -shared
libhuboctrl_la_SOURCES = src/Hubo_Control.cpp
libhuboctrl_la_LDFLAGS = -module -avoid-version -shared
#libhuboctrl_la_LIBADD = -lm -lach -lhuboparams
