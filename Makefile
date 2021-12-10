#############################################################################
# Makefile for building: myViewer
# Generated by qmake (3.1) (Qt 5.12.8)
# Project:  GPGPU_TP.pro
# Template: subdirs
# Command: /usr/lib/qt5/bin/qmake -o Makefile GPGPU_TP.pro
#############################################################################

MAKEFILE      = Makefile

EQ            = =

first: make_first
QMAKE         = /usr/lib/qt5/bin/qmake
DEL_FILE      = rm -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p
COPY          = cp -f
COPY_FILE     = cp -f
COPY_DIR      = cp -f -R
INSTALL_FILE  = install -m 644 -p
INSTALL_PROGRAM = install -m 755 -p
INSTALL_DIR   = cp -f -R
QINSTALL      = /usr/lib/qt5/bin/qmake -install qinstall
QINSTALL_PROGRAM = /usr/lib/qt5/bin/qmake -install qinstall -exe
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
TAR           = tar -cf
COMPRESS      = gzip -9f
DISTNAME      = myViewer1.0.0
DISTDIR = /user/5/.base/mathaiar/home/3A/GPGPU_TP/.tmp/myViewer1.0.0
SUBTARGETS    =  \
		sub-trimesh2 \
		sub-viewer


sub-trimesh2-qmake_all:  FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro
	cd trimesh2/ && $(MAKE) -f Makefile qmake_all
sub-trimesh2: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile
sub-trimesh2-make_first-ordered: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile 
sub-trimesh2-make_first: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile 
sub-trimesh2-all-ordered: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile all
sub-trimesh2-all: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile all
sub-trimesh2-clean-ordered: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile clean
sub-trimesh2-clean: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile clean
sub-trimesh2-distclean-ordered: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile distclean
sub-trimesh2-distclean: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile distclean
sub-trimesh2-install_subtargets-ordered: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile install
sub-trimesh2-install_subtargets: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile install
sub-trimesh2-uninstall_subtargets-ordered: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile uninstall
sub-trimesh2-uninstall_subtargets: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile uninstall
sub-viewer-qmake_all: sub-trimesh2-qmake_all FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro
	cd viewer/ && $(MAKE) -f Makefile qmake_all
sub-viewer: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile
sub-viewer-make_first-ordered: sub-trimesh2-make_first-ordered  FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile 
sub-viewer-make_first: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile 
sub-viewer-all-ordered: sub-trimesh2-all-ordered  FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile all
sub-viewer-all: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile all
sub-viewer-clean-ordered: sub-trimesh2-clean-ordered  FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile clean
sub-viewer-clean: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile clean
sub-viewer-distclean-ordered: sub-trimesh2-distclean-ordered  FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile distclean
sub-viewer-distclean: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile distclean
sub-viewer-install_subtargets-ordered: sub-trimesh2-install_subtargets-ordered  FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile install
sub-viewer-install_subtargets: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile install
sub-viewer-uninstall_subtargets-ordered: sub-trimesh2-uninstall_subtargets-ordered  FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile uninstall
sub-viewer-uninstall_subtargets: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile uninstall

Makefile: GPGPU_TP.pro /usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++/qmake.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/spec_pre.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/unix.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/linux.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/sanitize.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/gcc-base.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/gcc-base-unix.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/g++-base.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/g++-unix.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/qconfig.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_accessibility_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_bootstrap_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_concurrent.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_concurrent_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_core.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_core_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_dbus.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_dbus_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_devicediscovery_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_edid_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_egl_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eglfs_kms_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eglfsdeviceintegration_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eventdispatcher_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_fb_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_fontdatabase_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_glx_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_gui.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_gui_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_input_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_kms_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_linuxaccessibility_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_network.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_network_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_opengl.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_opengl_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_openglextensions.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_openglextensions_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_platformcompositor_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_printsupport.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_printsupport_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_service_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_sql.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_sql_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_testlib.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_testlib_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_theme_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_vulkan_support_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_widgets.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_widgets_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xml.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xml_private.pri \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qt_functions.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qt_config.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++/qmake.conf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/spec_post.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/exclusive_builds.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/toolchain.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/default_pre.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/resolve_config.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/default_post.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/warn_on.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qmake_use.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/file_copies.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/testcase_targets.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/exceptions.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/yacc.prf \
		/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/lex.prf \
		GPGPU_TP.pro
	$(QMAKE) -o Makefile GPGPU_TP.pro
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/spec_pre.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/unix.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/linux.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/sanitize.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/gcc-base.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/gcc-base-unix.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/g++-base.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/g++-unix.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/qconfig.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_accessibility_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_bootstrap_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_concurrent.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_concurrent_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_core.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_core_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_dbus.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_dbus_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_devicediscovery_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_edid_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_egl_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eglfs_kms_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eglfsdeviceintegration_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eventdispatcher_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_fb_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_fontdatabase_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_glx_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_gui.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_gui_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_input_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_kms_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_linuxaccessibility_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_network.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_network_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_opengl.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_opengl_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_openglextensions.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_openglextensions_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_platformcompositor_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_printsupport.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_printsupport_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_service_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_sql.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_sql_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_testlib.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_testlib_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_theme_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_vulkan_support_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_widgets.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_widgets_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xml.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xml_private.pri:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qt_functions.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qt_config.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++/qmake.conf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/spec_post.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/exclusive_builds.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/toolchain.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/default_pre.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/resolve_config.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/default_post.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/warn_on.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qmake_use.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/file_copies.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/testcase_targets.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/exceptions.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/yacc.prf:
/usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/lex.prf:
GPGPU_TP.pro:
qmake: FORCE
	@$(QMAKE) -o Makefile GPGPU_TP.pro

qmake_all: sub-trimesh2-qmake_all sub-viewer-qmake_all FORCE

make_first: sub-trimesh2-make_first-ordered sub-viewer-make_first-ordered  FORCE
all: sub-trimesh2-all-ordered sub-viewer-all-ordered  FORCE
clean: sub-trimesh2-clean-ordered sub-viewer-clean-ordered  FORCE
distclean: sub-trimesh2-distclean-ordered sub-viewer-distclean-ordered  FORCE
	-$(DEL_FILE) Makefile
	-$(DEL_FILE) .qmake.stash
install_subtargets: sub-trimesh2-install_subtargets-ordered sub-viewer-install_subtargets-ordered FORCE
uninstall_subtargets: sub-trimesh2-uninstall_subtargets-ordered sub-viewer-uninstall_subtargets-ordered FORCE

sub-trimesh2-check_ordered:
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile check
sub-viewer-check_ordered: sub-trimesh2-check_ordered 
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile check
check: sub-trimesh2-check_ordered sub-viewer-check_ordered

sub-trimesh2-benchmark_ordered:
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -f Makefile benchmark
sub-viewer-benchmark_ordered: sub-trimesh2-benchmark_ordered 
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -f Makefile benchmark
benchmark: sub-trimesh2-benchmark_ordered sub-viewer-benchmark_ordered
install:install_subtargets  FORCE

uninstall: uninstall_subtargets FORCE

FORCE:

dist: distdir FORCE
	(cd `dirname $(DISTDIR)` && $(TAR) $(DISTNAME).tar $(DISTNAME) && $(COMPRESS) $(DISTNAME).tar) && $(MOVE) `dirname $(DISTDIR)`/$(DISTNAME).tar.gz . && $(DEL_FILE) -r $(DISTDIR)

distdir: sub-trimesh2-distdir sub-viewer-distdir FORCE
	@test -d $(DISTDIR) || mkdir -p $(DISTDIR)
	$(COPY_FILE) --parents /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/spec_pre.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/unix.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/linux.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/sanitize.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/gcc-base.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/gcc-base-unix.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/g++-base.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/common/g++-unix.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/qconfig.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_accessibility_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_bootstrap_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_concurrent.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_concurrent_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_core.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_core_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_dbus.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_dbus_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_devicediscovery_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_edid_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_egl_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eglfs_kms_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eglfsdeviceintegration_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_eventdispatcher_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_fb_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_fontdatabase_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_glx_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_gui.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_gui_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_input_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_kms_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_linuxaccessibility_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_network.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_network_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_opengl.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_opengl_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_openglextensions.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_openglextensions_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_platformcompositor_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_printsupport.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_printsupport_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_service_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_sql.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_sql_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_testlib.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_testlib_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_theme_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_vulkan_support_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_widgets.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_widgets_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xcb_qpa_lib_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xml.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/modules/qt_lib_xml_private.pri /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qt_functions.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qt_config.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++/qmake.conf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/spec_post.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/exclusive_builds.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/toolchain.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/default_pre.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/resolve_config.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/default_post.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/warn_on.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/qmake_use.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/file_copies.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/testcase_targets.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/exceptions.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/yacc.prf /usr/lib/x86_64-linux-gnu/qt5/mkspecs/features/lex.prf GPGPU_TP.pro $(DISTDIR)/

sub-trimesh2-distdir: FORCE
	@test -d trimesh2/ || mkdir -p trimesh2/
	cd trimesh2/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/trimesh2/trimesh2.pro ) && $(MAKE) -e -f Makefile distdir DISTDIR=$(DISTDIR)/trimesh2

sub-viewer-distdir: FORCE
	@test -d viewer/ || mkdir -p viewer/
	cd viewer/ && ( test -e Makefile || $(QMAKE) -o Makefile /user/5/.base/mathaiar/home/3A/GPGPU_TP/viewer/viewer.pro ) && $(MAKE) -e -f Makefile distdir DISTDIR=$(DISTDIR)/viewer

