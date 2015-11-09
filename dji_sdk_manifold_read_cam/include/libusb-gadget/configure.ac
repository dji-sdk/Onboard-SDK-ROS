AC_INIT([libusb-gadget], [0.0.0], [ueno@unixuser.org])
AM_INIT_AUTOMAKE
AC_PROG_CC
AC_CONFIG_MACRO_DIR([m4])
LT_INIT

AC_CHECK_HEADERS([linux/usb/gadgetfs.h],
  [AC_DEFINE([HAVE_LINUX_USB_SUBDIR], 1,
    [Linux USB headers are arranged in a dedicated directory.])],
    dnl Don't check the existence of linux/usb_gadgetfs.h here to
    dnl ignore autoconf cache of the above check.
  [AC_CHECK_HEADERS([linux/usb_ch9.h], ,
    [AC_MSG_ERROR([Can't find linux/usb/gadgetfs.h nor linux/usb_gadgetfs.h.])])])

pkgconfigdir='${libdir}/pkgconfig'
AC_SUBST(pkgconfigdir)
PKG_CHECK_MODULES([libusb], [libusb >= 0.1.12])
AC_CONFIG_HEADERS([src/config.h])
AC_CONFIG_FILES([Makefile
libusb-gadget.pc
src/Makefile
tests/Makefile
scripts/Makefile])
AC_OUTPUT
