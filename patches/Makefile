Bottom: 1cbd30856ca7158f7c6906330ebe58cc55aa9b2d
Top:    609ff9f254416f29c73b76a65054bfc4432a11f0
Author: Jon Smirl <jonsmirl@gmail.com>
Date:   2009-06-18 09:57:29 -0400

Makefile



---

diff --git a/Makefile b/Makefile
index ea63667..538e558 100644
--- a/Makefile
+++ b/Makefile
@@ -35,6 +35,9 @@ MAKEFLAGS += -rR --no-print-directory
 # To put more focus on warnings, be less verbose as default
 # Use 'make V=1' to see the full commands
 
+ARCH=powerpc
+CROSS_COMPILE=powerpc-603e-linux-gnu-
+
 ifeq ("$(origin V)", "command line")
   KBUILD_VERBOSE = $(V)
 endif
diff --git a/arch/powerpc/boot/.gitignore b/arch/powerpc/boot/.gitignore
index 2f50acd..d57aa5c 100644
--- a/arch/powerpc/boot/.gitignore
+++ b/arch/powerpc/boot/.gitignore
@@ -22,6 +22,7 @@ uImage
 cuImage.*
 dtbImage.*
 treeImage.*
+vmlinux.strip
 zImage
 zImage.initrd
 zImage.bin.*
diff --git a/scripts/basic/.gitignore b/scripts/basic/.gitignore
index bf8b199..d78fbb1 100644
--- a/scripts/basic/.gitignore
+++ b/scripts/basic/.gitignore
@@ -1,3 +1,4 @@
 hash
 fixdep
 docproc
+hash
