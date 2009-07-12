Bottom: cfafc7c6c9f169a7a308c290363dde765e21735d
Top:    d0a00309f3d81acc768325ad1d49e6083875f61c
Author: Jon Smirl <jonsmirl@gmail.com>
Date:   2009-07-12 19:54:26 -0400

Makefile



---

diff --git a/Makefile b/Makefile
index bbe8453..f760fc0 100644
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
index 3d80c3e..dab1b41 100644
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
