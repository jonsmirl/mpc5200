Bottom: 3056d4ffeecd171cf9f091437e721d2556e63c6d
Top:    a8eb4864e9e7ba0af1bf59b12a14b7bc305cc5f9
Author: Jon Smirl <jonsmirl@gmail.com>
Date:   2008-10-30 23:44:05 -0400

Makefile



---

diff --git a/Makefile b/Makefile
index d500817..86e3e8e 100644
--- a/Makefile
+++ b/Makefile
@@ -35,6 +35,9 @@ MAKEFLAGS += -rR --no-print-directory
 # To put more focus on warnings, be less verbose as default
 # Use 'make V=1' to see the full commands
 
+ARCH=powerpc
+CROSS_COMPILE=powerpc-603e-linux-gnu-
+
 ifdef V
   ifeq ("$(origin V)", "command line")
     KBUILD_VERBOSE = $(V)
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
