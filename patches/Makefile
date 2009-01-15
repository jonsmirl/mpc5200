Bottom: 00df86ebd8b598b5db3c4fe4612c6614409caf94
Top:    3168fc92d27cae943053875b7101836807ad0f0e
Author: Jon Smirl <jonsmirl@gmail.com>
Date:   2009-01-15 18:02:10 -0500

Makefile



---

diff --git a/Makefile b/Makefile
index c06e250..ad2a908 100644
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
