#!/bin/bash
VERS=`cat kernel.release`
sudo cp {config,initrd.img,System.map,vmlinuz}-$VERS /boot/ && \
sudo cp -r $VERS /lib/modules/$VERS && \
sudo mkdir -p /usr/src/linux-headers-$VERS && \
sudo cp -r include /usr/src/linux-headers-$VERS && \
sudo update-grub
