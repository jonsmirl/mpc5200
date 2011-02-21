#!/bin/bash
VERS=`cat include/config/kernel.release`
rm -rf /tmp/kernel-dist-$VERS && \
mkdir -p /tmp/kernel-dist-$VERS && \
cp -rf usr/include /tmp/kernel-dist-$VERS/ && \
cp /boot/*$VERS /tmp/kernel-dist-$VERS/ && \
cp -rp /lib/modules/$VERS /tmp/kernel-dist-$VERS && \
cp include/config/kernel.release /tmp/kernel-dist-$VERS && \
cp install_distribution.sh /tmp/kernel-dist-$VERS && \
cd /tmp && \
tar -czf kernel-dist-$VERS.tgz kernel-dist-$VERS && \
cd - && \
rm -rf /tmp/kernel-dist-$VERS
