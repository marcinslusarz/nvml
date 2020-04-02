#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2016-2020, Intel Corporation

#
# run-build.sh - is called inside a Docker container; prepares the environment
#                and starts a build of PMDK project.
#

set -e

# Prepare build environment
./prepare-for-build.sh

# Build all and run tests
cd $WORKDIR
git config --global http.sslVerify false
git clone https://github.com/brendangregg/FlameGraph.git
#echo $USERPASS | sudo -S apt update || true
#echo $USERPASS | sudo -S apt -y install linux-tools-generic || true
#echo $USERPASS | sudo -S apt -y install linux-tools-azure || true
#echo $USERPASS | sudo -S apt -y install linux-tools-`uname -r` || true
#echo $USERPASS | sudo -S dnf -y install perf || true

whereis perf

echo $USERPASS | sudo -S chmod a+r /proc/kallsyms || true

if [ "$SRC_CHECKERS" != "0" ]; then
	make -j$(nproc) check-license
	make -j$(nproc) cstyle
fi

make -j$(nproc) BUILD_EXAMPLES=n BUILD_BENCHMARKS=n
make -j$(nproc) test  BUILD_EXAMPLES=n BUILD_BENCHMARKS=n
cd src/test
cp tools/obj_verify/obj_verify.static-debug tools/obj_verify/obj_verify
make sync-remotes  BUILD_EXAMPLES=n BUILD_BENCHMARKS=n

cd pmempool_sync_remote
make TEST28 TEST_BUILD=$TEST_BUILD
#echo "STARTXXX"
#cat res.svg
#echo "ENDXXX"
