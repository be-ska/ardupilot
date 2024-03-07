#!/bin/bash
# build for all copters boards

set -e
set -x

rm -rf align-build
mkdir align-build

BOARDS="AP3-M460 AP3-M460-dev AP3-M460-G3P"

for b in $BOARDS; do
    echo "Testing $b build"
    ./waf configure --board $b
    ./waf clean
    ./waf copter
    cp build/$b/bin/arducopter.apj align-build/$b.apj
done

echo "align builds completed"

exit 0
