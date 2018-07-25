#!/bin/sh
set -e
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# build the verilator tests
make verilate
# run the tests in parallel, 4 at a time
printf "$(xargs printf '\n%s' < $ROOT/ci/test.list | cut -b 1-)" | xargs -n1 -P4 -I{} $ROOT/obj_dir/Variane_testharness tmp/riscv-tests/build/isa/{}
