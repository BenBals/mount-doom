#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." >/dev/null 2>&1 && pwd )"

function run_format() {
    local subdir=$1
    find "$DIR/$subdir" -iname '*.h' -o -iname '*.cpp' -exec clang-format -i {} \;
}

run_format "src"
run_format "test"
