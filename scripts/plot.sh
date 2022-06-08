#!/bin/bash
# plots the given input files using graphviz
cpus=$(nproc)
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$#" -eq 0 ]; then
	echo "Usage: $0 <dot-files>"
	exit 1
fi

echo $@ | tr ' ' '\n' | xargs -L 1 -P $cpus $__dir/plot-one.sh
