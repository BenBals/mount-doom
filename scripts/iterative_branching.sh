#! /bin/bash

echo "execute me in your build folder"
mkdir -p ../solutions
for f in ../instances/exact_public/* ; do
    echo "starting $f"
    timeout 1m ./src/doom --branching --matching "$(basename $f)" > "../solutions/$(basename $f)"
done
